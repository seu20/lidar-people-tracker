#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_opengl3.h"
#include <SDL.h>
#include <SDL_opengl.h>

#include "SharedState.h"
#include "UDPReceiver.h"
#include "TCPSender.h"
#include "Protocol.h"

#include <algorithm>
#include <cmath>
#include <csignal>
#include <iostream>
#include <vector>

#define RPI_IP    "100.69.228.12"   // RPi Tailscale IP - 실제 값으로 교체
#define TCP_PORT  2000
#define UDP_PORT  3000

// ─────────────────────────────────────────────────────────────
//  화면 색상 (흰 배경 테마)
// ─────────────────────────────────────────────────────────────
static const ImU32 COL_CANVAS      = IM_COL32(255, 255, 255, 255);
static const ImU32 COL_RING        = IM_COL32(205, 210, 216, 255);
static const ImU32 COL_RING_AXIS   = IM_COL32(180, 187, 195, 255);
static const ImU32 COL_RING_LABEL  = IM_COL32(130, 140, 150, 255);
static const ImU32 COL_BACKGROUND  = IM_COL32(175, 182, 190, 200);
static const ImU32 COL_POINT       = IM_COL32( 30,  36,  46, 255);
static const ImU32 COL_BOX         = IM_COL32(214,  40,  40, 255);
static const ImU32 COL_BOX_FILL    = IM_COL32(214,  40,  40,  28);
static const ImU32 COL_LABEL       = IM_COL32(150,  30,  30, 255);
static const ImU32 COL_VEL         = IM_COL32(  0, 140,  70, 255);
static const ImU32 COL_ORIGIN      = IM_COL32(  0, 110, 190, 255);

// 시야 모드
enum FovMode { FOV_360 = 0, FOV_180 = 1 };

// 라이다 좌표(m) -> 화면 픽셀 변환
static ImVec2 worldToScreen(float x, float y, ImVec2 origin, float scale)
{
    return ImVec2(origin.x + y * scale, origin.y - x * scale);  // x: 전방, y: 좌우
}

// 이 좌표가 현재 시야에 들어오는가 (전방 = x >= 0)
static inline bool inFov(int fov_mode, float x)
{
    return (fov_mode == FOV_360) || (x >= 0.0f);
}

// 줌 배율에 맞춰 링 간격을 고름 (화면상 최소 45px 간격 유지)
static float pickRingStep(float scale)
{
    static const float kSteps[] = { 0.25f, 0.5f, 1.0f, 2.0f, 5.0f, 10.0f, 20.0f, 50.0f };
    for (float s : kSteps) {
        if (s * scale >= 45.0f) return s;
    }
    return 50.0f;
}

// 트랙 주변 foreground 점들로 만든 bounding box
struct Box {
    float minx = 0.0f, maxx = 0.0f, miny = 0.0f, maxy = 0.0f;
    int   count = 0;
};

int main(int, char**)
{
    signal(SIGPIPE, SIG_IGN);   // RPi 연결이 끊겨도 send()가 프로세스를 죽이지 않게

    // ---- SDL2 + OpenGL 초기화 ----
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << std::endl;
        return -1;
    }
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);

    SDL_Window *window = SDL_CreateWindow("LiDAR Viewer",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 800,
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, gl_context);
    SDL_GL_SetSwapInterval(1);

    // ---- ImGui 초기화 ----
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsLight();
    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init("#version 150");

    // ---- 네트워크 초기화 ----
    SharedState shared_state;
    UDPReceiver udp_receiver(UDP_PORT, &shared_state);
    udp_receiver.start();

    TCPSender tcp_Sender(RPI_IP, TCP_PORT);
    bool is_running = false;
    bool link_lost  = false;

    int   fov_mode    = FOV_360;
    float scale       = 40.0f;   // 픽셀/미터
    float point_size  = 3.5f;
    float assign_r    = 0.6f;    // box 계산 시 트랙에 점을 붙이는 반경 (m)
    bool  show_rings  = true;
    bool  show_bg     = true;
    bool  show_vel    = true;
    bool  app_running = true;

    while (app_running)
    {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) app_running = false;
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        ViewState snapshot = shared_state.getSnapshot();

        // ── 트랙별 bounding box 계산 ──────────────────────────
        // RPi는 centroid만 보내므로, 뷰어가 받은 foreground 점들을
        // 가장 가까운 트랙에 붙여서 외곽 상자를 만든다.
        std::vector<Box> boxes(snapshot.objects.size());
        if (!snapshot.objects.empty())
        {
            const float r2 = assign_r * assign_r;
            for (const auto &pt : snapshot.points)
            {
                int   best  = -1;
                float bestd = r2;
                for (size_t k = 0; k < snapshot.objects.size(); ++k)
                {
                    float dx = pt.x - snapshot.objects[k].x;
                    float dy = pt.y - snapshot.objects[k].y;
                    float d  = dx * dx + dy * dy;
                    if (d < bestd) { bestd = d; best = static_cast<int>(k); }
                }
                if (best < 0) continue;

                Box &b = boxes[static_cast<size_t>(best)];
                if (b.count == 0) {
                    b.minx = b.maxx = pt.x;
                    b.miny = b.maxy = pt.y;
                } else {
                    b.minx = std::min(b.minx, pt.x);
                    b.maxx = std::max(b.maxx, pt.x);
                    b.miny = std::min(b.miny, pt.y);
                    b.maxy = std::max(b.maxy, pt.y);
                }
                ++b.count;
            }
        }

        // ---- 컨트롤 패널 ----
        ImGui::Begin("Control");

        if (ImGui::Button(is_running ? "STOP" : "START"))
        {
            if (is_running) {
                if (tcp_Sender.sendStop())  { is_running = false; link_lost = false; }
                else                        { link_lost  = true; }
            } else {
                if (tcp_Sender.sendStart()) { is_running = true;  link_lost = false; }
                else                        { link_lost  = true; }
            }
        }
        ImGui::SameLine();
        ImGui::TextColored(is_running ? ImVec4(0.0f, 0.55f, 0.1f, 1.0f) : ImVec4(0.8f, 0.1f, 0.1f, 1.0f),
                           "%s", is_running ? "Running" : "Stopped");

        if (link_lost) {
            ImGui::TextColored(ImVec4(0.85f, 0.35f, 0.0f, 1.0f),
                               "RPi link lost - restart the RPi program, then this viewer");
        }

        // ★ 시야 모드 전환. 라이다 설정은 그대로 두고 화면에서만 자른다.
        const char *fov_names[] = { "360 (full circle)", "180 (front only)" };
        ImGui::Combo("Field of view", &fov_mode, fov_names, 2);

        ImGui::SliderFloat("Zoom (px/m)",    &scale, 2.0f, 100.0f);
        ImGui::SliderFloat("Point size",     &point_size, 1.0f, 8.0f);
        ImGui::SliderFloat("Box radius (m)", &assign_r, 0.2f, 1.5f);
        ImGui::Checkbox("Range rings", &show_rings); ImGui::SameLine();
        ImGui::Checkbox("Background",  &show_bg);    ImGui::SameLine();
        ImGui::Checkbox("Velocity",    &show_vel);

        // 시야 안에 들어오는 개수만 세기
        size_t vis_pts = 0, vis_objs = 0;
        for (const auto &p : snapshot.points)  if (inFov(fov_mode, p.x)) ++vis_pts;
        for (const auto &o : snapshot.objects) if (inFov(fov_mode, o.x)) ++vis_objs;

        ImGui::Text("Points: %zu / %zu   Objects: %zu / %zu",
                    vis_pts, snapshot.points.size(), vis_objs, snapshot.objects.size());
        ImGui::Text("Background bins: %zu", snapshot.background.size());

        // 트랙별 수치 표 - 속도가 실제로 오는지 확인용
        if (vis_objs > 0 && ImGui::BeginTable("tracks", 5,
                ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit))
        {
            ImGui::TableSetupColumn("ID");
            ImGui::TableSetupColumn("x (m)");
            ImGui::TableSetupColumn("y (m)");
            ImGui::TableSetupColumn("speed (m/s)");
            ImGui::TableSetupColumn("size (m)");
            ImGui::TableHeadersRow();

            for (size_t k = 0; k < snapshot.objects.size(); ++k)
            {
                const auto &o = snapshot.objects[k];
                if (!inFov(fov_mode, o.x)) continue;

                const Box &b = boxes[k];
                float speed = std::sqrt(o.vx * o.vx + o.vy * o.vy);

                ImGui::TableNextRow();
                ImGui::TableNextColumn(); ImGui::Text("#%d", o.id);
                ImGui::TableNextColumn(); ImGui::Text("%.2f", o.x);
                ImGui::TableNextColumn(); ImGui::Text("%.2f", o.y);
                ImGui::TableNextColumn(); ImGui::Text("%.2f", speed);
                ImGui::TableNextColumn();
                if (b.count > 0) ImGui::Text("%.2f x %.2f", b.maxx - b.minx, b.maxy - b.miny);
                else             ImGui::TextUnformatted("-");
            }
            ImGui::EndTable();
        }

        ImGui::End();

        // ---- 시각화 캔버스 ----
        ImGui::SetNextWindowPos(ImVec2(0, 280), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(1280, 520), ImGuiCond_FirstUseEver);
        ImGui::Begin("LiDAR View");

        ImDrawList *draw_list  = ImGui::GetWindowDrawList();
        ImVec2 canvas_pos  = ImGui::GetCursorScreenPos();
        ImVec2 canvas_size = ImGui::GetContentRegionAvail();

        // ★ 180도 모드에선 원점을 아래로 내려 전방이 화면 전체를 쓰게 한다
        ImVec2 origin = (fov_mode == FOV_360)
            ? ImVec2(canvas_pos.x + canvas_size.x * 0.5f,
                     canvas_pos.y + canvas_size.y * 0.5f)
            : ImVec2(canvas_pos.x + canvas_size.x * 0.5f,
                     canvas_pos.y + canvas_size.y - 24.0f);

        draw_list->AddRectFilled(canvas_pos,
            ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
            COL_CANVAS);
        draw_list->PushClipRect(canvas_pos,
            ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y), true);

        // ── 거리 링 ───────────────────────────────────────────
        if (show_rings)
        {
            float ring_step = pickRingStep(scale);
            float max_px = 0.5f * std::sqrt(canvas_size.x * canvas_size.x +
                                            canvas_size.y * canvas_size.y);
            if (fov_mode == FOV_180)
                max_px = std::sqrt(canvas_size.x * canvas_size.x * 0.25f +
                                   canvas_size.y * canvas_size.y);
            int ring_count = static_cast<int>(max_px / (ring_step * scale)) + 1;
            if (ring_count > 40) ring_count = 40;

            // 축
            draw_list->AddLine(ImVec2(canvas_pos.x, origin.y),
                               ImVec2(canvas_pos.x + canvas_size.x, origin.y),
                               COL_RING_AXIS, 1.0f);
            draw_list->AddLine(ImVec2(origin.x, canvas_pos.y),
                               ImVec2(origin.x, (fov_mode == FOV_360)
                                                ? canvas_pos.y + canvas_size.y
                                                : origin.y),
                               COL_RING_AXIS, 1.0f);

            for (int k = 1; k <= ring_count; ++k)
            {
                float r_m  = ring_step * k;
                float r_px = r_m * scale;

                if (fov_mode == FOV_360) {
                    draw_list->AddCircle(origin, r_px, COL_RING, 0, 1.0f);
                } else {
                    // 화면 위쪽 반원 (ImGui 각도: 0=오른쪽, 시계방향 증가)
                    draw_list->PathArcTo(origin, r_px, IM_PI, 2.0f * IM_PI, 64);
                    draw_list->PathStroke(COL_RING, 0, 1.0f);
                }

                char buf[16];
                if (ring_step < 1.0f) snprintf(buf, sizeof(buf), "%.2fm", r_m);
                else                  snprintf(buf, sizeof(buf), "%.0fm",  r_m);
                draw_list->AddText(ImVec2(origin.x + 5.0f, origin.y - r_px - 15.0f),
                                   COL_RING_LABEL, buf);
            }
        }

        // ── 배경 (회색 점) ────────────────────────────────────
        if (show_bg)
        {
            size_t bin_count = snapshot.background.size();
            for (size_t i = 0; i < bin_count; ++i)
            {
                float dist = snapshot.background[i];
                if (dist <= 0.0f || dist >= 15.0f) continue;   // 미측정/fallback bin 제외

                float angle = (2.0f * (float)M_PI * (float)i) / (float)bin_count;
                float wx = dist * std::cos(angle);
                float wy = dist * std::sin(angle);
                if (!inFov(fov_mode, wx)) continue;

                draw_list->AddCircleFilled(worldToScreen(wx, wy, origin, scale),
                                           point_size * 0.55f, COL_BACKGROUND);
            }
        }

        // ── foreground 포인트 ─────────────────────────────────
        for (const auto &pt : snapshot.points)
        {
            if (!inFov(fov_mode, pt.x)) continue;
            draw_list->AddCircleFilled(worldToScreen(pt.x, pt.y, origin, scale),
                                       point_size, COL_POINT);
        }

        // ── 추적 객체: 사각형 + 라벨 + 속도 ───────────────────
        for (size_t k = 0; k < snapshot.objects.size(); ++k)
        {
            const auto &obj = snapshot.objects[k];
            if (!inFov(fov_mode, obj.x)) continue;

            const Box &b = boxes[k];

            float minx, maxx, miny, maxy;
            if (b.count > 0) {
                const float PAD = 0.06f;                 // 여유 6cm
                minx = b.minx - PAD; maxx = b.maxx + PAD;
                miny = b.miny - PAD; maxy = b.maxy + PAD;
            } else {
                const float HALF = 0.25f;                // 점이 안 붙었을 때 기본 크기
                minx = obj.x - HALF; maxx = obj.x + HALF;
                miny = obj.y - HALF; maxy = obj.y + HALF;
            }

            // 화면 좌표에서 좌상단 = (maxx, miny), 우하단 = (minx, maxy)
            ImVec2 p_min = worldToScreen(maxx, miny, origin, scale);
            ImVec2 p_max = worldToScreen(minx, maxy, origin, scale);

            draw_list->AddRectFilled(p_min, p_max, COL_BOX_FILL, 2.0f);
            draw_list->AddRect(p_min, p_max, COL_BOX, 2.0f, 0, 2.0f);

            // 라벨: ID + 속도 (상자 위쪽)
            float speed = std::sqrt(obj.vx * obj.vx + obj.vy * obj.vy);
            char label[48];
            snprintf(label, sizeof(label), "#%d  %.2f m/s", obj.id, speed);
            draw_list->AddText(ImVec2(p_min.x, p_min.y - 17.0f), COL_LABEL, label);

            // 속도 화살표: 1초 뒤 예상 위치까지
            if (show_vel && speed > 0.05f)
            {
                ImVec2 c   = worldToScreen(obj.x, obj.y, origin, scale);
                ImVec2 tip = worldToScreen(obj.x + obj.vx, obj.y + obj.vy, origin, scale);
                draw_list->AddLine(c, tip, COL_VEL, 2.5f);

                float ang = std::atan2(tip.y - c.y, tip.x - c.x);
                const float H = 8.0f, S = 0.45f;
                draw_list->AddLine(tip, ImVec2(tip.x - H * std::cos(ang - S),
                                               tip.y - H * std::sin(ang - S)), COL_VEL, 2.5f);
                draw_list->AddLine(tip, ImVec2(tip.x - H * std::cos(ang + S),
                                               tip.y - H * std::sin(ang + S)), COL_VEL, 2.5f);
            }
        }

        // ── 센서 원점 ─────────────────────────────────────────
        draw_list->AddCircleFilled(origin, 5.0f, COL_ORIGIN);
        draw_list->PopClipRect();

        ImGui::End();

        // ---- 렌더 ----
        ImGui::Render();
        glViewport(0, 0, (int)ImGui::GetIO().DisplaySize.x, (int)ImGui::GetIO().DisplaySize.y);
        glClearColor(0.90f, 0.91f, 0.92f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    }

    // ---- 종료 처리 ----
    if (is_running) tcp_Sender.sendStop();
    udp_receiver.stop();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}