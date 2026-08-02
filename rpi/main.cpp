#include <iostream>

#include "SensorWorker.h"
#include "Background.h"
#include "SDKThread.h"
#include "ProcessThread.h"
#include "UDPSender.h"
#include "TCPReceiver.h"
#include "Protocol.h"

// ---- 설정값 (튜닝 필요) ----
#define TCP_PORT        2000
#define UDP_PORT        3000
#define PC_IP           "100.110.120.9"
#define LIDAR_PORT      "/dev/ttyUSB0"
#define LIDAR_BAUD      128000

#define BG_NUM_BINS     360     // 0.5도 단위 (MAX_BINS와 일치해야 함)
#define BG_K            3.0f    // foreground 판정 민감도
#define BG_CALIB_MS     5000    // 캘리브레이션 3초

#define GRID_MAX_RANGE  800     // cm 단위, LiDAR 최대 측정 거리
#define GRID_CELL_SIZE  10      // 칸마다 크기

int main()
{
    // 1. 라이다 센서 + SDK 쓰레드
    Lidar lidar;
    SDKThread sdk_thread(LIDAR_PORT, LIDAR_BAUD, &lidar);

    std::cout << "Starting SDK thread..." << std::endl;
    if (!sdk_thread.start())
    {
        std::cerr << "SDK thread failed to start" << std::endl;
        return -1;
    }

    // 2. 배경 캘리브레이션 (SDK가 스캔을 채워주고 있어야 동작함)
    BackgroundModel background(BG_NUM_BINS, BG_K);
    std::cout << "Calibrating background... (" << BG_CALIB_MS << "ms)" << std::endl;
    background.calibrate(lidar, BG_CALIB_MS);
    std::cout << "Calibration done." << std::endl;

    // 3. UDP 송신자 + ProcessThread (아직 시작은 안 함)
    UDPSender udp_sender(PC_IP, UDP_PORT);
    ProcessThread process_thread(&lidar, &background, GRID_MAX_RANGE, GRID_CELL_SIZE, std::move(udp_sender));


    // 4. TCP - PC로부터 제어 명령 대기
    TCPReceiver tcp_receiver(TCP_PORT);
    std::cout << "Waiting for PC connection..." << std::endl;
    if (!tcp_receiver.WaitConnection())
    {
        std::cerr << "TCP connection failed" << std::endl;
        return -1;
    }
    std::cout << "PC connected." << std::endl;

    // 배경은 캘리브레이션 끝난 직후 딱 한 번만 PC로 보냄
    process_thread.sendBackgroundFrame();

    bool running = false;
    while (true)
    {
        TCPFrame cmd = tcp_receiver.receiveCommand();

        switch (cmd.cmd)
        {
            case TCPCmdType::START:
                if (running)
                {
                    std::cout << "Already running, ignoring START" << std::endl;
                    break;
                }
                process_thread.start();
                running = true;
                std::cout << "Started" << std::endl;
                break;

            case TCPCmdType::STOP:
                if (!running)
                {
                    std::cout << "Already stopped, ignoring STOP" << std::endl;
                    break;
                }
                process_thread.stop();
                running = false;
                std::cout << "Stopped" << std::endl;
                break;

            default:
                std::cerr << "Unknown command: "
                          << static_cast<int>(cmd.cmd) << std::endl;
                break;
        }
    }

    // (참고) 이 아래는 정상적으로 도달하지 않음 - TCPReceiver::receiveCommand()가
    // 연결 끊기면 exit(-1)로 바로 종료시키기 때문. 필요하면 종료 처리 별도로 다듬어야 함.
}