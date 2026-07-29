CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O2 -g

# 내 프로젝트 폴더들만 자동으로 -I 잡기 (third_party 제외)
INCLUDE_DIRS := $(shell find . -type d -not -path './build*' -not -path './third_party*' -not -path './.git*' -not -path './.vscode*')
CXXFLAGS += $(addprefix -I,$(INCLUDE_DIRS))
CXXFLAGS += -Ithird_party/imgui -Ithird_party/imgui/backends
CXXFLAGS += $(shell pkg-config --cflags sdl2)

LDFLAGS := $(shell pkg-config --libs sdl2) -lGL -ldl -pthread

TARGET := lidar_viewer
BUILD_DIR := build

# 내가 짠 소스만 (third_party 통째로는 빼고, 필요한 ImGui 파일만 따로 추가)
PROJECT_SRCS := $(patsubst ./%,%,$(shell find . -name "*.cpp" -not -path './build/*' -not -path './third_party/*'))

# ImGui 코어 + SDL2/OpenGL3 백엔드만 (예제/다른 백엔드는 제외)
IMGUI_SRCS := third_party/imgui/imgui.cpp \
              third_party/imgui/imgui_draw.cpp \
              third_party/imgui/imgui_tables.cpp \
              third_party/imgui/imgui_widgets.cpp \
              third_party/imgui/imgui_demo.cpp \
              third_party/imgui/backends/imgui_impl_sdl2.cpp \
              third_party/imgui/backends/imgui_impl_opengl3.cpp

SRCS := $(PROJECT_SRCS) $(IMGUI_SRCS)
OBJS := $(addprefix $(BUILD_DIR)/,$(SRCS:.cpp=.o))

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $@ $(LDFLAGS)

$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR) $(TARGET)