CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -pthread -O2 -g

INCLUDE_DIRS := $(shell find . -type d -not -path './build*' -not -path './.git*')
CXXFLAGS += $(addprefix -I,$(INCLUDE_DIRS))
CXXFLAGS += -I/usr/local/include/src
CXXFLAGS += -I/usr/include/eigen3
LDFLAGS  := -pthread -L/usr/local/lib -lydlidar_sdk -lm

TARGET := lidar_detection
BUILD_DIR := build

SRCS := $(shell find . -name "*.cpp" -not -path './build/*')
OBJS := $(patsubst ./%,$(BUILD_DIR)/%,$(SRCS:.cpp=.o))
DEPS := $(OBJS:.o=.d)

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $@ $(LDFLAGS)

$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

-include $(DEPS)

clean:
	rm -rf $(BUILD_DIR) $(TARGET)