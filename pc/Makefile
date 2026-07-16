CXX := g++
CXXFLAGS := -std=c++17 -Wall -Iinclude
LDFLAGS := 

SRCS := main.cpp $(wildcard source/*.cpp)
OBJS := $(SRCS:.cpp=.o)
TARGET := sensor_fusion_client

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $@ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)