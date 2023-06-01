CX = g++    # c++ 코드이면 g++
CCFLAGS = -g -Wall  # debug mode, 모든경고 출력
#SRCS = main.cpp

DXLFLAGS = -I/usr/local/include/dynamixel_sdk_cpp
DXLFLAGS += -ldxl_x64_cpp   #라이브러리 파일명에서 접두어 lib와 확장자를 빼고 적어줌
                                              #32bit이면 x64->x32, cpp버전이면 c->cpp로 수정할것
DXLFLAGS += -lrt

OPENCV = `pkg-config opencv4 --cflags --libs`
LIBS = $(OPENCV)

TARGET = following_robot_yolo
OBJS = main.o dxl.o
$(TARGET):$(OBJS)
	$(CX) $(CCFLAGS) -o $(TARGET) $(OBJS) $(DXLFLAGS) $(LIBS)
main.o:main.cpp
	$(CX) $(CCFLAGS) -c main.cpp $(DXLFLAGS) $(LIBS)
dxl.o:dxl.hpp dxl.cpp
	$(CX) $(CCFLAGS) -c dxl.cpp $(DXLFLAGS)
#$(TARGET):$(SRCS)
#$(CX) $(CCFLAGS) -o $(TARGET) $(SRCS) $(LIBS)
.PHONY: all clean

all: $(TARGET)

clean:
	rm -rf $(TARGET) $(OBJS)