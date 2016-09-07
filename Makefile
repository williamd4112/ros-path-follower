# Module config
MODULE_ROS_ADAPTER = n
MODULE_VIDEO_PIPELINE = y
MODULE_MOTION_DETECT = n
MODULE_LANE_DETECT = n
MODULE_LANE_DETECT_QUANTNIZE = n
MODULE_MOMENT_DETECT = y
MODULE_OBJECT_DETECT = y

# Compiler option
CC = g++
CFLAGS = -Wall -O3 -std=c++11

# CPU of GPU
CFLAGS += -DGPU
CFLAGS += -DGPU_MOTION
#CFLAGS += -DASYNC

SOURCES = main.cpp keyboard.cpp module_def.cpp
BUILD_PATH = ./build
SOURCE_PATH = ./src
TEST_BUILD_PATH = ./test/build
TEST_SOURCE_PATH = ./test

OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=path-follower

LDFLAGS =`pkg-config --cflags opencv`
INC_PATH = -I./inc
LIB_PATH = 
LIBS = -lpthread `pkg-config --libs opencv`
 
# Features
ifeq ($(MODULE_ROS_ADAPTER), y)
CFLAGS += -DROS_ADAPTER
CFLAFS += -DTEST_STRAIGHT
SOURCES += ros_adapter.cpp 
INC_PATH += -I/opt/ros/indigo/include
LIB_PATH += -L/opt/ros/indigo/lib
LIBS += -lroscpp -lrosconsole -lrostime -lroscpp_serialization
endif

ifeq ($(MODULE_VIDEO_PIPELINE), y)
CFLAGS += -DVIDEO_PIPELINE
SOURCES += video_stream.cpp 
endif

ifeq ($(MODULE_MOTION_DETECT), y)
CFLAGS += -DMOTION_DETECT
SOURCES += motion.cpp 
endif

ifeq ($(MODULE_LANE_DETECT), y)
CFLAGS += -DLANE_DETECT
SOURCES += lane.cpp
endif

ifeq ($(MODULE_LANE_DETECT_QUANTNIZE), y)
CFLAGS += -DLANE_DETECT_QUANTNIZE
endif

ifeq ($(MODULE_MOMENT_DETECT), y)
CFLAGS += -DMOMENT_DETECT
SOURCES += moment.cpp
endif

ifeq ($(MODULE_OBJECT_DETECT), y)
CFLAGS += -DOBJECT_DETECT
SOURCES += object.cpp 
endif

# Debug option
CFLAGS += -DDEBUG
#CFLAGS += -DDEBUG_MAIN
#CFLAGS += -DDEBUG_MOTION_DETECT_BOUNDING_BOX 
#CFLAGS += -DDEBUG_MOTION_DETECT
#CFLAGS += -DDEBUG_LANE_DETECT
CFLAGS += -DDEBUG_MOMENT_DETECT
CFLAGS += -DDEBUG_OBJECT_HAAR_DETECT
CFLAGS += -DDEBUG_OBJECT_DETECT_REDCIRCLE
CFLAGS += -DDEBUG_FSM
CFLAGS += -DDEBUG_FPS

all: $(EXECUTABLE)
    
$(EXECUTABLE): $(OBJECTS)
	cd $(BUILD_PATH); \
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJECTS) $(INC_PATH) $(LIB_PATH) -o $@ $(LIBS)

%.o: $(SOURCE_PATH)/%.cpp
	$(CC) $(CFLAGS) -c $(INC_PATH) $< -o $(BUILD_PATH)/$@

test_videothread: 
	$(CC) $(CFLAGS) $(LDFLAGS) $(TEST_SOURCE_PATH)/$@.cpp $(INC_PATH) $(LIB_PATH) -o $(TEST_BUILD_PATH)/$@ $(LIBS) -std=c++11

clean:
	rm build/*.o
	rm build/path-follower
