CC = g++
CFLAGS = -Wall -O3 -std=c++11

# CPU of GPU
#CFLAGS += -DGPU

# Features
CFLAGS += -DVIDEO_PIPELINE
CFLAGS += -DMOTION_DETECT
CFLAGS += -DLANE_DETECT
CFLAGS += -DLANE_DETECT_QUANTNIZE
CFLAGS += -DMOMENT_DETECT

# Debug option
CFLAGS += -DDEBUG
#CFLAGS += -DDEBUG_MAIN
#CFLAGS += -DDEBUG_MOTION_DETECT_BOUNDING_BOX 
#CFLAGS += -DDEBUG_MOTION_DETECT
CFLAGS += -DDEBUG_LANE_DETECT
CFLAGS += -DDEBUG_MOMENT_DETECT
CFLAGS += -DDEBUG_FPS

LDFLAGS =`pkg-config --cflags opencv`

BUILD_PATH = ./build
SOURCE_PATH = ./src
TEST_BUILD_PATH = ./test/build
TEST_SOURCE_PATH = ./test

SOURCES = main.cpp ros_adapter.cpp keyboard.cpp 
SOURCES += video_stream.cpp 
SOURCES += motion.cpp 
SOURCES += lane.cpp
SOURCES += moment.cpp

OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=path-follower

INC_PATH = -I/opt/ros/indigo/include -I./inc
LIB_PATH = -L/opt/ros/indigo/lib 
LIBS = -lpthread -lroscpp -lrosconsole -lrostime -lroscpp_serialization `pkg-config --libs opencv`

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
