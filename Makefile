CC = g++
CFLAGS = -Wall -std=c++11
CFLAGS += -DVIDEO_PIPELINE
LDFLAGS =`pkg-config --cflags opencv`

BUILD_PATH = ./build
SOURCE_PATH = ./src
TEST_BUILD_PATH = ./test/build
TEST_SOURCE_PATH = ./test

SOURCES=main.cpp ros_adapter.cpp keyboard.cpp video_stream.cpp
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