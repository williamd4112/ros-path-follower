CC=g++
CFLAGS = -c -Wall
CFLAGS += -DDEBUG 
LDFLAGS=`pkg-config --cflags opencv`

BUILD_PATH=./build
SOURCE_PATH=./src

SOURCES=main.cpp ros_adapter.cpp keyboard.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=path-follower

INC_PATH = -I/opt/ros/indigo/include -I./inc
LIB_PATH = -L/opt/ros/indigo/lib 
LIBS = -lpthread -lroscpp -lrosconsole -lrostime -lroscpp_serialization `pkg-config --libs opencv`

all: $(EXECUTABLE)
    
$(EXECUTABLE): $(OBJECTS)
	cd $(BUILD_PATH); \
	$(CC) $(LDFLAGS) $(OBJECTS) $(INC_PATH) $(LIB_PATH) -o $@ $(LIBS)

%.o: $(SOURCE_PATH)/%.cpp
	$(CC) $(CFLAGS) $(INC_PATH) $< -o $(BUILD_PATH)/$@

clean:
	rm build/*.o
	rm build/path-follower
