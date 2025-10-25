CXX = g++
TARGET = main
CXXFLAGS = -std=c++20 -Wall -DSIMULATION
LDFLAGS = -lboost_context -lboost_system
DEPFLAGS = -MMD -MP

CPP_SOURCES = $(wildcard *.cpp)
INO_SOURCES = $(wildcard *.ino)
SOURCES = $(CPP_SOURCES) $(INO_SOURCES)

CPP_OBJECTS = $(CPP_SOURCES:.cpp=.o)
INO_OBJECTS = $(INO_SOURCES:.ino=.o)
OBJECTS = $(CPP_OBJECTS) $(INO_OBJECTS)

DEPS = $(OBJECTS:.o=.d)

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(DEPFLAGS) -c $< -o $@

%.o: %.ino
	$(CXX) $(CXXFLAGS) $(DEPFLAGS) -x c++ -c $< -o $@

-include $(DEPS)

clean:
	rm -f $(TARGET) $(OBJECTS) $(DEPS)
