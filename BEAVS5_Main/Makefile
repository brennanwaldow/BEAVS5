CXX = g++
TARGET = main
CXXFLAGS = -std=c++20 -Wall -DSIMULATION
BUILDFLAGS = -O3
LDFLAGS = -lboost_context -lboost_system
DEPFLAGS = -MMD -MP

CPP_SOURCES = $(wildcard *.cpp)
INO_SOURCES = $(wildcard *.ino)
SOURCES = $(CPP_SOURCES) $(INO_SOURCES)

CPP_OBJECTS = $(CPP_SOURCES:.cpp=.o)
INO_OBJECTS = $(INO_SOURCES:.ino=.o)
OBJECTS = $(CPP_OBJECTS) $(INO_OBJECTS)

DEPS = $(OBJECTS:.o=.d)

.PHONY: build debug clean .FORCE

build: $(TARGET)

debug: BUILDFLAGS = -g
debug: build

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.cpp .buildflags.txt
	$(CXX) $(CXXFLAGS) $(BUILDFLAGS) $(DEPFLAGS) -c $< -o $@

%.o: %.ino .buildflags.txt
	$(CXX) $(CXXFLAGS) $(BUILDFLAGS) $(DEPFLAGS) -x c++ -c $< -o $@

.buildflags.txt: .FORCE
	@printf "%b" "$(BUILDFLAGS)" > .buildflags.txt.tmp
	@if ! cmp -s .buildflags.txt.tmp .buildflags.txt 2>/dev/null; then \
		mv .buildflags.txt.tmp .buildflags.txt; \
	else \
		rm .buildflags.txt.tmp; \
	fi


-include $(DEPS)

clean:
	rm -f $(TARGET) $(OBJECTS) $(DEPS) .buildflags.txt
