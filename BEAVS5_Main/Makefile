CXX = g++
TARGET = sim.so
CXXFLAGS = -std=c++20 -Wall -DSIMULATION
LINKFLAGS = -shared
BUILDFLAGS = -fPIC -O3
LDFLAGS = -lboost_context -lboost_system
DEPFLAGS = -MMD -MP

SOURCES = $(wildcard *.cpp)
OBJECTS = $(SOURCES:.cpp=.o)

DEPS = $(OBJECTS:.o=.d)

.PHONY: build debug clean classify .FORCE

build: $(TARGET)

debug: BUILDFLAGS += -g
debug: build

$(TARGET): $(OBJECTS) board.o
	$(CXX) $(CXXFLAGS) -shared -o $@ $^ $(LDFLAGS)

%.o: %.cpp .buildflags.txt board.h
	$(CXX) $(CXXFLAGS) $(BUILDFLAGS) $(DEPFLAGS) -c $< -o $@

.buildflags.txt: .FORCE
	@printf "%b" "$(BUILDFLAGS)" > .buildflags.txt.tmp
	@if ! cmp -s .buildflags.txt.tmp .buildflags.txt 2>/dev/null; then \
		mv .buildflags.txt.tmp .buildflags.txt; \
	else \
		rm .buildflags.txt.tmp; \
	fi

board.h board.cpp: BEAVS5_Main.ino additions.txt
	@python3 classify.py BEAVS5_Main.ino additions.txt board Board

-include $(DEPS)

clean:
	@rm -f $(TARGET) $(OBJECTS) $(DEPS) .buildflags.txt board.cpp board.h
