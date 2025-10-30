CXX = g++
CXXFLAGS = -std=c++20 -Wall -Wextra

TARGET_BIND = beavs_sim$(shell python3-config --extension-suffix)

LINKFLAGS = -shared
BUILDFLAGS = -fPIC -O3

LDFLAGS = -lboost_context -lboost_system $(shell python3-config --ldflags)
DEPFLAGS = -MMD -MP $(shell python3 -m pybind11 --includes)

SOURCES = $(wildcard *.cpp)
OBJECTS = $(SOURCES:.cpp=.o)

DEPS = $(OBJECTS:.o=.d)

.PHONY: build debug clean classify .FORCE

build: $(TARGET_BIND)

debug: BUILDFLAGS += -g
debug: build

$(TARGET_BIND): $(OBJECTS) board.o
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.cpp .buildflags.txt board.h
	$(CXX) $(CXXFLAGS) $(BUILDFLAGS) $(DEPFLAGS) -c $< -o $@

# TODO: Use things like $@ and $^
.buildflags.txt: .FORCE
	@printf "%b" "$(BUILDFLAGS)" > .buildflags.txt.tmp
	@if ! cmp -s .buildflags.txt.tmp .buildflags.txt 2>/dev/null; then \
		mv .buildflags.txt.tmp .buildflags.txt; \
	else \
		rm .buildflags.txt.tmp; \
	fi

board.h board.cpp: BEAVS5_Main.ino additions.txt
	python3 classify.py BEAVS5_Main.ino additions.txt board Board

-include $(DEPS)

clean:
	rm $(TARGET_BIND) $(OBJECTS) $(DEPS) .buildflags.txt board.cpp board.h
