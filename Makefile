CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2
CXXFLAGS += -fopenmp
CPPFLAGS += -Icpp
LDLIBS += -lGL -lGLU -lglut

TARGET := cloth_sim
SOURCES := $(wildcard cpp/*.cpp)
OBJECTS := $(patsubst cpp/%.cpp,build/%.o,$(SOURCES))

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(LDFLAGS) $(OBJECTS) $(LDLIBS) -o $@

build:
	mkdir -p $@

build/%.o: cpp/%.cpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

run: $(TARGET)
	./$(TARGET)

clean:
	rm -rf build $(TARGET)
