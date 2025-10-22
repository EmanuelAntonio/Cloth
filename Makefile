CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2
CXXFLAGS += -fopenmp

CPPFLAGS += -Icpp

PLATFORM ?= $(if $(filter Windows_NT,$(OS)),windows,unix)

LIBDIR ?= cpp/lib
INCLUDEDIR ?= cpp/include

LDLIBS :=

TARGET_BASE := cloth_sim
TARGET := $(TARGET_BASE)

ifeq ($(PLATFORM),windows)
CPPFLAGS += -I$(INCLUDEDIR)
LDFLAGS += -L$(LIBDIR)
LDLIBS += -lfreeglut -lopengl32 -lglu32 -lwinmm -lgdi32
TARGET := $(TARGET_BASE).exe
else
LDLIBS += -lGL -lGLU -lglut
endif

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
	rm -rf build $(TARGET_BASE) $(TARGET_BASE).exe
