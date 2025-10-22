CXX ?= g++
CXXFLAGS += -O3 -fomit-frame-pointer -fexpensive-optimizations -m64 -std=c++2b -Iinclude -IEigen -MMD -MP
CPPFLAGS += -Icpp

PLATFORM ?= $(if $(filter Windows_NT,$(OS)),windows,unix)

LIBDIR ?= cpp/lib
INCLUDEDIR ?= cpp/include

LDLIBS :=

TARGET_BASE := cloth_sim
OBJDIR := build/obj
BINDIR := build/bin

# util: converte paths para Windows (barra invertida)
wpath = $(subst /,\,$(1))

TARGET_NAME := $(TARGET_BASE)

ifeq ($(PLATFORM),windows)
CPPFLAGS += -I$(INCLUDEDIR)
LDFLAGS += -L$(LIBDIR)
LDLIBS  += -lgomp -pthread -pthread -lfreeglut -lglu32 -lopengl32 -O3 -m64 cpp/lib/libfreeglut.a
TARGET_NAME := $(TARGET_BASE).exe
MKDIR_CMD = if not exist "$(call wpath,$(1))" mkdir "$(call wpath,$(1))"
RM_BUILD  = rmdir /S /Q "$(call wpath,build)" 2>nul || exit /B 0
else
LDLIBS  += -lGL -lGLU -lglut
MKDIR_CMD = mkdir -p "$(1)"
RM_BUILD  = rm -rf build
endif

TARGET := $(BINDIR)/$(TARGET_NAME)

SOURCES := $(wildcard cpp/*.cpp)
OBJECTS := $(patsubst cpp/%.cpp,$(OBJDIR)/%.o,$(SOURCES))
DEPS := $(OBJECTS:.o=.d)

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(OBJECTS) | $(BINDIR)
	$(CXX) $(LDFLAGS) $(OBJECTS) $(LDLIBS) -o $@

# cria diretórios de build (compatível Win/Unix)
$(OBJDIR):
	@$(call MKDIR_CMD,$@)
$(BINDIR):
	@$(call MKDIR_CMD,$@)

# compila objetos em build/obj
$(OBJDIR)/%.o: cpp/%.cpp | $(OBJDIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

run: $(TARGET)
	"$(TARGET)"

clean:
	-@$(RM_BUILD)

# inclui dependências geradas por -MMD -MP
-include $(DEPS)
