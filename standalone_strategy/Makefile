CXX ?= g++
AR ?= ar

CXXFLAGS ?= -std=c++20 -Wall -Wextra -O2
CXXFLAGS += -Wno-invalid-constexpr -Wno-builtin-declaration-mismatch

INCLUDES = -Iinclude \
           -Iinclude/shims \
           -Iinclude/Streaming \
           -Iinclude/Math \
           -Iinclude/MathBase \
           -Iinclude/Strategy \
           -Iinclude/Strategy/ActiveRoles \
           -Iinclude/Strategy/PositionRoles \
           -Iinclude/Tools/BehaviorControl \
           -Iinclude/Tools/Modeling \
           -Iinclude/Tools/Motion \
           -Iinclude/Representations \
           -Iinclude/Representations/BehaviorControl \
           -Iinclude/Representations/Configuration \
           -Iinclude/Representations/Infrastructure \
           -Iinclude/Representations/Communication \
           -Iinclude/Representations/Modeling \
           -Iinclude/Representations/MotionControl \
           -Iinclude/Representations/Sensing \
           -Iinclude/Platform \
           -Iinclude/Debugging

SRCDIR = src
BUILDDIR = build
TARGET_LIB = libbhuman_strategy.a
TARGET_TEST = strategy_test

SRCS = $(wildcard $(SRCDIR)/*.cpp)
OBJS = $(patsubst $(SRCDIR)/%.cpp,$(BUILDDIR)/%.o,$(SRCS))

all: $(TARGET_LIB) $(TARGET_TEST)

lib: $(TARGET_LIB)

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

$(BUILDDIR)/%.o: $(SRCDIR)/%.cpp | $(BUILDDIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(TARGET_LIB): $(OBJS)
	$(AR) rcs $@ $^

$(TARGET_TEST): main_test.cpp $(TARGET_LIB)
	$(CXX) $(CXXFLAGS) $(INCLUDES) main_test.cpp $(TARGET_LIB) -o $@

clean:
	rm -rf $(BUILDDIR) $(TARGET_LIB) $(TARGET_TEST)

.PHONY: all lib clean
