# Standalone Makefile for rr_graph_transform
# Can be used independently of the VTR build system

CXX = g++
CXXFLAGS = -O3 -std=c++17 -Wall -Wextra

# Path to pugixml source
PUGIXML_DIR = ../../libs/EXTERNAL/libpugixml/src

INCLUDES = -I$(PUGIXML_DIR)

# Source files
SRCS = src/main.cpp $(PUGIXML_DIR)/pugixml.cpp

# Output executable
TARGET = rr_graph_transform

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $(SRCS)

clean:
	rm -f $(TARGET)

.PHONY: all clean

