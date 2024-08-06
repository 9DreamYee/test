#compiler
CXX = g++
#compiler flags
CXXFLAGS = -std=c++17 -O2 -g
#target file
TARGET = OctilinearizeBoundary
#source files
SOURCES = OctilinearizeBoundary.cpp net.cpp CalculatePolygonArea.cpp
#object files
OBJECTS = $(SOURCES:.cpp=.o)
#dependency files
DEPS = net.h CalculatePolygonArea.h
#default target
all: $(TARGET)
#link the object files to the target file
$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJECTS)
OctilinearizeBoundary.o: OctilinearizeBoundary.cpp $(DEPS)
	$(CXX) $(CXXFLAGS) -c OctilinearizeBoundary.cpp
net.o: net.cpp net.h
	$(CXX) $(CXXFLAGS) -c net.cpp
CalculatePolygonArea.o: CalculatePolygonArea.cpp CalculatePolygonArea.h
	$(CXX) $(CXXFLAGS) -c CalculatePolygonArea.cpp
#clean the object files and the target file
clean:
	rm -f $(TARGET) $(OBJECTS)
.PHONY: all clean
