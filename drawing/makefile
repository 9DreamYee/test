SRC_DIR := src
BIN_DIR := bin
INC_DIR := inc
LIB_DIR := lib
CXX := g++
GTKFLAGS := $$(pkg-config gtkmm-3.0 --cflags --libs)
SOURCES  := $(wildcard $(SRC_DIR)/*.cc $(INC_DIR)/*.hpp)
OBJECTS  := $(patsubst $(SRC_DIR)/%.cc,$(SRC_DIR)/%.o,$(SOURCES))
DEPENDS  := $(patsubst $(SRC_DIR)/%.cc,$(SRC_DIR)/%.d,$(SOURCES))
EXECUTABLE := TIBA
INCLUDES := $(INC_DIR)
#INCLUDES := $($(INC_DIR)/)
LIBRARY  := $(LIB_DIR)
#CXXFLAGS := -I$(INC_DIR) -std=c++17 
CXXFLAGS :=-g -I$(INC_DIR) -std=c++17 -DLEDA_MULTI_THREAD
LDFLAGS := -L$(LIBRARY) -lX11 -lXft -ltbb -lm -fopenmp -pthread
DEBUG_CXXFLAGS := -Wall -g -I$(INC_DIR) -std=c++17 -DLEDA_MULTI_THREAD

.PHONY: clean all


all: $(SOURCES) $(BIN_DIR)/$(EXECUTABLE)

	echo $(SOURCES)

debug: CXXFLAGS=$(DEBUG_CXXFLAGS)
debug: clean all 

$(BIN_DIR)/$(EXECUTABLE): $(OBJECTS)
	$(CXX) $(CXXFLAGS) $(OBJECTS) -o $@ $(LDFLAGS) $(GTKFLAGS)


# Linking the executable from the object files

$(SRC_DIR)/%.o: $(SRC_DIR)/%.cc
		$(CXX) -MMD -c $(CXXFLAGS) $< -o $@ $(GTKFLAGS)

-include $(DEPENDS)


clean:
	rm -rf $(SRC_DIR)/*.o $(SRC_DIR)/*.d $(BIN_DIR)/$(EXECUTABLE)

