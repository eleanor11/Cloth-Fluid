ROOT = ../../..

include $(ROOT)/makefile.common

LDFLAGS.release = -g -L/usr/lib -L$(CUDA_LIB) $(LIB)/flexRelease_x64.a -lGL -lglut -lGLU -lGLEW -lcudart
LDFLAGS.debug = -g -L/usr/lib -L$(CUDA_LIB) $(LIB)/flexDebug_x64.a -lGL -lglut -lGLU -lGLEW -lcudart
LDFLAGS = $(LDFLAGS.$(CFG))

TARGET.release = $(ROOT)/bin/linux64/flexDemoRelease 
TARGET.debug = $(ROOT)/bin/linux64/flexDemoDebug 
TARGET = $(TARGET.$(CFG))

SOURCES = $(wildcard *.cpp) $(CORE)/platform.cpp $(CORE)/perlin.cpp $(CORE)/sdf.cpp $(CORE)/shader.cpp $(CORE)/maths.cpp $(CORE)/mesh.cpp $(CORE)/aabbtree.cpp $(CORE)/tga.cpp $(CORE)/pfm.cpp $(CORE)/voxelize.cpp $(DEMO)/main.cpp $(DEMO)/shaders.cpp $(DEMO)/imgui.cpp $(DEMO)/imguiRenderGL.cpp
HEADERS = $(wildcard *.h) $(wildcard $(DEMO)/*.h)
OBJECTS = $(SOURCES:.cpp=.o)

all: $(TARGET)

release:
	$(MAKE) $(MAKEFILE) CFG=release

debug:
	$(MAKE) $(MAKEFILE) CFG=debug

$(TARGET): $(OBJECTS) makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $(TARGET) 

clean:
	-rm -f $(OBJECTS) $(TARGET)

%.o: %.cpp $(HEADERS)
	$(CC) $(CFLAGS) -c -o $@ $<

run: $(TARGET)
	./$(TARGET)

.PHONY : all clean

