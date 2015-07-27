ROOT = ../../..

include $(ROOT)/makefile.common

LDFLAGS = -g -L/usr/lib -L$(CUDA_LIB) -lcudart

TARGET.release = $(ROOT)/lib/linux64/flexReleaseExt_x64.a 
TARGET.debug = $(ROOT)/lib/linux64/flexDebugExt_x64.a 
TARGET = $(TARGET.$(CFG))

SOURCES = $(wildcard *.cpp) $(CORE)/platform.cpp $(CORE)/perlin.cpp $(CORE)/sdf.cpp $(CORE)/shader.cpp $(CORE)/maths.cpp $(CORE)/mesh.cpp $(CORE)/aabbtree.cpp $(CORE)/tga.cpp $(CORE)/pfm.cpp $(CORE)/voxelize.cpp 
HEADERS = $(wildcard *.h)
OBJECTS = $(SOURCES:.cpp=.o) 

CUSOURCES = $(wildcard *.cu) $(EXT)/flexExt.cu
CUOBJECTS = $(CUSOURCES:.cu=.co) 

all: $(TARGET)

release:
	$(MAKE) $(MAKEFILE) CFG=release

debug:
	$(MAKE) $(MAKEFILE) CFG=debug

$(TARGET): $(OBJECTS) $(CUOBJECTS) makefile
	$(AR) rcs $(TARGET) $(OBJECTS) $(CUOBJECTS) 

clean:
	-rm -f $(OBJECTS) $(CUOBJECTS) 

%.o: %.cpp $(HEADERS)
	$(CC) $(CFLAGS) -c -o $@ $<

%.co: %.cu
	$(NVCC) -I$(CUDA_INC) $(CUFLAGS) -I$(ROOT) -I$(EXTERNAL) -I$(CUB) -c $< -o $@ 

.PHONY : all clean

