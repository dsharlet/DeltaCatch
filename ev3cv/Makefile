CFLAGS=-O2 -march=armv5
CCFLAGS=-std=c++11 -Wall -D_GLIBCXX_USE_NANOSLEEP -Iinclude
DEPS= \
	include/cl/arg_port.h \
	include/cl/cl.h \
	include/math/autodiff.h \
	include/math/matrix.h \
	include/math/pid_controller.h \
	include/math/quaternion.h \
	include/math/vector2.h \
	include/math/vector3.h \
	include/vision/calibration.h \
	include/vision/camera.h \
	include/vision/nxtcam.h \
	include/ev3cv.h \

# libcl.a
obj/cl/cl.o: src/cl/cl.cpp include/cl/cl.h
	mkdir -p obj/cl
	$(CC) -c -o $@ $< $(CFLAGS) $(CCFLAGS)

lib/libcl.a: obj/cl/cl.o
	mkdir -p lib
	ar rc $@ $^ && ranlib $@

# libev3cv.a
obj/%.o: src/%.cpp $(DEPS)
	mkdir -p $(@D)
	$(CC) -c -o $@ $< $(CFLAGS) $(CCFLAGS)

lib/libev3cv.a: obj/vision/calibration.o obj/vision/nxtcam.o
	mkdir -p lib
	ar rc $@ $^ && ranlib $@

# tests
obj/test/%.o: test/%.cpp $(DEPS)
	mkdir -p obj/test
	$(CC) -c -o $@ $< $(CFLAGS) $(CCFLAGS)

bin/test/%: obj/test/%.o
	mkdir -p bin/test
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) -lstdc++ -lm -lpthread


.PHONY: all clean

clean:
	rm -rf obj/* bin/* lib/*

all: lib/libcl.a lib/libev3cv.a

test:  \
	bin/test/autodiff \
	bin/test/calibration \
	bin/test/camera \
	bin/test/matrix 
