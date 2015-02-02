CFLAGS=-O2 -march=armv5
CCFLAGS=-std=c++11 -Wall -D_GLIBCXX_USE_NANOSLEEP -Iinclude
DEPS= \
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

obj/%.o: src/%.cpp $(DEPS)
	mkdir -p $(@D)
	$(CC) -c -o $@ $< $(CFLAGS) $(CCFLAGS)

# libcl.a
lib/libcl.a: obj/cl/cl.o
	mkdir -p $(@D)
	ar rc $@ $^ && ranlib $@

# libev3cv.a
lib/libev3cv.a: obj/ev3cv.o obj/vision/calibration.o obj/vision/nxtcam.o
	mkdir -p $(@D)
	ar rc $@ $^ && ranlib $@

# calibrate
bin/calibrate: obj/calibrate.o lib/libev3cv.a lib/libcl.a
	mkdir -p $(@D)
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) -lstdc++ -lm -lpthread -Llib -lcl -lev3cv

# tests
bin/test/%: obj/test/%.o lib/libcl.a lib/libev3cv.a
	mkdir -p bin/test
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) -lstdc++ -lm -lpthread -Llib -lcl -lev3cv

.PHONY: all clean

clean:
	rm -rf obj/* bin/* lib/*

all: \
	bin/calibrate \
	lib/libcl.a \
	lib/libev3cv.a

test: \
	bin/test/autodiff \
	bin/test/calibration \
	bin/test/camera \
	bin/test/matrix 
