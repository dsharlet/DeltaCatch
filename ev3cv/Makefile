EV3DEVLANG?=../../../ev3dev-lang/
CFLAGS=-O2 -march=armv5
CCFLAGS=-std=c++11 -Wall -D_GLIBCXX_USE_NANOSLEEP -Iinclude -I$(EV3DEVLANG)/cpp
DEPS= \
	include/cl/cl.h \
	include/ev3/nxtcam.h \
	include/ev3/servo.h \
	include/math/autodiff.h \
	include/math/matrix.h \
	include/math/pid_controller.h \
	include/math/quaternion.h \
	include/math/vector2.h \
	include/math/vector3.h \
	include/vision/calibration.h \
	include/vision/camera.h \
	include/ev3cv.h

LIBEV3DEV=-L$(EV3DEVLANG)/cpp/lib -lev3dev

obj/%.o: src/%.cpp $(DEPS)
	mkdir -p $(@D)
	$(CC) -c -o $@ $< $(CFLAGS) $(CCFLAGS)

# libcl.a
lib/libcl.a: obj/cl/cl.o
	mkdir -p $(@D)
	ar rc $@ $^ && ranlib $@

# libev3cv.a
lib/libev3cv.a: obj/ev3cv.o obj/vision/calibration.o obj/ev3/nxtcam.o obj/ev3/servo.o
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

bin/test/servo: obj/test/servo.o lib/libcl.a lib/libev3cv.a
	mkdir -p bin/test
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) -lstdc++ -lm -lpthread -Llib -lcl -lev3cv $(LIBEV3DEV)

.PHONY: all clean

clean:
	rm -rf obj/* bin/* lib/*
	
test: \
	bin/test/autodiff \
	bin/test/calibration \
	bin/test/camera \
	bin/test/matrix \
	bin/test/servo
	
all: \
	bin/calibrate \
	lib/libcl.a \
	lib/libev3cv.a \
	test
