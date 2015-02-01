
CFLAGS=-O2 -march=armv5
CCFLAGS=-std=c++11 -Wall -D_GLIBCXX_USE_NANOSLEEP -I../../ev3dev-lang/cpp -I../../Util -Iev3cv
OBJ=
LIBS=-lstdc++ -L../../Util/lib -lutil -L../../ev3dev-lang/cpp -lev3dev -lpthread -lm
DEPS=\
	arg_port.h \
	calibration_data.h \
	camera.h \
	circular_array.h \
	debug.h \
	delta_hand.h \
	delta_robot.h \
	delta_robot_args.h \
	nxtcam.h \
	servo.h \
	stereo_config.h \
	test.h \
	trajectory.h \
	viz_client.h \

obj/%.o: %.cpp $(DEPS)
	mkdir -p obj
	$(CC) -c -o $@ $< $(CFLAGS) $(CCFLAGS)
		
bin/delta_test: obj/delta_test.o obj/delta_robot.o obj/servo.o obj/debug.o
	mkdir -p bin
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)
		
bin/delta_catch: obj/delta_catch.o obj/delta_robot.o obj/nxtcam.o obj/debug.o obj/delta_hand.o obj/trajectory.o obj/viz_client.o obj/servo.o
	mkdir -p bin
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)
		
bin/calibrate: obj/calibrate.o obj/nxtcam.o obj/debug.o
	mkdir -p bin
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)

bin/stereo_test: obj/stereo_test.o obj/nxtcam.o obj/delta_robot.o obj/servo.o obj/debug.o
	mkdir -p bin
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)
	
bin/autodiff_test: obj/autodiff_test.o
	mkdir -p bin
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)
	
bin/camera_test: obj/camera_test.o
	mkdir -p bin
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)
	
bin/matrix_test: obj/matrix_test.o
	mkdir -p bin
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)
	
bin/trajectory_test: obj/trajectory_test.o obj/trajectory.o obj/nxtcam.o obj/debug.o
	mkdir -p bin
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)

bin/servo_test: obj/servo_test.o obj/servo.o
	mkdir -p bin
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)

.PHONY: all clean

clean:
	rm -rf obj bin *~

all:  bin/delta_test bin/delta_catch bin/autodiff_test bin/calibrate bin/stereo_test bin/camera_test bin/matrix_test bin/trajectory_test bin/servo_test
