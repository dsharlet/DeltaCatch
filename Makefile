
CFLAGS=-O2 -march=armv5
CCFLAGS=-std=c++11 -D_GLIBCXX_USE_NANOSLEEP -I../../ev3dev-lang/cpp -I../../Util
OBJ=
LIBS=-lstdc++ -L../../Util/lib -lutil -L../../ev3dev-lang/cpp -lev3dev -lpthread -lm
DEPS=delta_robot.h delta_hand.h delta_config.h vector2.h vector3.h arg_port.h nxtcam.h circular_array.h autodiff.h matrix.h debug.h trajectory.h camera.h viz_client.h

obj/%.o: %.cpp $(DEPS)
	mkdir -p obj
	$(CC) -c -o $@ $< $(CFLAGS) $(CCFLAGS)
		
bin/delta_test: obj/delta_test.o obj/delta_robot.o
	mkdir -p bin
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)
		
bin/delta_catch: obj/delta_catch.o obj/delta_robot.o obj/nxtcam.o obj/debug.o obj/delta_hand.o obj/trajectory.o obj/viz_client.o
	mkdir -p bin
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)
	
bin/nxtcam_test: obj/nxtcam_test.o obj/nxtcam.o
	mkdir -p bin
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)

bin/autodiff_test: obj/autodiff_test.o
	mkdir -p bin
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)

.PHONY: all clean

clean:
	rm -rf obj bin *~

all:  bin/delta_test bin/nxtcam_test bin/delta_catch bin/autodiff_test
