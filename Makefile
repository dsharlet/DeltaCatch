
CFLAGS=-O2 -march=armv5
CCFLAGS=-std=c++11 -Wall -D_GLIBCXX_USE_NANOSLEEP -I../ev3dev-lang/cpp -Iev3cv/include
OBJ=
LIBS=-lstdc++ -Lev3cv/lib -lcl -lev3cv -L../ev3dev-lang/cpp/lib -lev3dev -lpthread -lm
DEPS=\
	debug.h \
	delta_hand.h \
	delta_robot.h \
	delta_robot_args.h \
	stereo_config.h \
	test.h \
	trajectory.h \

obj/%.o: %.cpp $(DEPS)
	mkdir -p obj
	$(CC) -c -o $@ $< $(CFLAGS) $(CCFLAGS)
		
bin/delta_test: obj/delta_test.o obj/delta_robot.o obj/debug.o
	mkdir -p bin
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)
		
bin/delta_catch: obj/delta_catch.o obj/delta_robot.o obj/debug.o obj/delta_hand.o obj/trajectory.o
	mkdir -p bin
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)
		
bin/stereo_test: obj/stereo_test.o obj/delta_robot.o obj/debug.o
	mkdir -p bin
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)
		
bin/trajectory_test: obj/trajectory_test.o obj/trajectory.o obj/debug.o
	mkdir -p bin
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)
	
.PHONY: all clean

clean:
	rm -rf obj/* bin/*

all:  bin/delta_test bin/delta_catch bin/stereo_test bin/trajectory_test