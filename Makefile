CXX = g++
CC = g++
ODE_CFLAGS = $(shell pkg-config ode drawstuff --cflags)
ODE_LIBS = $(shell pkg-config ode drawstuff --libs)
#GLUT_CFLAGS = $(shell pkg-config glut --cflags)
#GLUT_LIBS = $(shell pkg-config glut --libs)
#GLUT_LIBS =  -framework OpenGL -framework GLUT -lm -lpthread

SOURCES = BackProp.cpp M3.cpp compassSensor.cpp environment.cpp envs.cpp joint.cpp lightSensor.cpp matrix.cpp neuralNetwork.cpp object.cpp optimizer.cpp propSensor.cpp robot.cpp tau.cpp tauOptimizer.cpp touchSensor.cpp userModel.cpp

# template.cpp

CPPFLAGS = $(ODE_CFLAGS) $(GLUT_CFLAGS)

LDFLAGS = -Wl,-bind_at_load $(ODE_LIBS) $(GLUT_LIBS)

OBJECTS = $(SOURCES:.cpp=.o)

all: M3

M3: $(OBJECTS)

clean:
	$(RM) $(OBJECTS) M3

run:
	./M3 &
	./M3 -null
