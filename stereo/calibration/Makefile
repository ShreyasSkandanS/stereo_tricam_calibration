CXX=g++
CXXFLAGS= -std=c++11

CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`

all:
	$(CXX) -o calibrate src/calibrate.cpp $(CFLAGS) $(LIBS) -lpopt

clean:
	rm -rf calibrate rectify
	rm -rf build/
	rm -rf *~
	rm -rf *.o
