CXX=g++
CXXFLAGS= -std=c++11

CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`

all:
	$(CXX) $(CXXFLAGS) -O2 -Wall -o convert convert_calib.cpp $(CFLAGS) $(LIBS)
clean:
	rm convert
	rm -rf build/
	rm -rf *~
	rm -rf *.o
