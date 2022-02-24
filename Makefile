
magneto: main.cpp coordinate.cpp event.cpp obstacle.cpp segment.cpp simbox.cpp tracer.cpp event.cpp fluid.cpp measurement.cpp simulation.cpp
	g++  -O2 -std=c++11 -o  magneto.x main.cpp obstacle.cpp coordinate.cpp  segment.cpp simbox.cpp tracer.cpp event.cpp fluid.cpp measurement.cpp simulation.cpp
clean:
	rm magneto.x

