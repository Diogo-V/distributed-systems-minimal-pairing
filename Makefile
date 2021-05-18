CC = g++
debug_flags = -O3 -Wall -std=c++11 -g -lm
flags = -O3 -Wall -std=c++11 -lm

# gen2procs input parameters
params = 100 30 0

# Compiles gen2procs, creates a new dag and puts it into a file
random: src/gen2procs.cpp
	$(CC) $(flags) -o cmake-build-debug/create-graph src/gen2procs.cpp
	./cmake-build-debug/create-graph $(params) > tests/problems.txt

all: src/main.cpp
	$(CC) $(flags) -o cmake-build-debug/main src/main.cpp

run: all
	time ./cmake-build-debug/main < tests/problem.txt

clean:
	rm -f cmake-build-debug/main cmake-build-debug/gen2procs
