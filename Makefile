CC = g++
debug_flags = -O3 -Wall -std=c++11 -g -lm
flags = -O3 -Wall -std=c++11 -lm

all: src/main.cpp
	$(CC) $(flags) -o cmake-build-debug/main src/main.cpp

run: all
	time ./cmake-build-debug/main < tests/problems.txt

clean:
	rm -f cmake-build-debug/main
