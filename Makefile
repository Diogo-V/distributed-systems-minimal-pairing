CC = g++
debug_flags = -O3 -Wall -std=c++11 -g -lm
flags = -O3 -Wall -std=c++11 -lm

# randomDAG input parameters
params = 3000 0.3

all: random src/final.cpp
	$(CC) $(flags) -o cmake-build-debug/final src/final.cpp

run: all
	time ./cmake-build-debug/final < tests/problems.txt

debug: src/main.cpp
	$(CC) $(debug_flags) -o cmake-build-debug/debug src/main.cpp

clean:
	rm -f cmake-build-debug/final cmake-build-debug/randomDAG
