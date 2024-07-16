# Makefile to compile both server and client programs

CXX = g++
CXXFLAGS = -std=c++11 -I /usr/include/pcl-1.13
LDFLAGS = -lpthread -lboost_system -lboost_thread -lpcl_common -lpcl_io

# Targets
all: server client

# Server target
server: server.o
	$(CXX) -o server server.o $(LDFLAGS)

server.o: server.cpp
	$(CXX) $(CXXFLAGS) -c server.cpp

# Client target
client: client.o
	$(CXX) -o client client.o $(LDFLAGS)

client.o: client.cpp
	$(CXX) $(CXXFLAGS) -c client.cpp

# Clean target
clean:
	rm -f *.o server client

.PHONY: all clean

