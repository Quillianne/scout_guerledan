# Makefile for blueboat_rtt C++
CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -pthread

all: blueboat_rtt

blueboat_rtt: blueboat_rtt.cpp
	$(CXX) $(CXXFLAGS) $< -o $@

clean:
	rm -f blueboat_rtt
