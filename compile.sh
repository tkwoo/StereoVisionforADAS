#!/bin/bash

g++ -o stereovisionforadas ./*.cpp `pkg-config --cflags --libs opencv` --std=c++11