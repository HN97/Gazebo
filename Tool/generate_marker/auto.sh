#!/usr/bin/bash

chmod +x main.cpp
echo File main.cpp is building......
g++ main.cpp -o out `pkg-config --cflags --libs opencv4`
echo done!

