#!/bin/bash
cd "$(dirname "$0")"
echo "Compiling geckogrip_climb_animator.cc..."
g++ -std=c++17 -shared -fPIC geckogrip_climb_animator.cc -o libgeckogrip_climb_animator.so $(pkg-config --cflags --libs gazebo)
echo "Compilation done. Exit code: $?"
