#!/bin/bash

# Check for protos. If not found, download and compile.
if [ ! -d "protos/protos" ]; then

    echo "Protos not found. Downloading.."

    # Download the protos
    git init protos
    cd protos
    git remote add -f origin https://github.com/FormantIO/formant.git
    git config core.sparseCheckout true
    echo "protos/" >>.git/info/sparse-checkout

    git pull origin master

    # Compile the protos with protoc
    protoc -I . --grpc_out=. --plugin=protoc-gen-grpc=$(which grpc_cpp_plugin) protos/*/v1/*.proto
    protoc -I . --cpp_out=. protos/*/v1/*.proto

    cd ..

fi

if [ ! -f "devel/lib/ros_bag_adapter/main" ]; then
    echo "Executable needs to be built.."
    catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-O3
fi

echo "Attempting to start the bag recorder..."

source devel/setup.bash
rosrun ros_bag_adapter main

