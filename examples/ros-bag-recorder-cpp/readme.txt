

To build proto files:

    cd src/protos
    sudo protoc -I . --grpc_out=. --plugin=protoc-gen-grpc=`which grpc_cpp_plugin` protos/*/v1/*.proto
    sudo protoc -I . --cpp_out=. protos/*/v1/*.proto


sudo apt install ros-noetic-ros-type-introspection