# CPP ROS Bag Recorder

The CPP ROS is a tool that allows you to easily record ROS bags. 
The tools can run in the background and record only when needed, 
or record at all times. 

# Quick start

## nlohmann/JSON

The nlohmann JSON library will need to be installed on the machine. 

Ubuntu 20.04
```bash
sudo apt install nlohmann-json3-dev
```

Ubuntu 18.04
```bash
sudo apt install nlohmann-json-dev
```

## protobuf Install 

Protobuf is required to build this adapter. You will need to build 
protobuf from source to use it with Cmake. To do so, follow the install
instructions found at https://github.com/protocolbuffers/protobuf/blob/main/src/README.md

## gRPC Install

gRPC is required, and also needs to be built from source to be 
used with cmake. 

Please refer to the `build from source` section at https://github.com/grpc/grpc/blob/master/BUILDING.md

Note: after running 

```
$ mkdir -p cmake/build
$ cd cmake/build
$ cmake ../..
$ make
```

as specified in the build instructions, you will need to also run
```
$ sudo make install 
```

This will ensure the headers are installed and the cmake config files are placed in their proper directories for building. 

## Additional ROS libraries

Once gRPC is installed, you'll need to install some additional ROS
libraries. Namely, you'll need **Topic Tools** and **ROS Type introspection**

Those can be installed as follows: 

```bash
sudo apt install ros-$ROS_DISTRO-topic-tools
sudo apt install ros-$ROS_DISTRO-ros-type-introspection 
```

## Run

Now that everything is installed, you can run the adapter using 
```bash
./start.sh
```

On first run, start.sh will build the node then run. All subsequent 
runs will use the already build binary. 

# Configuration

you'll notice that calling start.sh doesn't record any bags, but only 
starts the adapter. We must start recording through Formant. 

We create a command which will take the name 'start_ros_recorder'. 
This command takes no parameters. If we call this command, the 
adapter will immediately start recording bags to the configured output
location.

We can stop the recorder with the command 'stop_ros_recorder'. 

We may also ask the recorder to record for a specified number of seconds. 
To do so, we execute the command 'start_ros_recorder_duration' which takes
the number of seconds to record as a parameter. 

That's it! You should be up and running now. 