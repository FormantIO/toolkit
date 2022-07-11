# CPP ROS Bag Recorder

The CPP ROS is a tool that allows you to easily record ROS bags. 
The tools can run in the background and record only when needed, 
or record at all times. 

# Quick start

## protobuf Install 

To install the protobuf compiler, all that is required is 
```bash
sudo apt update
sudo apt install protobuf-compiler
```

## gRPC Install
To begin, you will need to first install the C++ gRPC library. 

Please refer to https://grpc.io/blog/installation/ to find instructions
for building and installing gRPC for C++ 

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