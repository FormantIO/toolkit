# ROS Diagnostics

This example is divided in two parts: ROS diagnostics adapter and ROS diagnostics module

## Adapter

This adapter reads current online ROS topics and send them to Formant's server

[ROS Diagnostics Adapter](https://github.com/FormantIO/toolkit/tree/master/examples/ros-diagnostics/adapter)

## Run Locally

Go to the project directory

```bash
  cd /examples/ros_diagnostics/adapter
```

Install dependencies

```bash
  python3 -m pip install -r requirements.txt
```

Run the adapter

```bash
  ./start.sh
```

## ROS Diagnostics Module

This module, has a configuration which shows what ROS topics are expected. It reads the data that was sent from the
adapter, and then it compares, the data from the configuration with the data that is being received.

[ROS Diagnostics Module](https://github.com/FormantIO/toolkit/tree/master/examples/ros-diagnostics/module)

![App Screenshot](https://github.com/FormantIO/toolkit/blob/master/examples/ros-diagnostics/images/table.png)

The table displays the topic's health which can be represent in three states: in good standing (blue), bad(orange), or unknown(gray)

## Run Locally

Go to the project directory

```bash
  cd /examples/ros_diagnostics/module
```

Install dependencies

```bash
  yarn or npm i
```

Run the module

```bash
  .yarn dev or npm run start
```
