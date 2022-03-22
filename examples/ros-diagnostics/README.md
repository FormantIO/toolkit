# ROS Diagnostics

This example is divided in two parts: ROS Diagnostics Adapter and ROS Diagnostics Module

## Adapter

This adapter reads current online ROS topics and sends them to Formant's server

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

This module has a configuration that shows which ROS topics are expected. It reads the data that was sent from the
adapter and then it compares the data from the configuration with the data that is being received.

[ROS Diagnostics Module](https://github.com/FormantIO/toolkit/tree/master/examples/ros-diagnostics/module)

![App Screenshot](https://github.com/FormantIO/toolkit/blob/master/examples/ros-diagnostics/images/table.png)

The table displays the topics health which can be represented in three states:  good standing (blue), bad(orange), or unknown(gray)

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
`
