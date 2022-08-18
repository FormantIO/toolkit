
# ROS Service Call Adapter 

## About

The ROS service call adapter allows a user to map Formant commands and Formant button presses to different service calls. For example, a user could have a button named <b>Check Robot</b>. In the service call adapter, you could map this button to a service called <b>/robot_status_check</b>. Thus, then the button Check Robot is pressed, it will internally call /robot_status_check. The result of the service call is then posted to a Formant Stream. 

## Getting Started

To get started with this adapter, first ensure that rosgraph is installed.

Ubuntu 20.04
```bash
apt-get install python3-rosgraph
```

Ubuntu 18.04
```bash
apt-get install python-rosgraph
```

### Command Mapping

By default, the adapter will listen for commands under the name 
`rosservice`. This is the stream which the front-end module
sends commands to, and thus should be left alone unless the user
needs to change it for an advanced use case. 


### Button Mapping

As mentioned earlier, the ros service call adapter is able to call
services based on a button press from formant. The button can be 
set-up as either a ROS button (in which case other sources can also
request the adapter call a services by sending a boolean message) and 
also as an API button. 

To map a button press to a specified ROS service, go into the config.json file.
Under either `ros-button-mapping` or `api-button-mapping`, we need to add a dictionary 
entry. The key of this entry will be the name of either the ros topic or the api button. 
The value of this entry will be a dictionary which holds the service call parameters, and this
dictionary can be automatically generated via the `create_call.py` script. The usage of the 
script is show in the section below.  

This is a sample of a ROS button mapped to a call to the `/random` service looks like in
the config.json:


```json
{
    "service-commands": [
        "rosservice"
    ],
    "api-button-mapping": {},
    "ros-button-mapping": {
        "/ros_button": {
            "/random": {
                "string": "sample-string",
                "int64_arr": [
                    1.0,
                    2.0
                ],
                "float32": 3.0,
                "float64_arr": [
                    4.0
                ],
                "twist": {
                    "twist.linear": {
                        "twist.linear.x": 5.0,
                        "twist.linear.y": 6.0,
                        "twist.linear.z": 7.0
                    },
                    "twist.angular": {
                        "twist.angular.x": 8.0,
                        "twist.angular.y": 9.0,
                        "twist.angular.z": 10.0
                    }
                }
            }
        }
    }
}
```

That's it. Now a button press to the `i_am_a_button` button will 
map to the service call!

## Generate the service call using `create_call.py`

To generate the service call using `create_call.py` (which is highly recommended),
first ensure that `roscore` and the service are both running. Further, ensure that 
the proper catkin_ws has been sourced. Once that is done, run `python3 create_call.py`. 

This will first output all the current running services. You will be prompted to 
type the service you would like to generate the call json for. In our example case, 
we input `/random`. Once we input the service name, the script will ask for an 
input for each parameter in the service. Simply type the wanted value for each parameter. 
Once complete, the script will print out the service call json. Copy this value and paste 
it into the `config.json`, specifically, it is should be the value of the key we added 
for the button which we would like to call the specified service. 

Here is an example of using the `create_call.py` script:

```
$ python3 create_call.py

INFO: Agent communication established.
Current services: 

        /rosout/get_loggers
        /rosout/set_logger_level
        /formant_tf2_lookup_transform_node/get_loggers
        /formant_tf2_lookup_transform_node/set_logger_level
        /formant_tf2_lookup_transform_node/tf2_frames
        /formant_bridge_node/get_loggers
        /formant_bridge_node/set_logger_level
        /random_server/get_loggers
        /random_server/set_logger_level
        /random


Please enter service: /random
Please enter a value for parameter 'string': sample-string
Please enter a comma separated array of numbers for parameter 'int64_arr':
        1,2
Please enter a number for parameter 'float32': 3
Please enter a comma separated array of numbers for parameter 'float64_arr':
        4
Please enter a number for parameter 'twist.linear.x': 5
Please enter a number for parameter 'twist.linear.y': 6
Please enter a number for parameter 'twist.linear.z': 7
Please enter a number for parameter 'twist.angular.x': 8
Please enter a number for parameter 'twist.angular.y': 9
Please enter a number for parameter 'twist.angular.z': 10
Successfully Generated service call...


{'/random': {'string': 'sample-string', 'int64_arr': [1.0, 2.0], 'float32': 3.0, 'float64_arr': [4.0], 'twist': {'twist.linear': {'twist.linear.x': 5.0, 'twist.linear.y': 6.0, 'twist.linear.z': 7.0}, 'twist.angular': {'twist.angular.x': 8.0, 'twist.angular.y': 9.0, 'twist.angular.z': 10.0}}}}
```

## Updating current services in Formant.io

It is possible to get this adapter to send back all the currently
active services along with their argument names and types.
To do this, simply add the command `ros.services.update-services` to 
Formant. Issuing this Command will cause the adapter to send back all
 currently active services to the stream `ros.services.json`.

## Trouble Shooting

You may attempt to run the service call adapter and
notice that one of the service's that is running
on your machine is not showing up on the stream
`ros.services.json`. The most common reason that this
occurs is that the `export CATKIN_WS=<catkin_ws dir>` is not located in `/var/lib/formant/.bashrc`. You 
can check by running `cat /var/lib/formant/.bashrc` and
checking if the `export CATKIN_WS=<catkin_ws dir>` is 
located in the file. If not, then simply add a new line
in the bashrc file, and replace `<catkin_ws dir>` with
the directory of your machines catkin workspace such
that `setup.bash` may be found at 
`<catkin_ws dir>/devel/setup.bash`.