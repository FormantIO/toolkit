

# Ros Bag Recorder Adapter

### About

This adapter was created to make the collection of ROS bags simple. The adapter allows for configurable bag lengths, configurable bag overlap, configurable topics to record from, and more. 

### Usage

To run the adapter, use `./adapter/start.sh`

### Adapter Configuration

This adapter can be configured in two ways. You may either adjust the config.json, or set key-value pairs on your account at Formant.io.

It is likely that users would like to configure the adapter from Formant.io. To do so, do device->configure->general->Application configuration->ADD CONFIGURATION. From there, you have the ability to add a key, which is the name of the configuration variable you wish to set, and then its value. 

#### Configuration parameters

The following outlines configuration parameters, what they do, and their possible values. 

1.) `subscribe_to_all` : true | false <br>
The subscribe_to_all configuration parameter specifies if you would like to subscribe to every available ros topic, less the ones listed in `ignore_topics` 

2.) `topics` : ["topic1", "topic2", ..., "topicN"]<br>
The topics parameter allows the user to specify which topics that they would like to subscribe to. This is used when `subscribe_to_all` is set to false.

3.) `ignore_topics` : ["topic1", "topic2", ..., "topicN"]<br>
The ignore topics parameter lists which topics to not subscribe to and record. This is used regardless of the value of `subscribe_to_all`.

4.) `topic_refresh_rate` : int <br>
This is the time, in seconds, between checking if there are new topics to subscribe to and record. 

5.) `bag_length` : int<br>
This specifies the length of each recorded ros bag, in seconds.

6.) `bag_overlap` : int<br>
This specifies the amount of overlap for each bag. If `bag_overlap` is set to 25 and `bag_length` is set to 100,
then 25 seconds before a bag closes, a new one will open, and they will both record 25 seconds of the same data until the first bad is closed. 

7.) `bag_storage_path` : str<br>
The file location which we would like to store the ros bags

8.) `bag_naming_convention` : str<br>
The way each bag will be named when saving to memory. The token $bn gets replaced by the bag number (its index), and $dt gets replaced by the current datetime. The datetime formant can be specified by the `date_time_string` parameter. 

9.) `date_time_string` : str<br>
This is a python datetime format string

10.) loglevel<br>
Coming soon