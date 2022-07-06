
# ROS Button Toggle Adapter

The button toggle adapter gives state to Formant buttons.
Natively, buttons in Formant are only in the **ON** state 
when they are being pressed by the user. Once the button is 
released, the button is reverted back to the **OFF** state. 
To combat this issue, the button toggle adapter provides 
the ability for the user to add toggle-able state to the button 
by only button state on an output topic for each 
**ON**->**OFF** state transition of the Formant button. 

# How to use 

To use the adapter, simply add the name of the button in the 
config.json file. If the button is setup using the API button interface,
then add the name of the API button and the output ROS topic. 
If the button is a ROS button, then add a ROS topic button. Thats it! 
Just add the adapter to your robot and it will work!