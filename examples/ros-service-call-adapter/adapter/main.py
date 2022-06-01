"""
main.py
ROS Service Call Adapter
-------------------------
A program that allows commands to be sent to the Formant agent, 
which are then processed as ROS service calls. The response
(if any) is posted to the stream named `ros.service.<service_name>.response`
"""

from adapter import Adapter

def main():
    try:
        adapter = Adapter()
        adapter.run()
    except KeyboardInterrupt:
        adapter.shutdown() 

if __name__ == "__main__":
    main()
