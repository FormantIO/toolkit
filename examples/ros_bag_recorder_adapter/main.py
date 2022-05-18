#!/usr/bin/python

import rospy
from utils import *
from adapter import Adapter
import time
import threading

def main():
    adapter = Adapter()
    try:
        print(threading.active_count())
        adapter.run()
         #time.sleep(10)
        # print("Shutting Down")
        # adapter.shutdown()
        while True:
            time.sleep(1)
            print(threading.active_count())
            print(adapter.bag_thread.is_alive())
    except KeyboardInterrupt:
        print("Shutting down", flush=True)
        adapter.shutdown()

if __name__ == "__main__":
    logging.basicConfig(level=getattr(logging, "DEBUG", None))
    logging.debug("HELLO")
    main()
