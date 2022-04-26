import time
from ros2_bag_recorder import Ros2BagRecorder

if __name__ == "__main__":
    ros2_bag_recorder = Ros2BagRecorder()
    ros2_bag_recorder.run()
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        ros2_bag_recorder.stop()
        exit()
