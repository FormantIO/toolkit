class RosTopicStats:
    def __init__(self, name, type):
        self.name = name
        self.type = type
        self.hz = 0

    def set_hz(self, hz):
        self.hz = hz
