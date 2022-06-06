import rospy
import rosservice
import rosmsg
from io import StringIO

print(rosservice.get_service_type("/random"))
print(rosmsg.get_srv_text(rosservice.get_service_type("/random")))

stream = StringIO(rosmsg.get_srv_text(rosservice.get_service_type("/random")))

for line in stream:
    print(f": {line}")

s_iter = iter(stream)

#while True:
#    print(next(s_iter)) 

def is_primitive(self):
    pass

class RosSRVParser:
    
    def __init__(self, service_name):
        self._service_name = service_name
        self._string_stream = StringIO(self._service_name) 
        self._stream_iter = StreamWrapper(
            rosmsg.get_srv_text(
                rosservice.get_service_type(service_name)))

        self._input_params = [] 

        self._parse()

    def _parse(self):
        pass

class StreamWrapper:

    def __init__(self, input):
        self._input = input
        self._stream = StringIO(input) 
        self._s_iter = iter(self._stream) 
        self._eof = False

    def next_line(self):
        try:
            if self._next_line:
                next_line = self._next_line
                self._next_line = None
                return next_line
            return next(self._s_iter) 
        except StopIteration:
            return ""
    
    def has_next_line(self):
        try:
            self._next_line = next(self._s_iter)
            return True
        except StopIteration:
            return False

    def peek_next_line(self):
        if self._next_line:
            return self._next_line
        try:
            self._next_line = next(self._s_iter)
            return self._next_line
        except StopIteration:
            return "" 

    def __iter__(self):
        return iter(self._stream) 

class RosType:

    def __init__(self, stream: StreamWrapper):
        self._stream = stream 
        self._children = []
        self._parse()


    def _parse(self):
        _next_line = self._stream.next_line() 
        self.data_type, self.data_name = _next_line.rstrip().lstrip().replace("\n").split(" ")

        if is_primitive(self.data_type):
            return
        
