
from io import StringIO


def get_indentation(line):
    """Return the indentation level of a line"""
    return len(line) - len(line.lstrip())


class IndentedStream:
    """
    An indented stream allows one to look at both 
    indentation level and the current line."""

    def __init__(self, string):
        self._string = string
        self._stream = StringIO(string)
        self._s_iter = iter(self._stream)

        self._next_line = None
        self._next_indentation = None

    def peek_indentation(self):
        """Peek the next """
        if self._next_indentation is not None:
            return self._next_indentation
        self.read()
        return self._next_indentation

    def peek_line(self):
        if self._next_line:
            return self._next_line
        self.read()
        return self._next_line

    def read(self):
        try:
            self._next_line = next(self._s_iter)
            self._next_indentation = get_indentation(self._next_line)
        except StopIteration:
            self._next_line = None
            self._next_indentation = None


class IndentStreamParser:

    def __init__(self, input):
        self._input = input
        self._stream = IndentedStream(input)

        self._base_indentation = self._stream.peek_indentation()

    def parse(self):
        objs = []

        while(self._stream.peek_indentation() != None):
            objs.append(self._parse())

        return objs

    def _parse(self):
        current_indentation = self._stream.peek_indentation()
        key = self._stream.peek_line()
        children = []

        self._stream.read()

        while(self._stream.peek_indentation() != None and
                self._stream.peek_indentation() > current_indentation):
            children.append(self._parse())

        return {"name": key, "children": children}


def parse_indented_string(string):
    """
    Parse a string which uses indentation for parent-children
    relationships to a json which captures the relationships.

    ex.)
        'parent
            child1
            child2'
        parses to
        [
            {
                "name":"parent",
                "children":[
                    {"name":"child1", children:[]},
                    {"name":"child2", children:[]}
                ]
            }
        ]

    """
    parser = IndentStreamParser(string)
    parsed = parser.parse()
    return parsed
