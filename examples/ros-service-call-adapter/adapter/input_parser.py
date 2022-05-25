"""
This parser safely parses the datatypes supported by ROS messages.
This allows a user to parse a string into data that could appear in a ROS message,
in a safe way. 

Implements a recursive descent parser for a few python primitives that could 
appear in a ROS message. 
"""

import ast

DELIMITERS = {"}", ")", "]", " ", "", ","}


def parse(input: str):
    """Parses a string for parsing data for use
        in ros messages

    The grammar:
    Primitive = Int | Float | String | Bool
    List = [ Primitive, ... ]
    Tuple = ( Primitive, ... )
    Set = { Primitive, ... }
    """

    rdp = _ROS_MSG_RDP(input)
    return rdp.parse_input()


class StringStream:
    """The string stream class allows the user to read a string of characters through peek_char and read_char."""

    def __init__(self, input: str):
        self.input = input
        self.str_index = 0

    def peek_char(self):
        return self.input[self.str_index] if self.str_index < len(self.input) else ""

    def read_char(self):
        return_char = self.input[self.str_index] if self.str_index < len(
            self.input) else ""
        self.str_index += 1
        return return_char

    def remove_leading_whitespace(self):
        while self.peek_char() == " ":
            self.read_char()

    def is_eof(self):
        return self.peek_char() == ""

    def validate_stream(self):
        if self.is_eof():
            raise SyntaxError(f"Unexpected EOF. Cannot parse {self.input}")


class _ROS_MSG_RDP:

    def __init__(self, input_str):
        self.string_stream = StringStream(input_str)

    def parse_input(self):
        if self.string_stream.peek_char() == "[":
            return self.parse_list()

        if self.string_stream.peek_char() == "(":
            return tuple(self.parse_list("(", ")"))

        if self.string_stream.peek_char() == "{":
            return set(self.parse_list("{", "}"))

        if self.string_stream.peek_char().isnumeric() or self.string_stream.peek_char() == "." or self.string_stream.peek_char() == "-":
            return self.parse_numeric()

        if self.string_stream.peek_char() == '"' or self.string_stream.peek_char() == "'":
            delimiter = self.string_stream.peek_char()
            return self.parse_string(delimiter)

        if self.string_stream.peek_char() == "F" or self.string_stream.peek_char() == "T":
            return self.parse_bool()

        raise SyntaxError(f"Cannot parse input")

    def parse_bool(self):
        if self.string_stream.peek_char() != "F" and self.string_stream.peek_char() != "T":
            raise SyntaxError(
                f"Invalid boolean expression. Cannot parse {self.string_stream.input}")

        if self.string_stream.peek_char() == "F":
            return self.parse_bool_false()

        if self.string_stream.peek_char() == "T":
            return self.parse_bool_true()

        raise SyntaxError(
            f"Invalid boolean. Cannot parse {self.string_stream.input}")

    def parse_bool_false(self):
        self.parse_from_stream("False")
        return False

    def parse_bool_true(self):
        self.parse_from_stream("True")
        return True

    def parse_from_stream(self, string_to_parse: str):
        for character in string_to_parse:
            if self.string_stream.read_char() != character:
                raise SyntaxError(f"Unmatched Character. Expected {character}")

        if self.string_stream.peek_char() not in DELIMITERS:
            raise SyntaxError("Expected Delimiter")

    def parse_numeric(self):
        if not self.string_stream.peek_char().isnumeric() and not self.string_stream.peek_char() == "."\
                and not self.string_stream.peek_char() == "-":
            raise SyntaxError(
                f"Invalid input. Cannot parse {self.string_stream.input}")

        if self.string_stream.peek_char().isnumeric():
            return self.parse_numeric_before_decimal()
        elif self.string_stream.peek_char() == "-":
            delim = self.string_stream.read_char()
            self.string_stream.validate_stream()
            return self.parse_numeric_before_decimal(delim)
        else:
            return self.parse_numeric_after_decimal(self.string_stream.read_char())

    def parse_numeric_before_decimal(self, soFar=""):

        if self.string_stream.peek_char().isnumeric():
            return self.parse_numeric_before_decimal(soFar+self.string_stream.read_char())

        elif self.string_stream.peek_char() == ".":
            return self.parse_numeric_after_decimal(soFar+self.string_stream.read_char())

        elif self.string_stream.peek_char() not in DELIMITERS:
            raise SyntaxError(f"Expected delimiter")

        return int(soFar)

    def parse_numeric_after_decimal(self, soFar=""):
        if self.string_stream.peek_char().isnumeric():
            return self.parse_numeric_after_decimal(soFar+self.string_stream.read_char())
        elif self.string_stream.peek_char() not in DELIMITERS:
            raise SyntaxError(
                f"Expected delimiter, got {self.string_stream.peek_char()}")
        return float(soFar)

    def parse_string(self, string_delimiter='"'):
        string = ""

        if self.string_stream.read_char() != string_delimiter:
            raise SyntaxError(
                f"Expected {string_delimiter} got {self.string_stream.peek_char()}")

        while self.string_stream.peek_char() != string_delimiter:

            if self.string_stream.peek_char() == "\\":
                string += self.parse_escaped()
            else:
                string += self.string_stream.read_char()

            self.string_stream.validate_stream()

        self.string_stream.read_char()

        return string

    def parse_escaped(self):
        if self.string_stream.peek_char() != "\\":
            raise SyntaxError(
                f"Expected \\, got {self.string_stream.peek_char()}. Cannot parse {self.string_stream.input}")

        self.string_stream.read_char()
        self.string_stream.validate_stream()
        return ast.literal_eval(f"'\\{self.string_stream.read_char()}'")

    def parse_list(self, list_start="[", list_delim="]"):
        if self.string_stream.read_char() != list_start:
            raise SyntaxError(
                f"Invalid List. Cannot parse {self.string_stream.input}")

        parsed_list = []
        self.string_stream.remove_leading_whitespace()

        if self.string_stream.peek_char() == ",":
            raise SyntaxError(
                f"Must have element before ,. Cannot parse {self.string_stream.input}")

        while self.string_stream.peek_char() != list_delim:

            self.string_stream.remove_leading_whitespace()
            parsed_list.append(self.parse_input())
            self.string_stream.validate_stream()
            self.string_stream.remove_leading_whitespace()

            if self.string_stream.peek_char() != ',' and self.string_stream.peek_char() != list_delim:
                raise SyntaxError("List must either have ] or ,")

            if self.string_stream.peek_char() == ",":
                self.string_stream.read_char()
                if self.string_stream.peek_char() == list_delim:
                    raise SyntaxError("Commas must separate elements. ")

        self.string_stream.read_char()
        return parsed_list
