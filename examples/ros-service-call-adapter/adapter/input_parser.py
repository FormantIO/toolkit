"""
This parser safely parses the datatypes supported by ROS messages.
This allows a user to parse a string into data that could appear in a ROS message,
in a safe way. 

Implements a recursive descent parser for a few python primitives that could 
appear in a ROS message. 
"""

import ast

from pkg_resources import parse_version

DELIMITERS = {"}", ")", "]", " ", "", ","}

class StringStream:
    def __init__(self, input: str):
        self.input = input
        self.str_index = 0
    
    def peek_char(self):
        return self.input[self.str_index] if self.str_index < len(self.input) else ""

    def read_char(self):
        return_char = self.input[self.str_index] if self.str_index < len(self.input) else ""
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

def parse(input: str):
    ss = StringStream(input) 
    return parse_input(ss)

def parse_input(string_stream: StringStream):
    if string_stream.peek_char() == "[":
        return parse_list(string_stream) 

    if string_stream.peek_char() == "(":
        return tuple(parse_list(string_stream, "(", ")"))
    
    if string_stream.peek_char() == "{":
        return set(parse_list(string_stream, "{", "}"))

    if string_stream.peek_char().isnumeric() or string_stream.peek_char() == "." or string_stream.peek_char() == "-":
        return parse_numeric(string_stream)

    if string_stream.peek_char() == '"' or string_stream.peek_char() == "'":
        delimiter = string_stream.peek_char()
        return parse_string(string_stream, delimiter)

    if string_stream.peek_char() == "F" or string_stream.peek_char() == "T":
        return parse_bool(string_stream)      

    raise SyntaxError(f"Cannot parse input")

def parse_bool(string_stream: StringStream):
    if string_stream.peek_char() != "F" and string_stream.peek_char() != "T":
        raise SyntaxError(f"Invalid boolean expression. Cannot parse {string_stream.input}")

    if string_stream.peek_char() == "F":
        return parse_bool_false(string_stream)

    if string_stream.peek_char() == "T":
        return parse_bool_true(string_stream) 
    
    raise SyntaxError(f"Invalid boolean. Cannot parse {string_stream.input}")

def parse_bool_false(string_stream):
    parse_from_stream(string_stream, "False")
    return False 

def parse_bool_true(string_stream):
    parse_from_stream(string_stream, "True")
    return True

def parse_from_stream(string_stream:StringStream, string_to_parse:str):
    for character in string_to_parse:
        if string_stream.read_char() != character:
            raise SyntaxError(f"Unmatched Character. Expected {character}")

    if string_stream.peek_char() not in DELIMITERS:
        raise SyntaxError("Expected Delimiter")

def parse_numeric(string_stream: StringStream):
    if not string_stream.peek_char().isnumeric() and not string_stream.peek_char() == "."\
        and not string_stream.peek_char() == "-":
        raise SyntaxError(f"Invalid input. Cannot parse {string_stream.input}")

    if string_stream.peek_char().isnumeric():
        return parse_numeric_before_decimal(string_stream)
    elif string_stream.peek_char() == "-":
        delim = string_stream.read_char()
        string_stream.validate_stream()
        return parse_numeric_before_decimal(string_stream, delim)
    else:
        return parse_numeric_after_decimal(string_stream, string_stream.read_char()) 

def parse_numeric_before_decimal(string_stream: StringStream, soFar = ""):
    
    if string_stream.peek_char().isnumeric():
        return parse_numeric_before_decimal(string_stream, soFar+string_stream.read_char())

    elif string_stream.peek_char() == ".":
        return parse_numeric_after_decimal(string_stream, soFar+string_stream.read_char()) 

    elif string_stream.peek_char() not in DELIMITERS:
        raise SyntaxError(f"Expected delimiter")

    return int(soFar) 

def parse_numeric_after_decimal(string_stream: StringStream, soFar = ""):
    if string_stream.peek_char().isnumeric():
        return parse_numeric_after_decimal(string_stream, soFar+string_stream.read_char())
    elif string_stream.peek_char() not in DELIMITERS:
        raise SyntaxError(f"Expected delimiter, got {string_stream.peek_char()}")
    return float(soFar) 

def parse_string(string_stream: StringStream, string_delimiter='"'):
    string = ""

    if string_stream.read_char() != string_delimiter:
        raise SyntaxError(f"Expected {string_delimiter} got {string_stream.peek_char()}")

    while string_stream.peek_char() != string_delimiter:
        
        if string_stream.peek_char() == "\\":
            string += parse_escaped(string_stream) 
        else:
            string+=string_stream.read_char()

        string_stream.validate_stream()

    string_stream.read_char()

    return string 

def parse_escaped(string_stream: StringStream):
    if string_stream.peek_char() != "\\":
        raise SyntaxError(f"Expected \\, got {string_stream.peek_char()}. Cannot parse {string_stream.input}")
    
    string_stream.read_char()
    string_stream.validate_stream()
    return ast.literal_eval(f"'\\{string_stream.read_char()}'")

def parse_list(string_stream: StringStream, list_start = "[", list_delim = "]"):
    if string_stream.read_char() != list_start:
        raise SyntaxError(f"Invalid List. Cannot parse {string_stream.input}")
    
    parsed_list = []
    string_stream.remove_leading_whitespace()
    
    if string_stream.peek_char() == ",":
        raise SyntaxError(f"Must have element before ,. Cannot parse {string_stream.input}")
    
    while string_stream.peek_char() != list_delim:
        
        string_stream.remove_leading_whitespace()
        parsed_list.append(parse_input(string_stream))
        string_stream.validate_stream()
        string_stream.remove_leading_whitespace()
        
        if string_stream.peek_char() != ',' and string_stream.peek_char() != list_delim:
            raise SyntaxError("List must either have ] or ,")

        if string_stream.peek_char() == ",":
            string_stream.read_char()
            if string_stream.peek_char() == list_delim:
                raise SyntaxError("Commas must separate elements. ")

    string_stream.read_char()
    return parsed_list
