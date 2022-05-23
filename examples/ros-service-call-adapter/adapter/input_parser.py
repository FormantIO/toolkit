import ast

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

    if string_stream.peek_char().isnumeric() or string_stream.peek_char() == ".":
        return parse_numeric(string_stream)

    if string_stream.peek_char() == '"' or string_stream.peek_char() == "'":
        delimiter = string_stream.peek_char()
        return parse_string(string_stream, delimiter)

    if string_stream.peek_char() == "F" or string_stream.peek_char() == "T":
        return parse_bool(string_stream)      

def parse_bool(string_stream: StringStream):
    if string_stream.peek_char() != "F" and string_stream.peek_char() != "T":
        raise SyntaxError(f"Invalid boolean expression. Cannot parse {string_stream.input}")

    if string_stream.peek_char() == "F":
        return parse_bool_false(string_stream)

def parse_bool_false(string_stream):
    parse_from_stream(string_stream, "False")
    return False 

def parse_bool_true(string_stream):
    parse_from_stream(string_stream, "True")
    return True

def parse_from_stream(string_stream:StringStream, string_to_parse:str):
    for character in string_to_parse:
        if string_stream.read_char() != character:
            SyntaxError(f"Unmatched Character. Expected {character}")

def parse_numeric(string_stream: StringStream):
    if not string_stream.peek_char().isnumeric() or string_stream.peek_char() == ".":
        raise SyntaxError(f"Invalid input. Cannot parse {string_stream.input}")

    if string_stream.peek_char().isnumeric():
        return parse_numeric_before_decimal(string_stream)
    return parse_numeric_after_decimal(string_stream) 

def parse_numeric_before_decimal(string_stream: StringStream, soFar = ""):
    
    if string_stream.peek_char().isnumeric():
        return parse_numeric_before_decimal(string_stream, soFar+string_stream.read_char())

    if string_stream.peek_char() == ".":
        return parse_numeric_after_decimal(string_stream, soFar+string_stream.read_char()) 

    return int(soFar) 

def parse_numeric_after_decimal(string_stream: StringStream, soFar = ""):
    if string_stream.peek_char().isnumeric():
        return parse_numeric_after_decimal(string_stream, soFar+string_stream.read_char())

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

def parse_list(string_stream: StringStream):
    if string_stream.read_char() != "[":
        raise SyntaxError(f"Invalid List. Cannot parse {string_stream.input}")
    
    parsed_list = []
    string_stream.remove_leading_whitespace()
    
    if string_stream.peek_char() == ",":
        raise SyntaxError(f"Must have element before ,. Cannot parse {string_stream.input}")
    
    while string_stream.peek_char() != "]":
        
        string_stream.remove_leading_whitespace()
        parsed_list.append(parse_input(string_stream))
        string_stream.validate_stream()
        string_stream.remove_leading_whitespace()
        
        if string_stream.peek_char() != ',' and string_stream.peek_char() != ']':
            raise SyntaxError("List must either have ] or ,")

        if string_stream.peek_char() == ",":
            string_stream.read_char()
            if string_stream.peek_char() == "]":
                raise SyntaxError("Commas must separate elements. ")

    string_stream.read_char()
    return parsed_list
