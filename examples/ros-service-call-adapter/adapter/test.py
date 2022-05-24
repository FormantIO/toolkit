from input_parser import parse
from ast import Assert, literal_eval

def numeric_assert(val: str):
    val_calc = parse(val)
    val_actual = literal_eval(val) 
    if val_calc != val_actual:
        raise Exception("err")

numeric_assert("-1")
numeric_assert("-0.5")
numeric_assert("0.5")
numeric_assert(".5")
numeric_assert("-.1")
numeric_assert("1.123")


print(parse("[1, 2, 3, 4]"))
print(parse("{1, 2, 3, 4}"))
print(parse("(1, 2, 3, 4)"))

print(parse("False"))
print(parse("True"))

