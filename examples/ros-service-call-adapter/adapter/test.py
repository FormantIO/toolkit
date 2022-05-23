from input_parser import parse

print(parse("[.1,2.3,'hello']"))
print(parse('"Hello there\ng"'))
print(parse("'h\\"+"nq'"))

print(parse("\\"+"\""))