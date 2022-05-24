#!/bin/bash

variable=$(./setup.py)

n=${#variable}
if [ $n -ne 0 ]; then
source $variable
echo "source $variable"
else
echo "No Source file found"
fi

/usr/bin/python main.py