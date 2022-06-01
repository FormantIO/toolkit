#!/bin/bash

variable=$(./setup.py)

n=${#variable}
if [ $n -ne 0 ]; then
    source $variable
    echo "source $variable"
fi

/usr/bin/python3 main.py
