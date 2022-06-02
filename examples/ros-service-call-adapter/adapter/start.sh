#!/bin/bash

variable=$(./setup.py)

n=${#variable}
if [ $n -ne 0 ]; then
    source $variable
    echo "source $variable"
fi

pip3 install -r requirements.txt

/usr/bin/python3 main.py
