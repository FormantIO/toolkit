#! /bin/bash
export PYTHONUNBUFFERED=true
pip3 install setuptools
pip3 install wheel
python3 -m pip install -r requirements.txt

source config.env
python3 main.py