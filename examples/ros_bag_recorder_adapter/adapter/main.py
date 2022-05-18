#!/usr/bin/python

import logging
from utils import get_log_level
from adapter import Adapter

logging.basicConfig(level=get_log_level())

def main():
    adapter = Adapter()
    adapter.run()

if __name__ == "__main__": 
    logging.debug("Running")
    logging.info("Running")
    main()