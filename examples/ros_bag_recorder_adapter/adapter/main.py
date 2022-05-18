#!/usr/bin/python

import logging
from utils import get_log_level
from adapter import Adapter

logging.basicConfig(level=get_log_level())
logger = logging.getLogger(__name__)

def main():
    adapter = Adapter()
    adapter.run()

if __name__ == "__main__": 
    logger.info("Running")
    main()