
# log_level = 1[debug], 2[info], 3[warn], 4[fatal]
log_level = 1

def debug(msg):
    if log_level <= 1:
        print(f"DEBUG: {msg}")

def info(msg):
    if log_level <= 2:
        print(f"INFO: {msg}")

def warn(msg):
    if log_level <= 3:
        print(f"WARN: {msg}")

def fatal(msg):
    if log_level <= 4:
        print(f"FATAL: {msg}")

def getLogger(*args, **kwargs):
    return getLogger

getLogger.debug = debug
getLogger.info = info
getLogger.warn = warn
getLogger.fatal = fatal
getLogger.critical = fatal
getLogger.warning = warn 