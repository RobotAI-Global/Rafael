

import logging 
logger         = logging.getLogger("robotai")

# Check if a logger named 'logger' is already defined
if not logger.handlers:
    # Logger is not configured, so configure it

    #formatter   = logging.Formatter('[%(asctime)s.%(msecs)03d] {%(filename)6s:%(lineno)3d} %(levelname)s - %(message)s', datefmt="%M:%S", style="{")
    formatter       = logging.Formatter('[%(asctime)s] - [%(filename)16s:%(lineno)4d] - %(levelname)5s - %(message)s')
    #formatter       = logging.Formatter('[%(asctime)s] - %(levelname)s - %(message)s')
    logger.setLevel("DEBUG")
    
    console_handler = logging.StreamHandler()
    console_handler.setLevel("DEBUG")
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)

    # file_handler = logging.FileHandler("MainApp.log", mode="a", encoding="utf-8")
    # file_handler.setLevel("WARNING")
    # file_handler.setFormatter(formatter)
    # logger.addHandler(file_handler)


# class CustomFormatter(logging.Formatter):

#     grey = "\x1b[38;20m"
#     yellow = "\x1b[33;20m"
#     red = "\x1b[31;20m"
#     bold_red = "\x1b[31;1m"
#     reset = "\x1b[0m"
#     format = (
#         "[%(asctime)s][%(name)s][%(levelname)s] : %(message)s :(%(filename)s:%(lineno)d)"
#     )

#     FORMATS = {
#         logging.DEBUG: grey + format + reset,
#         logging.INFO: grey + format + reset,
#         logging.WARNING: yellow + format + reset,
#         logging.ERROR: red + format + reset,
#         logging.CRITICAL: bold_red + format + reset,
#     }

#     def format(self, record):
#         log_fmt = self.FORMATS.get(record.levelno)
#         formatter = logging.Formatter(log_fmt,datefmt="%Y-%m-%d %H:%M:%S")
#         return formatter.format(record)


# def get_console_handler():
#     console_handler = logging.StreamHandler(sys.stdout)
#     console_handler.setLevel(eval('logging.'+LOGLEVEL))
#     console_handler.setFormatter(CustomFormatter())
#     return console_handler


# def get_logger(logger_name):
#     logger = logging.getLogger(logger_name)
#     if not logger.hasHandlers():
#         logger.addHandler(get_console_handler())
#     logger.setLevel(logging.DEBUG)
#     logger.propagate = False
#     return logger

# neurapy_logger = get_logger("robotai")

logger.info('Logger started')
