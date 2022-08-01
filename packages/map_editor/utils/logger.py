import logging


def init_logger():
    log = logging.getLogger('map_editor')
    level = logging.INFO
    log.setLevel(level)
    return log
