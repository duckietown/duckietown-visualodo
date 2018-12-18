# coding=utf-8 
__version__ = '0.1.2'

import logging

logging.basicConfig()
logger = logging.getLogger('dt-visualodo')
logger.setLevel(logging.DEBUG)

logger.info('duckietown_visualodo %s' % __version__)

from .algo import *

