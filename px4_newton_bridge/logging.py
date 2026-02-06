import logging

import rerun as rr

logger = logging.getLogger("px4_newton_bridge")
logger.setLevel(logging.DEBUG)
logger.addHandler(rr.LoggingHandler("logs/bridge"))
