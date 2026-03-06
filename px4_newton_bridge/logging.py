import logging

import rerun as rr
from rerun.archetypes import TextLog


class _RerunHandler(rr.LoggingHandler):
    """Log to a single entity path with the module name prepended to the message."""

    def emit(self, record: logging.LogRecord) -> None:
        level = self.LVL2NAME.get(record.levelno)
        if level is None:
            level = rr.components.TextLogLevel(record.levelname)

        text = f"[sim/{record.module}] {record.getMessage()}"
        rr.log("logs/sim", TextLog(text, level=level))


logger = logging.getLogger("px4_newton_bridge")
logger.setLevel(logging.DEBUG)
logger.addHandler(_RerunHandler())
