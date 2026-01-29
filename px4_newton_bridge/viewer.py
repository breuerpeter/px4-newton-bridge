"""Custom Rerun viewer with PX4 bridge blueprint (3D + status + logs)."""

import time

import rerun as rr
import rerun.blueprint as rrb
from newton.viewer import ViewerRerun


class Viewer(ViewerRerun):
    """ViewerRerun subclass with a custom blueprint for the PX4 Newton bridge.

    Layout::

        Vertical
        ├── Horizontal
        │   ├── Spatial3DView (root /)
        │   └── TextDocumentView (status)
        └── TextLogView (logs)
    """

    def __init__(self, sim_dt: float, **kwargs):
        self._sim_dt = sim_dt
        self._frame_count = 0
        self._last_fps_time = time.time()
        self._fps_interval = 0.5
        super().__init__(**kwargs)

    def _get_blueprint(self):
        return rrb.Blueprint(
            rrb.Vertical(
                rrb.Horizontal(
                    rrb.Spatial3DView(origin="/"),
                    rrb.TextDocumentView(origin="status", name="Status"),
                    column_shares=[3, 1],
                ),
                rrb.TextLogView(origin="logs", name="Logs"),
                row_shares=[3, 1],
            ),
            rrb.TimePanel(state="collapsed"),
            collapse_panels=True,
        )

    def end_frame(self):
        super().end_frame()
        self._frame_count += 1
        now = time.time()
        elapsed = now - self._last_fps_time
        if elapsed >= self._fps_interval:
            sps = self._frame_count / elapsed
            rtf = sps * self._sim_dt
            rr.log("status", rr.TextDocument(
                f"**Real time factor:** {rtf:.2f}",
                media_type=rr.MediaType.MARKDOWN,
            ))
            self._frame_count = 0
            self._last_fps_time = now
