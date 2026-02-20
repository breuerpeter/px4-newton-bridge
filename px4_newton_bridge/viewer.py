import time
from pathlib import Path

import rerun as rr
import rerun.blueprint as rrb
from newton.viewer import ViewerRerun

_LOG_ROOT = (
    Path(__file__).resolve().parents[5]
    / "build"
    / "px4_sitl_default"
    / "rootfs"
    / "log"
)


class Viewer(ViewerRerun):
    """ViewerRerun subclass with a custom blueprint for the PX4 Newton bridge.

    Layout::

        Vertical
        ├── Horizontal
        │   ├── Spatial3DView (root /)
        │   └── TextDocumentView (status)
        └── TextLogView (logs)
    """

    def __init__(self, cfg: dict):
        self._sim_dt = cfg["sim"]["dt"]
        self._frame_count = 0
        self._last_fps_time = time.time()
        self._fps_interval = 0.5
        self._want_replay = cfg["viewer"]["want_replay"]
        self._export_rrd = cfg["viewer"]["export_rrd"]
        super().__init__(
            rec_id="px4-newton",
            keep_historical_data=self._want_replay,
        )
        # TODO: rr.save() already called by super().__init__() but doesn't work (file is very small and doesn't contain any logged data). The following is a workaround
        if self._export_rrd:
            _LOG_ROOT.mkdir(parents=True, exist_ok=True)
            rr.save(str(_LOG_ROOT / "newton.rrd"))

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
            rr.log(
                "status",
                rr.TextDocument(
                    f"**Real time factor:** {rtf:.2f}",
                    media_type=rr.MediaType.MARKDOWN,
                ),
            )
            self._frame_count = 0
            self._last_fps_time = now
