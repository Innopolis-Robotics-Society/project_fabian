import time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class Profiler:
    def __init__(self):
        self.t0 = None
        self.fps = 0.0
        self.alpha = 0.1

    def tick(self):
        now = time.time()
        if self.t0 is None:
            self.t0 = now; return 0.0
        dt = now - self.t0
        self.t0 = now
        if dt > 0:
            self.fps = (1.0 - self.alpha) * self.fps + self.alpha * (1.0/dt)
        return dt

def build_diag(name, fps, latency_ms):
    arr = DiagnosticArray()
    st = DiagnosticStatus()
    st.name = name
    st.level = DiagnosticStatus.OK
    st.message = "ok"
    st.values = [
        KeyValue(key="fps", value=f"{fps:.2f}"),
        KeyValue(key="latency_ms", value=f"{latency_ms:.2f}")
    ]
    arr.status = [st]
    return arr
