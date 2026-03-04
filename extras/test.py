"""
live_tf_arduino.py
──────────────────
Reads timestamped step-wave samples from an Arduino via pySerialTransfer,
runs them through a discrete-time model of a continuous transfer function,
sends the timestamped output back to the Arduino, and plots everything live.

Dependencies
------------
    pip install pySerialTransfer numpy scipy matplotlib

Packet protocol
---------------
  Arduino → PC  :  struct { uint32_t timestamp_ms;  float u; }   (8 bytes)
  PC → Arduino  :  struct { uint32_t timestamp_ms;  float y; }   (8 bytes)
"""

import threading
import queue
import time
import struct

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy import signal

try:
    from pySerialTransfer import pySerialTransfer as pst
except ImportError:
    raise SystemExit(
        "pySerialTransfer not found.\n"
        "Install it with:  pip install pySerialTransfer"
    )

# ─────────────────────────────────────────────────────────────────────────────
# USER CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────

# Serial port – change to match your system:
#   Windows  →  'COM3'
#   macOS    →  '/dev/cu.usbmodem...'
#   Linux    →  '/dev/ttyUSB0'  or  '/dev/ttyACM0'
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE   = 115200

# Continuous-time transfer function  H(s) = NUM(s) / DEN(s)
# Coefficients in descending power order (highest power first).
# Default: second-order low-pass   H(s) = 1 / (s² + 0.5s + 1)
NUM = [1.0]
DEN = [1.0, 0.5, 1.0]

# Sample period — must match SAMPLE_MS in the Arduino sketch.
DT = 0.05   # seconds (20 Hz)

# Rolling plot window (seconds of history shown).
WINDOW_SEC = 10.0

# ─────────────────────────────────────────────────────────────────────────────


# ── Transfer-function helpers ─────────────────────────────────────────────────

def discretise(num, den, dt):
    """Bilinear (Tustin) transform: continuous TF → discrete IIR b/a arrays."""
    sys_c = signal.TransferFunction(num, den)
    sys_d = sys_c.to_discrete(dt, method='bilinear')
    return sys_d.num, sys_d.den


def apply_iir(b, a, x_buf, y_buf, x_new):
    """Evaluate the IIR difference equation for one new sample."""
    order = len(a) - 1

    x_buf.append(x_new)
    if len(x_buf) > order + 1:
        x_buf.pop(0)

    xv = list(reversed(x_buf))
    while len(xv) < len(b):
        xv.append(0.0)

    y_new = sum(b[i] * xv[i] for i in range(len(b)))

    yv = list(reversed(y_buf))
    while len(yv) < order:
        yv.append(0.0)
    y_new -= sum(a[i + 1] * yv[i] for i in range(order))
    y_new /= a[0]

    y_buf.append(y_new)
    if len(y_buf) > order:
        y_buf.pop(0)

    return y_new


# ── Shared state ──────────────────────────────────────────────────────────────

data_q     = queue.Queue()   # (t_sec, u, y) tuples ready to plot
send_q     = queue.Queue()   # (timestamp_ms, y) pairs waiting to send
stop_event = threading.Event()


# ── Serial worker thread ──────────────────────────────────────────────────────

def serial_worker(port, baud, b, a, x_buf, y_buf):
    """
    Owns the serial link. Loops at ~1 ms:
      1. Check for an incoming packet from the Arduino (timestamp_ms, u).
      2. Compute y, enqueue (t_sec, u, y) for the plot, enqueue
         (timestamp_ms, y) to send back.
      3. Send any pending (timestamp_ms, y) to the Arduino.
    """
    link = pst.SerialTransfer(port, baud)
    link.open()
    print(f"[serial] opened {port} @ {baud}")
    time.sleep(2)   # allow Arduino to reset after DTR

    t0 = None

    while not stop_event.is_set():

        # ── Receive ──────────────────────────────────────────────────────────
        if link.available():
            if link.bytesRead >= 8:
                raw = bytes(link.rxBuff[:8])
                timestamp_ms, u = struct.unpack('<If', raw)

                if t0 is None:
                    t0 = timestamp_ms
                t_sec = (timestamp_ms - t0) / 1000.0

                y = apply_iir(b, a, x_buf, y_buf, float(u))
                data_q.put((t_sec, float(u), y))
                send_q.put((timestamp_ms, y))
            else:
                print(f"[serial] unexpected packet size: {link.bytesRead} bytes")

        elif link.status < 0:
            bad = {
                pst.CRC_ERROR       : 'CRC_ERROR',
                pst.PAYLOAD_ERROR   : 'PAYLOAD_ERROR',
                pst.STOP_BYTE_ERROR : 'STOP_BYTE_ERROR',
            }
            name = bad.get(link.status)
            if name:
                print(f"[serial] {name}")

        # ── Send ─────────────────────────────────────────────────────────────
        if not send_q.empty():
            ts_ms, y_out = send_q.get_nowait()
            # Pack as { uint32_t timestamp_ms, float y } — 8 bytes little-endian
            raw_out = struct.pack('<If', ts_ms, float(y_out))
            send_size = link.tx_obj(raw_out, start_pos=0, object_size=8)
            link.send(send_size)

        time.sleep(0.001)

    link.close()
    print("[serial] closed")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    b, a = discretise(NUM, DEN, DT)
    print(f"Continuous TF  num={NUM}  den={DEN}")
    print(f"Discrete (dt={DT}s)  b={np.round(b, 6)}  a={np.round(a, 6)}")

    x_buf, y_buf = [], []

    worker = threading.Thread(
        target=serial_worker,
        args=(SERIAL_PORT, BAUD_RATE, b, a, x_buf, y_buf),
        daemon=True
    )
    worker.start()

    # ── Plot setup ────────────────────────────────────────────────────────────
    ts, us, ys = [], [], []

    fig, (ax_u, ax_y) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle(
        f"Live Arduino → TF Simulation → Arduino\n"
        f"H(s) = {NUM} / {DEN}    dt = {DT} s    port = {SERIAL_PORT}",
        fontsize=10
    )

    line_u, = ax_u.plot([], [], 'b-', lw=1.5, label='Input u(t)  [step wave from Arduino]')
    line_y, = ax_y.plot([], [], 'r-', lw=1.5, label='Output y(t) [system response → Arduino]')

    for ax, ylabel in ((ax_u, 'Input u'), (ax_y, 'Output y')):
        ax.set_ylabel(ylabel)
        ax.legend(loc='upper right', fontsize=9)
        ax.grid(True, alpha=0.3)
    ax_y.set_xlabel('Time (s)')

    status_txt = ax_u.text(
        0.01, 0.95, 'Waiting for data…',
        transform=ax_u.transAxes, fontsize=8,
        verticalalignment='top', color='gray'
    )

    def update(_frame):
        got_new = False
        while not data_q.empty():
            t_new, u_new, y_new = data_q.get_nowait()
            ts.append(t_new)
            us.append(u_new)
            ys.append(y_new)
            got_new = True

        if not got_new or len(ts) < 2:
            return line_u, line_y, status_txt

        t_min = ts[-1] - WINDOW_SEC
        idx   = next((i for i, t in enumerate(ts) if t >= t_min), 0)
        t_w, u_w, y_w = ts[idx:], us[idx:], ys[idx:]

        line_u.set_data(t_w, u_w)
        line_y.set_data(t_w, y_w)

        ax_u.set_xlim(t_w[0], max(t_w[-1], t_w[0] + WINDOW_SEC))
        ax_u.relim(); ax_u.autoscale_view(scalex=False, scaley=True)
        ax_y.relim(); ax_y.autoscale_view(scalex=False, scaley=True)

        status_txt.set_text(
            f"t = {ts[-1]:.2f} s    u = {us[-1]:.4f}    y = {ys[-1]:.4f}    "
            f"n = {len(ts)} samples"
        )
        status_txt.set_color('black')

        return line_u, line_y, status_txt

    ani = animation.FuncAnimation(
        fig, update, interval=int(DT * 1000),
        blit=False, cache_frame_data=False
    )

    try:
        plt.tight_layout()
        plt.show()
    finally:
        stop_event.set()


if __name__ == '__main__':
    main()