"""
live_tf_sim.py – Live discrete-time simulation of a continuous-time transfer function.

Usage
-----
Edit the TRANSFER FUNCTION section below, then run:

    python live_tf_sim.py

The script reads timestamped input samples from stdin (or a background thread
that generates them), converts the continuous TF to a discrete IIR filter via
scipy's bilinear (Tustin) method, applies it sample-by-sample, and updates a
live matplotlib plot.

Input format (stdin mode)
--------------------------
Each line: "t,u"   e.g.  0.05,1.0
Press Ctrl-C to stop.

Demo mode (default)
-------------------
A background thread generates a step + sine input automatically so you can
watch it immediately without piping data.
"""

import threading
import queue
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy import signal

# ─────────────────────────────────────────────────────────────────────────────
# TRANSFER FUNCTION  (continuous-time, s-domain)
# Edit numerator / denominator coefficient lists (highest power first).
#
# Default: second-order low-pass   H(s) = 1 / (s^2 + 0.5s + 1)
# ─────────────────────────────────────────────────────────────────────────────
NUM = [1.0]               # numerator   coefficients  e.g. [2, 1] → 2s + 1
DEN = [1.0, 0.5, 1.0]    # denominator coefficients  e.g. [1, 3, 2] → s² + 3s + 2

# Desired sample period for discretisation (seconds).
# If you pipe real data, this should match your data rate.
DT = 0.05   # 20 Hz

# How many seconds of history to show on the plot.
WINDOW_SEC = 10.0

# Set to True to read "t,u" lines from stdin instead of the demo generator.
USE_STDIN = False
# ─────────────────────────────────────────────────────────────────────────────


def discretise(num, den, dt):
    """Convert continuous TF to discrete IIR coefficients via Tustin (bilinear)."""
    sys_c = signal.TransferFunction(num, den)
    sys_d = sys_c.to_discrete(dt, method='bilinear')
    # scipy returns b (numerator), a (denominator) for the difference equation
    return sys_d.num, sys_d.den


def apply_iir(b, a, x_buf, y_buf, x_new):
    """
    Compute the next output sample using the IIR difference equation.
    x_buf / y_buf hold past input/output samples (oldest first).
    """
    order = len(a) - 1  # filter order
    # y[n] = (1/a[0]) * (b[0]*x[n] + b[1]*x[n-1] + ... - a[1]*y[n-1] - ...)
    x_buf.append(x_new)
    if len(x_buf) > order + 1:
        x_buf.pop(0)
    xv = list(reversed(x_buf))   # xv[0] = x[n], xv[1] = x[n-1], …

    # pad if buffer not full yet
    while len(xv) < len(b):
        xv.append(0.0)

    y_new = sum(b[i] * xv[i] for i in range(len(b)))
    yv = list(reversed(y_buf))   # yv[0] = y[n-1], yv[1] = y[n-2], …
    while len(yv) < order:
        yv.append(0.0)
    y_new -= sum(a[i + 1] * yv[i] for i in range(order))
    y_new /= a[0]

    y_buf.append(y_new)
    if len(y_buf) > order:
        y_buf.pop(0)

    return y_new


# ── Data queue shared between producer thread and animation callback ──────────
data_q = queue.Queue()


def demo_producer(dt):
    """Generate a step + sinusoidal input at a fixed sample rate."""
    t = 0.0
    while True:
        u = 1.0 + 0.3 * np.sin(2 * np.pi * 0.5 * t)   # DC step + 0.5 Hz sine
        data_q.put((t, u))
        t += dt
        time.sleep(dt)


def stdin_producer():
    """Read 't,u' lines from stdin."""
    import sys
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            t_str, u_str = line.split(',')
            data_q.put((float(t_str), float(u_str)))
        except ValueError:
            print(f"[warn] skipping malformed line: {line!r}")


def main():
    # Discretise
    b, a = discretise(NUM, DEN, DT)
    print(f"Continuous TF  num={NUM}  den={DEN}")
    print(f"Discrete   (dt={DT}s)  b={np.round(b,6)}  a={np.round(a,6)}")

    # State buffers
    x_buf, y_buf = [], []

    # Rolling data for the plot
    max_pts = int(WINDOW_SEC / DT) + 2
    ts, us, ys = [], [], []

    # Start producer
    producer = threading.Thread(
        target=stdin_producer if USE_STDIN else demo_producer,
        args=() if USE_STDIN else (DT,),
        daemon=True
    )
    producer.start()

    # ── Set up matplotlib ────────────────────────────────────────────────────
    fig, (ax_u, ax_y) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle(
        f"Live Discrete-Time Simulation\nH(s) = {NUM} / {DEN}   dt = {DT} s",
        fontsize=11
    )

    line_u, = ax_u.plot([], [], 'b-', lw=1.5, label='Input u(t)')
    line_y, = ax_y.plot([], [], 'r-', lw=1.5, label='Output y(t)')

    ax_u.set_ylabel('Input')
    ax_u.legend(loc='upper right')
    ax_u.grid(True, alpha=0.3)

    ax_y.set_ylabel('Output')
    ax_y.set_xlabel('Time (s)')
    ax_y.legend(loc='upper right')
    ax_y.grid(True, alpha=0.3)

    def update(_frame):
        # Drain the queue
        got_new = False
        while not data_q.empty():
            t_new, u_new = data_q.get_nowait()
            y_new = apply_iir(b, a, x_buf, y_buf, u_new)
            ts.append(t_new)
            us.append(u_new)
            ys.append(y_new)
            got_new = True

        if not got_new or len(ts) < 2:
            return line_u, line_y

        # Trim to window
        t_min = ts[-1] - WINDOW_SEC
        idx = next((i for i, t in enumerate(ts) if t >= t_min), 0)
        t_win = ts[idx:]
        u_win = us[idx:]
        y_win = ys[idx:]

        line_u.set_data(t_win, u_win)
        line_y.set_data(t_win, y_win)

        ax_u.set_xlim(t_win[0], max(t_win[-1], t_win[0] + WINDOW_SEC))
        ax_u.relim(); ax_u.autoscale_view(scalex=False, scaley=True)
        ax_y.relim(); ax_y.autoscale_view(scalex=False, scaley=True)

        return line_u, line_y

    ani = animation.FuncAnimation(
        fig, update, interval=int(DT * 1000), blit=False, cache_frame_data=False
    )

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()