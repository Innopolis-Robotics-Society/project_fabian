import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter

# frames: numpy array of shape [T, 17, 3]  (x, y, confidence)

frames = np.load("data/anim.npy")
print("frames shape:", frames.shape)

# Connections between joints (common COCO format; adjust if yours differs)
bones = [
    (0, 1), (0, 2), (1, 3), (2, 4),                     # head region
    (5, 6), (5, 7), (7, 9), (6, 8), (8, 10),            # arms
    (11, 12), (5, 11), (6, 12),                         # torso
    (11, 13), (13, 15), (12, 14), (14, 16)              # legs
]

fig, ax = plt.subplots()
ax.set_aspect("equal")

# Extract min/max for frame bounds
xs = frames[:, :, 0]
ys = frames[:, :, 1]
ax.set_xlim(xs.min() - .5, xs.max() + .5)
ax.set_ylim(ys.min() - .5, ys.max() + .5)

points, = ax.plot([], [], 'o')
lines = [ax.plot([], [])[0] for _ in bones]

def init():
    points.set_data([], [])
    for line in lines:
        line.set_data([], [])
    return [points] + lines

def update(i):
    joints = frames[i]
    x = joints[:, 0]
    y = -joints[:, 1]

    # scatter points
    points.set_data(x, y)

    # draw bones
    for line, (a, b) in zip(lines, bones):
        line.set_data([x[a], x[b]], [y[a], y[b]])

    return [points] + lines

ani = FuncAnimation(fig, update, frames=len(frames), init_func=init, interval=20)
ani.save("data/anim.mp4", writer=FFMpegWriter(fps=30, codec="libx264"))
