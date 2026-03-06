#!/usr/bin/env python3
"""Plot x,y coordinates from circle.txt (3rd and 4th columns)."""

import numpy as np
import matplotlib
matplotlib.use("Agg")  # non-interactive backend (saves without opening window)
import matplotlib.pyplot as plt

# Load file: 3rd and 4th columns are x and y (skip rows with fewer columns)
x_list, y_list = [], []
with open("step_response_data.txt") as f:
    for line in f:
        parts = line.split()
        if len(parts) >= 4:
            try:
                x_list.append(float(parts[2]))
                y_list.append(float(parts[3]))
            except ValueError:
                pass

x = np.array(x_list)
y = np.array(y_list)

plt.figure(figsize=(8, 8))
plt.plot(x, y, "b-", linewidth=0.8, alpha=0.9)
plt.xlabel("x")
plt.ylabel("y")
plt.title("Circle data (circle.txt)")
plt.gca().set_aspect("equal")
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig("circle_plot.png", dpi=150)
print("Saved circle_plot.png")
