import matplotlib.pyplot as plt


def parse_xy_data(filename):
    """Parse tab-separated X Y data from serial monitor output."""
    x, y = [], []
    with open(filename, "r") as f:
        for line in f:
            parts = line.strip().split("\t")
            if len(parts) == 2:
                try:
                    x.append(float(parts[0]))
                    y.append(float(parts[1]))
                except ValueError:
                    continue
    return x, y


x, y = parse_xy_data("response.txt")

fig, ax = plt.subplots(figsize=(8, 8))

# Plot rover path
ax.plot(x, y, "b-", linewidth=1.5, label="Rover Path")
ax.plot(x[0], y[0], "go", markersize=10, label="Start")
ax.plot(x[-1], y[-1], "rs", markersize=10, label="End")

# Plot ideal square for reference
ideal_x = [0, 20, 20, 0, 0]
ideal_y = [0, 0, 20, 20, 0]
ax.plot(ideal_x, ideal_y, "k--", linewidth=1.5, alpha=0.5, label="Ideal Square")

ax.set_xlabel("X (inches)")
ax.set_ylabel("Y (inches)")
ax.set_title("Rover X-Y Position")
ax.set_aspect("equal")
ax.legend()
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()
