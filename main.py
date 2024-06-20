import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from math import pi
from Lidar import Lidar


# Function to collect data for a full 360-degree sweep
def collect_full_scan(swipe):
    dist, ang = lidar.get_lidar_scan(swipe)
    return np.array(dist), np.array(ang)

# Function to update the plot with new Lidar data
def update_plot(swipe, ox, oy):
    dist, ang = collect_full_scan(swipe)
    print(f" Angle: {ang}")
    oy = np.append(oy, np.sin(pi * ang / 180) * dist)
    ox = np.append(ox, np.cos(pi * ang / 180) * dist)
    plt.cla()
    plt.scatter(ox, oy, s=2)
    if len(ox) >= 360:
        ox = ox[80:]
        oy = oy[80:]
    plt.grid(True)
    return ox, oy

# Initialize the Lidar sensor
lidar = Lidar()

swipe = 0
oy = np.array([])
ox = np.array([])

# Set up the plot
def animate(frame):
    global swipe
    global ox
    global oy
    ox, oy = update_plot(swipe, ox, oy)
    swipe += 2
    if swipe > 9:
        swipe = 0
try:
    ani = FuncAnimation(plt.gcf(), animate, frames= 100, interval = 100, save_count=360)
    plt.tight_layout()  # Turn off interactive modes
    plt.show()  # Show the final plot
except KeyboardInterrupt:
    pass
plt.close()