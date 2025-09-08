import numpy as np
import matplotlib.pyplot as plt

# Parameters
ns = 60  # Rotating speed in rev/min
f_s = 0.54  # mm (Chamfer)
beta = np.sqrt(2 * np.pi * ns / 5)  # Spiral orbit calculated
tmax = 4.7  # Maximum search time in seconds
N = 12.84  # Number of orbits (not directly used here)
v_orbit = f_s * beta**2 / (2 * np.pi)  # Constant

# Time arrays
time_interval = 0.01
t = np.arange(0, tmax + time_interval, time_interval)  # Smooth curve time points

# Spiral equations
theta = beta * np.sqrt(t)
r = f_s / np.pi * theta
x = r * np.cos(theta)
y = r * np.sin(theta)

# Plot smooth spiral
plt.figure(figsize=(8, 8))
plt.plot(x, y, 'b-', linewidth=2, label='Smooth Spiral Path')

# Discrete points every 100 ms
time_points = np.arange(0, tmax + 0.1, 0.1)
theta_points = beta * np.sqrt(time_points)
r_points = f_s / np.pi * theta_points
x_points = r_points * np.cos(theta_points)
y_points = r_points * np.sin(theta_points)

# Plot discrete points
plt.scatter(x_points, y_points, color='red', label='100 ms Points', zorder=5)

# Coordinate matrix (each row is [x, y])
spiral_points = np.column_stack((x_points, y_points))

# Labels and formatting
plt.title('Smooth Spiral Path with M4 Screw Parameters (100 ms Discretization)')
plt.xlabel('X (mm)')
plt.ylabel('Y (mm)')
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.show()

# Optional: Print or inspect the points
print("Spiral coordinate points at 100 ms intervals:")
print(spiral_points)
