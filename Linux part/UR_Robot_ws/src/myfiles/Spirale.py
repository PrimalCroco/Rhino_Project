import numpy as np

# --- Spiral parameters ---
ns = 60  # Rotating speed in rev/min
f_s = 0.54  # mm (Chamfer)
beta = np.sqrt(2 * np.pi * ns / 5)  # Spiral orbit calculated
tmax = 4.7  # Maximum search time in seconds

# Discrete time points every 0.1 s
time_points = np.arange(0, tmax + 0.1, 0.1)
theta_points = beta * np.sqrt(time_points)
r_points = f_s / np.pi * theta_points
x_points = r_points * np.cos(theta_points)
y_points = r_points * np.sin(theta_points)

# Optional: Z coordinate (constant or ramp)
z_start = 0.3  # Starting height
z_end = 0.5    # Ending height
z_points = np.linspace(z_start, z_end, len(x_points))  # Linear increase in z

# Orientation (vertical tool)
roll = -np.pi/2
pitch = 0.0
yaw = 0.0

# Create waypoints list
waypoints = []
for x, y, z in zip(x_points, y_points, z_points):
    waypoints.append((x, y, z, roll, pitch, yaw))

# Print some waypoints
print("Waypoints for ROS MoveGroup:")
for wp in waypoints:  # print first 10 as example
    print(wp,",")

