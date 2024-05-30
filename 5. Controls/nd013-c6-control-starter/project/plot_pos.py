import pandas as pd
import matplotlib.pyplot as plt

# Load the data
data = pd.read_csv('waypoint_log.txt', delimiter=',', header=0)
data.columns = data.columns.str.strip()

# Plot the trajectory of waypoints and the car
plt.figure(figsize=(10, 5))
plt.subplot(1, 2, 1)
plt.plot(data['WP_X'], data['WP_Y'], label='Waypoint Trajectory')
plt.plot(data['Car_X'], data['Car_Y'], label='Car Trajectory', linestyle='--')
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Trajectory Comparison')
plt.legend()

# Plot the velocity over time
plt.subplot(1, 2, 2)
plt.plot(data['Time'], data['Velocity'], label='Velocity', color='r')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('Car Velocity Over Time')
plt.legend()

plt.tight_layout()
plt.savefig('plot-trajectory.png')  # Saves the plot as a PNG file
