import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# read the 7th row of the csv file
df_column_names = pd.read_csv('experimental-validation/Scope-Project.csv', skiprows = lambda x : x != 6)
print(df_column_names)
columns_name_to_index = {}
for i in range(len(df_column_names.columns)):
    column_name = df_column_names.columns[i]
    if "Name" != column_name:
        columns_name_to_index[column_name] = i

print(columns_name_to_index)
# Load data, skipping metadata rows
df = pd.read_csv('experimental-validation/Scope-Project.csv', skiprows=24)

# Extract relevant columns
time = df.iloc[:, 0]  # Time in ms
actual_px = df.iloc[:, columns_name_to_index['actual-px']]
setpoint_px = df.iloc[:, columns_name_to_index['setpoint-px']]
true_setpoint_px = df.iloc[:, columns_name_to_index['true-setpoint-px']]
actual_py = df.iloc[:, columns_name_to_index['actual-py']]
setpoint_py = df.iloc[:, columns_name_to_index['setpoint-py']]
true_setpoint_py = df.iloc[:, columns_name_to_index['true-setpoint-py']]
actual_vx = df.iloc[:, columns_name_to_index['actual-vx']]
true_setpoint_vx = df.iloc[:, columns_name_to_index['setpoint-vx']]
actual_vy = df.iloc[:, columns_name_to_index['actual-vy']]
true_setpoint_vy = df.iloc[:, columns_name_to_index['setpoint-vy']]

def get_plain_list(df):
    return [float(df[i]) for i in range(len(df)-1)]

# convert extracted columns to plain arrays (with only values)
time = get_plain_list(time)
actual_px = get_plain_list(actual_px)
true_setpoint_px = get_plain_list(true_setpoint_px)
actual_py = get_plain_list(actual_py)
true_setpoint_py = get_plain_list(true_setpoint_py)
actual_vx = get_plain_list(actual_vx)
true_setpoint_vx = get_plain_list(true_setpoint_vx)
actual_vy = get_plain_list(actual_vy)
true_setpoint_vy = get_plain_list(true_setpoint_vy)

def shift_samples(samples, shift_amount=1):
    # return [samples[i-1] for i in range(1, len(samples))] + [samples[-1]]
    return [samples[0]]*shift_amount + [samples[i] for i in range(0, len(samples)-shift_amount)]

true_setpoint_px = shift_samples(true_setpoint_px, 1)
true_setpoint_py = shift_samples(true_setpoint_py, 1)
true_setpoint_vx = shift_samples(true_setpoint_vx, 1)
true_setpoint_vy = shift_samples(true_setpoint_vy, 1)

# convert lists to np arrays
time = np.array(time)
actual_px = np.array(actual_px)
true_setpoint_px = np.array(true_setpoint_px)
actual_py = np.array(actual_py)
true_setpoint_py = np.array(true_setpoint_py)
actual_vx = np.array(actual_vx)
true_setpoint_vx = np.array(true_setpoint_vx)
actual_vy = np.array(actual_vy)
true_setpoint_vy = np.array(true_setpoint_vy)

position_distance = np.sqrt((actual_px - true_setpoint_px)**2 + (actual_py - true_setpoint_py)**2)
total_squared_error = np.sum(position_distance**2)
print(f'Total Squared Error: {total_squared_error:.2f} [mm^2]')
print(f"Max Position Distance Error: {np.max(position_distance):.2f} [mm]")

# Plot X Position Tracking
plt.figure(figsize=(10, 5))
plt.plot(time, actual_px, label='Actual X Position', linestyle='', marker='.')
# plt.plot(time, setpoint_px, label='Setpoint X Position', linestyle='--', marker='')
plt.plot(time, true_setpoint_px, label='True Setpoint X', linestyle='', marker='.')
plt.xlabel('Time (ms)')
plt.ylabel('X Position')
plt.title('X Position Tracking')
plt.legend()
# plt.grid()
# plt.show()

# Plot Y Position Tracking
plt.figure(figsize=(10, 5))
plt.plot(time, actual_py, label='Actual Y Position', linestyle='', marker='.')
# plt.plot(time, setpoint_py, label='Setpoint Y Position', linestyle='--', marker='')
plt.plot(time, true_setpoint_py, label='True Setpoint Y', linestyle='', marker='.')
plt.xlabel('Time (ms)')
plt.ylabel('Y Position')
plt.title('Y Position Tracking')
plt.legend()
# plt.grid()
# plt.show()

# Plot X Velocity Tracking
plt.figure(figsize=(10, 5))
plt.plot(time, actual_vx, label='Actual X Velocity', linestyle='', marker='.')
# plt.plot(time, setpoint_vx, label='Setpoint X Velocity', linestyle='--', marker='')
plt.plot(time, true_setpoint_vx, label='True Setpoint X Velocity', linestyle='', marker='.')
plt.xlabel('Time (ms)')
plt.ylabel('X Velocity')
plt.title('X Velocity Tracking')
plt.legend()
# plt.grid()
# plt.show()

# Plot Y Velocity Tracking
plt.figure(figsize=(10, 5))
plt.plot(time, actual_vy, label='Actual Y Velocity', linestyle='', marker='.')
# plt.plot(time, setpoint_vy, label='Setpoint Y Velocity', linestyle='--', marker='')
plt.plot(time, true_setpoint_vy, label='True Setpoint Y Velocity', linestyle='', marker='.')
plt.xlabel('Time (ms)')
plt.ylabel('Y Velocity')
plt.title('Y Velocity Tracking')
plt.legend()
# plt.grid()
# plt.show()

# Plot XY position
plt.figure(figsize=(10, 5))
plt.plot(actual_px, actual_py, label='Actual Position', linestyle='', marker='.')
plt.plot(true_setpoint_px, true_setpoint_py, label='True Setpoint Position', linestyle='', marker='.')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('XY Position Tracking')
plt.legend()
plt.axis('equal')
# plt.grid()
# plt.show()

# Plot Position Distance error
plt.figure(figsize=(10, 5))
plt.plot(time, position_distance, label='Position Distance Error', linestyle='', marker='.')
plt.xlabel('Time (ms)')
plt.ylabel('Position Distance Error (mm)')
plt.title('Position Distance Error (mm)')
plt.legend()
# plt.grid()
# plt.show()

# Plot XY measured position samples where the color of each sample shows the distance error
plt.figure(figsize=(10, 5))
plt.scatter(actual_px, actual_py, c=position_distance, cmap='coolwarm')
plt.colorbar(label='Position Distance Error (mm)')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('XY Position Tracking with Distance Error')
plt.axis('equal')

from matplotlib.collections import LineCollection
points = np.column_stack([actual_px, actual_py])
segments = np.array([points[:-1], points[1:]]).transpose(1, 0, 2)
lc = LineCollection(segments, cmap='coolwarm', norm=plt.Normalize(position_distance.min(), position_distance.max()), lw=4)
lc.set_array(position_distance[:-1])  # Set color based on position error
plt.figure(figsize=(10, 5))
ax = plt.gca()
ax.add_collection(lc)
plt.colorbar(lc, label='Position Distance Error (mm)')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('XY Position Tracking with Distance Error')
plt.axis('equal')



plt.show()


