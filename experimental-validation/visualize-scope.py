import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import json
import sys
sys.path.append('post-process')
from visualization_helpers import show_environment, show_corridors, plot_vehicle_footprint, \
        set_env_plot_limits

def create_environment_figure(output_file):
    with open(output_file) as f:
        data = json.load(f)

    plt.figure()

    show_environment(data['environment'])
    show_corridors(data['corridor_sequence'], max_alpha=0.5)
    
    start = data['corridor_sequence']['start']
    dest = data['corridor_sequence']['dest']
    w = data['parameters']['veh_width']
    h = data['parameters']['veh_height']
    plot_vehicle_footprint(plt.gca(), start['x'], start['y'], w, h, False)
    plot_vehicle_footprint(plt.gca(), dest['x'], dest['y'], w, h, True)

    plt.xticks([])
    plt.yticks([])

    set_env_plot_limits(data['environment'])

def get_plain_list(df):
    return [float(df[i]) for i in range(len(df)-1)]

def shift_samples(samples, shift_amount=1):
    # return [samples[i-1] for i in range(1, len(samples))] + [samples[-1]]
    return np.array([samples[0]]*shift_amount + [samples[i] for i in range(0, len(samples)-shift_amount)])

def get_data(df, columns_name_to_index, name):
    return np.array(get_plain_list(df.iloc[:, columns_name_to_index[name]]))

def get_variance(time, x, y, z):
    start_time = 0.001
    stop_time = 40000

    # select the correct indices
    # ind = np.asarray(np.logical_and(time > start_time, time < stop_time)).nonzero()
    ind = np.asarray(time < stop_time).nonzero()
    x_rest = x[ind]
    y_rest = y[ind]
    z_rest = z[ind]

    result = {
        "x_mean" : np.mean(x_rest),
        "x_variance" : np.var(x_rest),
        "y_mean" : np.mean(y_rest),
        "y_variance" : np.var(y_rest),
        "z_mean" : np.mean(z_rest),
        "z_variance" : np.var(z_rest),
    }
    return result

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
time = np.array(get_plain_list(df.iloc[:, 0]))  # Time in ms
actual_px = get_data(df, columns_name_to_index, 'actual-px')
true_setpoint_px = get_data(df, columns_name_to_index, 'true-setpoint-px')
actual_py = get_data(df, columns_name_to_index, 'actual-py')
true_setpoint_py = get_data(df, columns_name_to_index, 'true-setpoint-py')
actual_vx = get_data(df, columns_name_to_index, 'actual-vx')
true_setpoint_vx = get_data(df, columns_name_to_index, 'true-setpoint-vx')
actual_vy = get_data(df, columns_name_to_index, 'actual-vy')
true_setpoint_vy = get_data(df, columns_name_to_index, 'true-setpoint-vy')

pz = get_data(df, columns_name_to_index, 'pz')
pa = get_data(df, columns_name_to_index, 'pa')
pb = get_data(df, columns_name_to_index, 'pb')
pc = get_data(df, columns_name_to_index, 'pc')
vz = get_data(df, columns_name_to_index, 'vz')
va = get_data(df, columns_name_to_index, 'va')
vb = get_data(df, columns_name_to_index, 'vb')
vc = get_data(df, columns_name_to_index, 'vc')

true_setpoint_px = shift_samples(true_setpoint_px, 1)
true_setpoint_py = shift_samples(true_setpoint_py, 1)
true_setpoint_vx = shift_samples(true_setpoint_vx, 1)
true_setpoint_vy = shift_samples(true_setpoint_vy, 1)

position_distance = np.sqrt((actual_px - true_setpoint_px)**2 + (actual_py - true_setpoint_py)**2)
total_squared_error = np.sum(position_distance**2)
print(f'Total Squared Error: {total_squared_error:.2f} [mm^2]')
print(f"Max Position Distance Error: {np.max(position_distance):.2f} [mm]")

print(f"statistical data on position level:")
print(get_variance(time, actual_px, actual_py, pz))

print(f"statistical data on velocity level:")
print(get_variance(time, actual_vx, actual_vy, vz))

def plot_actual_vs_setpoint(time, actual, setpoint, name, ylabel):
    if setpoint is None:
        setpoint = 0*actual
    plt.figure(figsize=(10, 5))
    plt.plot(time, actual, label=f'Actual {name}', linestyle='', marker='.')
    plt.plot(time, setpoint, label=f'Setpoint {name}', linestyle='', marker='.')
    plt.xlabel('Time [ms]')
    plt.ylabel(ylabel)
    plt.legend()

plot_actual_vs_setpoint(time, actual_px, true_setpoint_px, 'px', 'x position [mm]')
plot_actual_vs_setpoint(time, actual_py, true_setpoint_py, 'py', 'y position [mm]')
plot_actual_vs_setpoint(time, actual_vx, true_setpoint_vx, 'vx', 'x velocity [mm/s]')
plot_actual_vs_setpoint(time, actual_vy, true_setpoint_vy, 'vy', 'y velocity [mm/s]')
plot_actual_vs_setpoint(time, pz, None, 'pz', 'z position [mm]')
plot_actual_vs_setpoint(time, vz, None, 'vz', 'z velocity [mm/s]')
plot_actual_vs_setpoint(time, pa, None, 'pa', 'a position')
plot_actual_vs_setpoint(time, pb, None, 'pb', 'b position')
plot_actual_vs_setpoint(time, pc, None, 'pc', 'c position')
plot_actual_vs_setpoint(time, va, None, 'va', 'a velocity')
plot_actual_vs_setpoint(time, vb, None, 'vb', 'b velocity')
plot_actual_vs_setpoint(time, vc, None, 'vc', 'c velocity')

from matplotlib.collections import LineCollection
actual_px = 0.001*actual_px
actual_py = 0.001*actual_py
points = np.column_stack([actual_px, actual_py])
segments = np.array([points[:-1], points[1:]]).transpose(1, 0, 2)
lc = LineCollection(segments, cmap='coolwarm', norm=plt.Normalize(position_distance.min(), position_distance.max()), lw=4, zorder=999)
lc.set_array(position_distance[:-1])  # Set color based on position error

create_environment_figure('experimental-validation/record_tracking_error_demo_output.json')
ax = plt.gca()
ax.add_collection(lc)
plt.colorbar(lc, label='Position Distance Error (mm)')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('XY Position Tracking with Distance Error')

plt.show()


