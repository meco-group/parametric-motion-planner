import json
import matplotlib.pyplot as plt
import numpy as np

def visualize_buffer_size_over_time(data):
    Tf = data["travelled_trajectory"]["Tf"]
    dt = data["travelled_trajectory"]["dt"]
    sample_durations = data["ms_to_retrieve_sample"]
    print(sample_durations)

    # make a histogram of the sample durations (bin width 1)
    plt.figure()
    plt.hist(sample_durations, bins=range(0, int(max(sample_durations))+1), edgecolor='black')

    sample_durations = [0.001*s for s in sample_durations]
    nb_samples = len(sample_durations)

    # construct a time-grid vector going from 0 to Tf with steps dt/refinment
    refinment = 10
    time_grid = np.arange(0, Tf, dt/refinment)

    # construct a buffer size vector
    buffer_size = np.zeros(len(time_grid))

    # fill the buffer size vector
    time_since_last_provided_sample = 0.0
    time_since_last_read_sample = 0.0
    curr_nb_samples_in_buffer = 0
    prev_time = time_grid[0]
    sample_duration_ptr = 0
    for i in range(1, len(time_grid)):
        curr_time = time_grid[i]
        dt = curr_time - prev_time
        
        # check if multiple of 0.010s has been reached
        time_since_last_provided_sample += dt
        time_since_last_read_sample += dt

        if time_since_last_read_sample >= 0.010:
            time_since_last_read_sample -= 0.010
            curr_nb_samples_in_buffer -= 1

        while sample_duration_ptr < len(sample_durations) and \
                time_since_last_provided_sample >= sample_durations[sample_duration_ptr]:
            time_since_last_provided_sample -= sample_durations[sample_duration_ptr]
            curr_nb_samples_in_buffer += 1
            sample_duration_ptr += 1

        buffer_size[i] = curr_nb_samples_in_buffer

        prev_time = curr_time
    
    # plot the buffer size over time
    plt.figure()
    plt.plot(time_grid, buffer_size)
    plt.xlabel("Time [s]")
    plt.ylabel("Number of samples in buffer")
    plt.show()

file = "build/output/dynamic_solution_movable_destination_sampler.json"
with open(file) as f:
    data = json.load(f)

visualize_buffer_size_over_time(data)
        