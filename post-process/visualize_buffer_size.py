import json
import matplotlib.pyplot as plt
import numpy as np

def visualize_buffer_size_over_time(data):
    Tf = data["travelled_trajectory"]["Tf"]
    dt = data["travelled_trajectory"]["dt"]
    sample_durations = data["ms_to_retrieve_sample"]

    sample_durations = [0.001*s for s in sample_durations]
    nb_samples = len(sample_durations)

    # construct a time-grid vector going from 0 to Tf with steps dt/refinment
    refinment = 10
    time_grid = np.arange(-0.5, Tf + 1, dt/refinment)

    # construct a buffer size vector
    buffer_size = np.zeros(len(time_grid))

    # fill the buffer size vector
    number_of_samples_provided_to_mover = 0
    number_of_samples_read_by_mover = 0
    number_of_irrelevant_samples_read = 0
    for i in range(len(time_grid)):
        t = time_grid[i]
        # number_of_samples_read_by_mover = max(0, int(min(t/data["travelled_trajectory"]["dt"],
        #                                                  data["travelled_trajectory"]["nb_samples"])))
        number_of_samples_read_by_mover = max(0, int(t/data["travelled_trajectory"]["dt"]) - 
                                                     number_of_irrelevant_samples_read)

        while (number_of_samples_provided_to_mover < len(data["duration_of_request_since_first_sample_in_ms"]) and
               t + data["mover_started_moving"]/1000 > data["duration_of_request_since_first_sample_in_ms"][number_of_samples_provided_to_mover]/1000):
            number_of_samples_provided_to_mover += 1

        # buffer_size[i] = max(0, number_of_samples_provided_to_mover - number_of_samples_read_by_mover)
        buffer_size[i] = number_of_samples_provided_to_mover - number_of_samples_read_by_mover

        if buffer_size[i] < 0:
            number_of_irrelevant_samples_read += -buffer_size[i] - 1
            buffer_size[i] = 0
    
    # plot the buffer size over time
    plt.figure()
    plt.plot(time_grid, buffer_size)
    plt.xlabel("Time [s]")
    plt.ylabel("Number of samples in buffer")
    plt.ylim([-1, max(buffer_size) + 1])
    plt.gca().grid(axis='y')
    plt.show()

file = "build/output/dynamic_solution_movable_destination_sampler.json"
with open(file) as f:
    data = json.load(f)

visualize_buffer_size_over_time(data)
        