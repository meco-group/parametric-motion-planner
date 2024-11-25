import sys
sys.path.append('build/')
sys.path.append('python-benchmark/')
import parametric_motion_planner_module as pmp
from solve_omg_tools import omg_example
import json
import time

def run_omg(local_param, local_env, start, start_vel):
    mp = pmp.MotionPlanner(pmp.PlannerMethod.ARENA, 
                            local_param, 
                            local_env)
    dest = [data_arena['previous_corridor_sequences'][0]['dest']['x'],
            data_arena['previous_corridor_sequences'][0]['dest']['y']]

    mp.SetStart(pmp.Point2Dd(start[0], start[1]))
    mp.SetDest(pmp.Point2Dd(dest[0], dest[1]))
    mp.SetStartVel(pmp.Point2Dd(start_vel[0], start_vel[1]))

    a = time.time()
    mp.UpdateCorridorSequence()
    corridors = mp.GetCorridorSequence()
    vmax = data_arena['motion_planner']['parameters']['v_max']
    amax = data_arena['motion_planner']['parameters']['a_max']
    veh_width = data_arena['motion_planner']['parameters']['veh_width']
    veh_height = data_arena['motion_planner']['parameters']['veh_height']
    nb_runs = 10
    t_solver_acc = 0.0
    first = True
    # print(f"\n\n\n\n\n\n\n\n\n\n\n\n\n\nstart={start}\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
    for i in range(nb_runs):
        tsolver, tf = omg_example(corridors, start, dest, vmax, amax, 
                                    veh_width, veh_height, dump_to_json=True, 
                                    file_name=f"build/output/omgtools_solution_temp.json",
                                    start_vel=start_vel)
        if first:
            b = time.time()
            total_time = (b - a)*1000
            first = False
        t_solver_acc += tsolver
    t_solver = t_solver_acc / nb_runs
    avg_solver_time = t_solver*1000
    with open("build/output/omgtools_solution_temp.json") as f:
        trajectory = json.load(f)

    return total_time, avg_solver_time, trajectory

def run_dynamic_example_omg_tools(data_arena):
    ### prepare dat containers
    data_omg = {'previous_trajectories': []}
    travelled_trajectory = {"t": [], "px": [], "py": []}
    avg_solver_times = []
    total_times = []

    start = [data_arena['previous_corridor_sequences'][0]['start']['x'],
             data_arena['previous_corridor_sequences'][0]['start']['y']]
    start_vel = [data_arena['previous_corridor_sequences'][0]['start_vel']['x'],
                 data_arena['previous_corridor_sequences'][0]['start_vel']['y']]

    ### prepare environments
    local_env = pmp.Environment(
        data_arena['previous_environments'][0]['nb_cell_rows'], 
        data_arena['previous_environments'][0]['nb_cell_cols'],
        data_arena['previous_environments'][0]['cell_width'], 
        data_arena['previous_environments'][0]['cell_height'])
    
    envs = []
    for env_json in data_arena['previous_environments']:
        env = pmp.Environment(env_json['nb_cell_rows'], 
                              env_json['nb_cell_cols'],
                              env_json['cell_width'], 
                              env_json['cell_height'])
        for i in range(env_json["nb_cell_rows"]):
            for j in range(env_json["nb_cell_cols"]):
                if env_json["occupancy_grid"][j][i] != 0:
                    env.AddObstacle(pmp.Point2Di(j, i))

        envs.append(env)

    ### prepare pararmeters
    local_param = pmp.Parameters(data_arena['motion_planner']['parameters']['v_max'], 
                                 data_arena['motion_planner']['parameters']['a_max'],
                                 data_arena['motion_planner']['parameters']['veh_width'], 
                                 data_arena['motion_planner']['parameters']['veh_height'],
                                 data_arena['motion_planner']['parameters']['margin'])
    local_env.CopyObstacles(envs[0])


    ### run first trajectory
    total_time, avg_solver_time, trajectory = run_omg(local_param, local_env,
                                                      start, start_vel)
    total_times.append(total_time)
    avg_solver_times.append(avg_solver_time)
    data_omg['previous_trajectories'].append(trajectory["trajectory"])


    arena_replanning_time_ptr = 0
    for i in range(len(data_arena['previous_trajectories'])-1):
        curr_traj_sample_ptr = 0
        curr_t = 0

        # simulate the trajectory until the replanning time
        time_duration = data_arena['replanning_times'][arena_replanning_time_ptr] - \
                        (data_arena['replanning_times'][arena_replanning_time_ptr-1] if arena_replanning_time_ptr > 0 else 0)
        while curr_t < time_duration:
            travelled_trajectory["t"].append(curr_t)
            travelled_trajectory["px"].append(trajectory['trajectory']["px"][curr_traj_sample_ptr])
            travelled_trajectory["py"].append(trajectory['trajectory']["py"][curr_traj_sample_ptr])

            curr_traj_sample_ptr += 1
            curr_t = trajectory["trajectory"]["t"][curr_traj_sample_ptr]

        arena_replanning_time_ptr += 1
            
        start = [trajectory["trajectory"]["px"][curr_traj_sample_ptr], \
                 trajectory["trajectory"]["py"][curr_traj_sample_ptr]]
        start_vel = [trajectory["trajectory"]["vx"][curr_traj_sample_ptr], \
                     trajectory["trajectory"]["vy"][curr_traj_sample_ptr]]
        # start_vel = [0,0]
        
        local_env.CopyObstacles(envs[i+1])

        total_time, avg_solver_time, trajectory = run_omg(local_param, local_env, start, start_vel)
        total_times.append(total_time)
        avg_solver_times.append(avg_solver_time)
        data_omg['previous_trajectories'].append(trajectory["trajectory"])


    # add all remaining samples to the travelled trajectory
    curr_traj_sample_ptr = 0
    curr_t = 0
    while curr_traj_sample_ptr < len(trajectory["trajectory"]["t"])-1:
        travelled_trajectory["t"].append(trajectory['trajectory']['t'][curr_traj_sample_ptr])
        travelled_trajectory["px"].append(trajectory['trajectory']["px"][curr_traj_sample_ptr])
        travelled_trajectory["py"].append(trajectory['trajectory']["py"][curr_traj_sample_ptr])

        curr_traj_sample_ptr += 1


    print(f"avg_solver_times: {avg_solver_times}")

    data_omg['avg_solver_times'] = avg_solver_times
    data_omg['total_times'] = total_times
    data_omg['travelled_trajectory'] = travelled_trajectory

    # store json
    with open("build/output/dynamic_solution_OMG.json", 'w') as f:
        json.dump(data_omg, f, indent=4)


file_arena = "build/output/dynamic_solution.json"
with open(file_arena) as f:
    data_arena = json.load(f)

run_dynamic_example_omg_tools(data_arena)