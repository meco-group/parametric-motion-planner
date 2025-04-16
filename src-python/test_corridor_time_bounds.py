import json
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle

def optimize_arc_1d(p0, pf, v0, v_max, a_max):
    pf_rel = abs(pf - p0)

    t_sol_vector = [0,0,0]

    if pf_rel == 0:
        t_sol_vector[0] = 0
        t_sol_vector[1] = 0
        t_sol_vector[2] = 0
        return 0

    if pf - p0 < 0:
        v0 = -v0

    T = 0
    ts1 = 0
    ts2 = 0
    ts3 = 0

    if v0 >= v_max + 1.0e-7:
        t1 = (v0 - v_max) / a_max
        t2 = (1.0 / v_max) * (pf_rel - (1.0 / a_max) * (2 * v0 * v_max - 0.5 * v0**2 - v_max))
        t3 = v_max / a_max
        T = t1 + t2 + t3

        ts1 = t1
        ts2 = t1 + t2

    else:
        V1 = np.sqrt(2 * a_max * pf_rel)

        if v0 >= V1:
            T = (v0 + np.sqrt(max(0.0, 2 * v0**2 - 4 * a_max * pf_rel))) / a_max
            ts = 0.5 * (T + v0 / a_max)
        else:
            T = (-v0 + np.sqrt(max(0.0, 2 * v0**2 + 4 * a_max * pf_rel))) / a_max
            ts = 0.5 * (T - v0 / a_max)

        if a_max * (T - ts) > v_max:
            delta_t = T - ts - v_max / a_max
            ts1 = ts - delta_t
            ts2 = ts + a_max / v_max * delta_t**2 + delta_t
            T = ts2 + v_max / a_max
        else:
            ts1 = ts
            ts2 = ts

    t_sol_vector[0] = ts1
    t_sol_vector[1] = ts2 - ts1
    t_sol_vector[2] = T - ts2

    return sum(t_sol_vector)

def sample_const_accel(p0, v0, a, t0, tf, n=1000):
    t_sol = np.linspace(t0, tf, n)
    p_sol = p0 + v0 * t_sol + 0.5 * a * t_sol**2
    v_sol = v0 + a * t_sol
    return p_sol, v_sol, t_sol

def solve_BVP_1D_min_time(p0, pf, v0, vf, v_max, a_max):
    pf_rel = abs(pf - p0)
    
    if pf - p0 < 0:
        v0 = -v0
        vf = -vf

    # assume -v_max <= v0 <= v_max
    #   and  -v_max <= vf <= v_max

    V1 = np.sqrt(2 * a_max * (pf_rel + np.abs(vf)**2/(2*a_max)))
    V2 = np.sqrt(2 * a_max * (pf_rel + np.abs(v0)**2/(2*a_max)))
    if v0 >= V1:
        # we will overshoot the target
        # print("case overshoot")
        phat = (v0**2 - vf**2) / (4 * a_max) + pf_rel/2
        temp = np.sqrt(2*(v0**2 + vf**2 - 2*a_max*pf_rel))
        vhat = -temp/2
        t1 = v0/a_max + temp/(2*a_max)
        t2 = 0
        t3 = (2*vf + temp)/(2*a_max)
        
        a1 = -a_max
        a2 = a_max

        if vhat < -v_max:
            delta_t = -(vhat + v_max) / a_max
            t1 = t1 - delta_t
            vhat = v_max
            phat = v0 * t1 - 0.5 * a_max * t1**2
            t3 = t3 - delta_t
            t2 = np.abs(pf_rel - vf*t3 + a_max*t3**2/2 - phat)/np.abs(vhat)
    elif vf >= V2:
        # we will undershoot the starting position
        # print("case undershoot")
        phat = (v0**2 - vf**2) / (4 * a_max) + pf_rel/2
        temp = np.sqrt(2*(v0**2 + vf**2 - 2*a_max*pf_rel))
        vhat = -temp/2
        t1 = v0/a_max + temp/(2*a_max)
        t2 = 0
        t3 = (2*vf + temp)/(2*a_max)
        
        a1 = -a_max
        a2 = a_max

        if vhat < -v_max:
            delta_t = -(vhat + v_max) / a_max
            t1 = t1 - delta_t
            vhat = v_max
            phat = v0 * t1 - 0.5 * a_max * t1**2
            t3 = t3 - delta_t
            t2 = np.abs(pf_rel - vf*t3 + a_max*t3**2/2 - phat)/np.abs(vhat)

    else:
        # we do not overshoot or undershoot
        # print("case basic")
        phat = (vf**2 - v0**2) / (4 * a_max) + pf_rel/2
        temp = np.sqrt(2*(v0**2 + vf**2 + 2*a_max*pf_rel))
        vhat = temp/2
        t1 = -v0/a_max + temp/(2*a_max)
        t2 = 0
        t3 = (-2*vf + temp)/(2*a_max)
       
        a1 = a_max
        a2 = -a_max

        if vhat > v_max:
            delta_t = (vhat - v_max) / a_max
            t1 = t1 - delta_t
            vhat = v_max
            phat = v0 * t1 + 0.5 * a_max * t1**2
            t3 = t3 - delta_t
            t2 = np.abs(pf_rel - vf*t3 - a_max*t3**2/2 - phat)/np.abs(vhat)

    if pf < p0:
        a1 = -a1
        a2 = -a2
        v0 = -v0
        vf = -vf
        vhat = -vhat
        phat = -phat
    phat = p0 + phat
        
    return {
        'T': t1 + t2 + t3,
        'tt': [t1, t2, t3],
        'aa': [a1, 0, a2],
        'phat': phat,
        'vhat': vhat,
        'p0': p0,
        'pf': pf,
        'v0': v0,
        'vf': vf,
        'v_max': v_max,
        'a_max': a_max,
    }

def solve_BVP_1D_fixed_time(p0, pf, v0, vf, T, v_max, a_max):
    # solve minimum-time problem
    result_min_time = solve_BVP_1D_min_time(p0, pf, v0, vf, v_max, a_max)

    if result_min_time['T'] > T:
        raise ValueError("Infeasible fixed-time problem: T < T*")

    # solve fixed-time problem
    # if we reach zero velocity, insert a waiting time there
    if np.sign(v0) != np.sign(result_min_time['vhat']):
        # insert waiting time in first arc
        t1 = -v0/result_min_time['aa'][0]
        t2 = T - result_min_time['T']
        t3 = result_min_time['tt'][0] - t1

        return {
            'T': T,
            'tt': [t1, t2, t3] + result_min_time['tt'][1:],
            'aa': [result_min_time['aa'][0], 0, result_min_time['aa'][0]] + result_min_time['aa'][1:],
            'phat': result_min_time['phat'],
            'vhat': result_min_time['vhat'],
            'p0': p0,
            'pf': pf,
            'v0': v0,
            'vf': vf,
            'v_max': v_max,
            'a_max': a_max,
        }
    
    if np.sign(vf) != np.sign(result_min_time['vhat']):
        # insert waiting time in last arc
        t3 = np.abs(-vf/result_min_time['aa'][-1])
        assert t3 >= 0
        t2 = T - result_min_time['T']
        t1 = result_min_time['tt'][-1] - t3

        return {
            'T': T,
            'tt': result_min_time['tt'][:-1] + [t1, t2, t3],
            'aa': result_min_time['aa'][:-1] + [result_min_time['aa'][-1], 0, result_min_time['aa'][-1]],
            'phat': result_min_time['phat'],
            'vhat': result_min_time['vhat'],
            'p0': p0,
            'pf': pf,
            'v0': v0,
            'vf': vf,
            'v_max': v_max,
            'a_max': a_max,
        }
    
    # decrease a_max
    a_max_lower = 0
    a_max_upper = result_min_time['a_max']
    a_values = []
    T_values = []
    while np.abs(result_min_time['T'] - T) > 1.0e-6 and a_max_upper - a_max_lower > 1.0e-10:
        # print(f"refining value of a_max: {a_max_lower:.3f} - {a_max_upper:.3f} ({result_min_time['T'] - T:.4f})")
        a_max = (a_max_lower + a_max_upper) / 2
        result_min_time = solve_BVP_1D_min_time(p0, pf, v0, vf, v_max, a_max)
        a_values.append(a_max)
        T_values.append(result_min_time['T'])
        if result_min_time['T'] > T:
            a_max_lower = a_max
        else:
            a_max_upper = a_max

    if np.abs(result_min_time['T'] - T) > 1.0e-6:
        a_max = a_max_lower
        result_min_time = solve_BVP_1D_min_time(p0, pf, v0, vf, v_max, a_max)

    # # create a scatter plot where color shows index in the list
    # plt.scatter(a_values, T_values, c=range(len(a_values)), cmap='viridis')

    return result_min_time

def get_intermediate_points(traj):
    pp_intermediate = [traj['p0']]
    vv_intermediate = [traj['v0']]
    pp_traj = []; vv_traj = []; tt_traj = []
    for k in range(len(traj['tt'])):
        p_curr = pp_intermediate[-1]
        v_curr = vv_intermediate[-1]
        t_curr = sum(traj['tt'][:k])
        pp, vv, tt = sample_const_accel(p_curr, v_curr, traj['aa'][k], 0, traj['tt'][k])
        pp_traj += list(pp)
        vv_traj += list(vv)
        for t in tt:
            tt_traj.append(t + t_curr)
        pp_intermediate.append(pp[-1])
        vv_intermediate.append(vv[-1])
    
    return pp_traj, vv_traj, tt_traj


def visualize_traj(traj, PHASE_PLOT=True, TIME_PLOT=True, BLOCK=True):
    # get intermediate points
    pp_traj, vv_traj, tt_traj = get_intermediate_points(traj)
    
    if PHASE_PLOT:
        plt.figure()

        plt.axvline(0, color='k')
        plt.axhline(0, color='k')
        plt.axhline(-traj['v_max'], color='k', linestyle='--')
        plt.axhline(traj['v_max'], color='k', linestyle='--')

        plt.plot(pp_traj, vv_traj, 'b')

        plt.plot([traj['p0']], [traj['v0']], 'ro')
        plt.plot([traj['pf']], [traj['vf']], 'ro', markerfacecolor='none')
        plt.xlim([-25, 25])
        plt.ylim([-15, 15])
        
        # set equal data aspect
        plt.gca().set_aspect('equal', adjustable='box')

    if TIME_PLOT:
        plt.figure()
        plt.plot(tt_traj, pp_traj, 'b', label='position')
        plt.plot(tt_traj, vv_traj, 'r', label='velocity')
        plt.legend()

    if PHASE_PLOT or TIME_PLOT:
        plt.show(block=BLOCK)

def visualize_traj_2D(traj_2d, TIME_PLOT=True, SHOW=True, fig=None):
    # get intermediate points
    pp_traj_x, vv_traj_x, tt_traj_x = get_intermediate_points(traj_2d['x'])
    pp_traj_y, vv_traj_y, tt_traj_y = get_intermediate_points(traj_2d['y'])

    tt = np.linspace(0, traj_2d['x']['T'], 500)
    px = np.interp(tt, tt_traj_x, pp_traj_x)
    py = np.interp(tt, tt_traj_y, pp_traj_y)
    vx = np.interp(tt, tt_traj_x, vv_traj_x)
    vy = np.interp(tt, tt_traj_y, vv_traj_y)

    if fig is None:
        plt.figure()
    else:
        plt.figure(fig.number)
    plt.plot(px, py, 'b')
    plt.plot(traj_2d['x']['p0'], traj_2d['y']['p0'], 'ro')
    plt.plot(traj_2d['x']['pf'], traj_2d['y']['pf'], 'ro', markerfacecolor='none')
    plt.quiver(traj_2d['x']['p0'], traj_2d['y']['p0'],
                traj_2d['x']['v0'], traj_2d['y']['v0'],
                angles='xy', scale_units='xy', scale=1, color='r')
    plt.quiver(traj_2d['x']['pf'], traj_2d['y']['pf'],
                traj_2d['x']['vf'], traj_2d['y']['vf'],
                angles='xy', scale_units='xy', scale=1, color='r')
    plt.gca().set_aspect('equal', adjustable='box')

    if TIME_PLOT:
        _, axs = plt.subplots(2,1)
        axs[0].plot(tt, px, 'b', label='x position')
        axs[0].plot(tt, py, 'r', label='y position')
        axs[0].legend()
        axs[1].plot(tt, vx, 'b', label='x velocity')
        axs[1].plot(tt, vy, 'r', label='y velocity')
        axs[1].legend()
        axs[1].set_xlabel('time (s)')
    
    if SHOW:
        plt.show()

def solve_BVP_2D(p0x, poy, pfx,pfy,  v0x, v0y, vfx, vfy, v_max, a_max):
    # compute the time-optimal trajectory in 2D, adhering to a_max and v_max
    # moving from p0 with velocity v0 to pf with velocity vf
    traj_x = solve_BVP_1D_min_time(p0x, pfx, v0x, vfx, v_max, a_max)
    traj_y = solve_BVP_1D_min_time(poy, pfy, v0y, vfy, v_max, a_max)

    ready = np.abs(traj_x['T'] - traj_y['T']) < 1.0e-6
    while not ready:
        # print(f"Trying to solve BVP_2D with fixed time {traj_x['T']:.3f} - {traj_y['T']:.3f}")
        if traj_x['T'] > traj_y['T']:
            traj_y = solve_BVP_1D_fixed_time(poy, pfy, v0y, vfy, traj_x['T'], v_max, a_max)
        else:
            traj_x = solve_BVP_1D_fixed_time(p0x, pfx, v0x, vfx, traj_y['T'], v_max, a_max)
        ready = np.abs(traj_x['T'] - traj_y['T']) < 1.0e-6
    # print("We did it!\n")
    return {'x': traj_x, 'y': traj_y}

def test_solve_BVP_2D():
    def get_p0x(t):
        return -4*np.sin(0.5*t)
    
    def get_v0x(t):
        return np.sin(t)
    
    def get_pfx(t):
        return 5 - 2*np.exp(-t)
    
    def get_vfx(t):
        return 2*np.sin(2*t)
    
    def get_p0y(t):
        return -1 - 3*np.exp(-(t-1.5)**2)
    
    def get_v0y(t):
        return 1
    
    def get_pfy(t):
        return 4 + 0.1*(t-3)**3
    
    def get_vfy(t):
        return 2*np.cos(2*t)

    v_max = 2
    a_max = 5

    from matplotlib import animation

    my_fig = plt.figure()

    def update(t):
        my_fig.clf()
        p0x = get_p0x(t)
        p0y = get_p0y(t)
        pfx = get_pfx(t)
        pfy = get_pfy(t)
        v0x = get_v0x(t)
        v0y = get_v0y(t)
        vfx = get_vfx(t)
        vfy = get_vfy(t)

        traj_2d = solve_BVP_2D(p0x, p0y, pfx, pfy, v0x, v0y, vfx, vfy, v_max, a_max)
        visualize_traj_2D(traj_2d, TIME_PLOT=False, SHOW=False, fig=my_fig)
        plt.xlim([-10, 10])
        plt.ylim([-10, 10])
        plt.title(f"T = {traj_2d['x']['T']:.3f} s")

    anim = animation.FuncAnimation(my_fig, update, frames=np.linspace(0, 5, 100), interval=10)
    plt.show()

def point_inside_corridor(px, py, corridor):
    return corridor['x_min'] <= px and \
           corridor['x_max'] >= px and \
           corridor['y_min'] <= py and \
           corridor['y_max'] >= py

def get_overlapping_corridor(c1, c2):
    return {"x_min": max(c1['x_min'], c2['x_min']),
            "x_max": min(c1['x_max'], c2['x_max']),
            "y_min": max(c1['y_min'], c2['y_min']),
            "y_max": min(c1['y_max'], c2['y_max'])}

def get_actual_corridor_time(data):
    corridor_times = []
    nb_corridors = data['parametrization']['nb_corridors']
    tx_sol = data['parametrization']['t_x_sol']
    ty_sol = data['parametrization']['t_y_sol']

    if sum(tx_sol[1]) == 0 and nb_corridors > 1:
        # single arc case
        prev_time = 0
        curr_sample_idx = 0
        curr_corridor_idx = 0
        while curr_corridor_idx < nb_corridors-1:
            next_corridor = data['corridor_sequence']['sequence'][curr_corridor_idx+1]
            px = data['trajectory']['px'][curr_sample_idx]
            py = data['trajectory']['py'][curr_sample_idx]
            while not point_inside_corridor(px, py, next_corridor):
                curr_sample_idx += 1
                px = data['trajectory']['px'][curr_sample_idx]
                py = data['trajectory']['py'][curr_sample_idx]
            
            corridor_times.append(data['trajectory']['t'][curr_sample_idx] - prev_time)
            prev_time = data['trajectory']['t'][curr_sample_idx]
            curr_corridor_idx += 1
        
        corridor_times.append(data['trajectory']['t'][-1] - prev_time)
    else:
        for k in range(nb_corridors):
            corridor_times.append(max(sum(tx_sol[k]), sum(ty_sol[k])))

    for j in range(len(corridor_times)):
        if corridor_times[j] == 0:
            print("Warning: corridor time is 0 for corridor", j)

    return corridor_times

def get_minimum_travel_distance_from_corridor_to_corridor(c1, c2):
    x_overlap = check_overlapping_intervals([c1['x_min'], c1['x_max']], [c2['x_min'], c2['x_max']])
    y_overlap = check_overlapping_intervals([c1['y_min'], c1['y_max']], [c2['y_min'], c2['y_max']])
    x_sign = -1 if x_overlap else 1
    y_sign = -1 if y_overlap else 1
    return max(
        x_sign*min(np.abs(c1['x_max'] - c2['x_max']),
            np.abs(c1['x_max'] - c2['x_min']),
            np.abs(c1['x_min'] - c2['x_min']),
            np.abs(c1['x_min'] - c2['x_max'])) + data['parameters']['veh_width'],
        y_sign*min(np.abs(c1['y_max'] - c2['y_max']),
            np.abs(c1['y_max'] - c2['y_min']),
            np.abs(c1['y_min'] - c2['y_min']),
            np.abs(c1['y_min'] - c2['y_max'])) + data['parameters']['veh_height']
    )

def get_maximum_travel_distance_from_corridor_to_corridor(c1, c2):
    return max(
        max(np.abs(c1['x_max'] - c2['x_max']),
            np.abs(c1['x_max'] - c2['x_min']),
            np.abs(c1['x_min'] - c2['x_min']),
            np.abs(c1['x_min'] - c2['x_max'])) - data['parameters']['veh_width'],
        max(np.abs(c1['y_max'] - c2['y_max']),
            np.abs(c1['y_max'] - c2['y_min']),
            np.abs(c1['y_min'] - c2['y_min']),
            np.abs(c1['y_min'] - c2['y_max'])) - data['parameters']['veh_height']
    )

def get_minimum_travel_distance_from_corridor_to_point(c, p):
    x_overlap = check_overlapping_intervals([c['x_min'], c['x_max']], [p['x'], p['x']])
    y_overlap = check_overlapping_intervals([c['y_min'], c['y_max']], [p['y'], p['y']])
    x_sign = -1 if x_overlap else 1
    y_sign = -1 if y_overlap else 1
    return max(
        x_sign*min(np.abs(c['x_max'] - p['x']),
                   np.abs(c['x_min'] - p['x'])) + data['parameters']['veh_width']/2,
        y_sign*min(np.abs(c['y_max'] - p['y']),
                   np.abs(c['y_min'] - p['y'])) + data['parameters']['veh_height']/2
    )

def get_maximum_travel_distance_from_corridor_to_point(c, p):
    return max(
        max(np.abs(c['x_max'] - p['x']),
            np.abs(c['x_min'] - p['x'])) - data['parameters']['veh_width']/2,
        max(np.abs(c['y_max'] - p['y']),
            np.abs(c['y_min'] - p['y'])) - data['parameters']['veh_height']/2
    )

def get_lower_bounds(data):
    nb_corridors = data['corridor_sequence']['nb_of_corridors']
    start = data['corridor_sequence']['start']
    dest = data['corridor_sequence']['dest']
    corridors = data['corridor_sequence']['sequence']
    if nb_corridors == 1:
        distances = [max(np.abs(start['x'] - dest['x']), 
                         np.abs(start['y'] - dest['y']))]
    else:
        distances = \
            [get_minimum_travel_distance_from_corridor_to_point(corridors[1], start)] + \
            \
            [get_minimum_travel_distance_from_corridor_to_corridor(corridors[k-1], corridors[k+1])
            for k in range(1, nb_corridors-1)] + \
            \
            [get_minimum_travel_distance_from_corridor_to_point(corridors[nb_corridors-2], dest)]
        
    distances = [max(d, 0) for d in distances]
    return [distances[k] / data['parameters']['v_max'] 
            for k in range(nb_corridors)]

def get_upper_bounds(data):
    nb_corridors = data['corridor_sequence']['nb_of_corridors']
    start = data['corridor_sequence']['start']
    dest = data['corridor_sequence']['dest']
    corridors = data['corridor_sequence']['sequence']
    if nb_corridors == 1:
        distances = [max(np.abs(start['x'] - dest['x']), 
                         np.abs(start['y'] - dest['y']))]
    else:
        print(f"start: {start}")
        print(f"overlapping corridor: {get_overlapping_corridor(corridors[0], corridors[1])}")
        print(f"first distance: {get_maximum_travel_distance_from_corridor_to_point(
                get_overlapping_corridor(corridors[0], corridors[1]), start)}")
        print(f"time in first corridor: {sum(data['parametrization']['t_x_sol'][0])}")
        print(f"total trajectory time: {data['trajectory']['t'][-1]}")
        print(f"parameters: {data['parameters']}")
        distances = \
            [get_maximum_travel_distance_from_corridor_to_point(
                get_overlapping_corridor(corridors[0], corridors[1]), start)] + \
            \
            [get_maximum_travel_distance_from_corridor_to_corridor(
                get_overlapping_corridor(corridors[k-1], corridors[k]),
                get_overlapping_corridor(corridors[k+1], corridors[k]))
            for k in range(1, nb_corridors-1)] + \
            \
            [get_maximum_travel_distance_from_corridor_to_point(
                get_overlapping_corridor(corridors[nb_corridors-2],
                                         corridors[nb_corridors-1]), dest)]
        
    distances = [max(d, 0) for d in distances]
    return [optimize_arc_1d(0, distances[k], 0, 
                            data['parameters']['v_max'],
                            data['parameters']['a_max'])
            for k in range(nb_corridors)]

def get_corridor_times(data):
    # extract the time spent in every corridor
    corridor_times = get_actual_corridor_time(data)
    assert len(corridor_times) == data['parametrization']['nb_corridors']

    # get the lower bounds
    lower_bounds = get_lower_bounds(data)

    # get the upper bounds
    upper_bounds = get_upper_bounds(data)

    return np.array(corridor_times), np.array(lower_bounds), np.array(upper_bounds)

def check_overlapping_intervals(range1, range2, tolerance=0):
    return (range1[1] >= range2[0] + tolerance and range1[0] <= range2[0] - tolerance or \
            range1[1] >= range2[1] + tolerance and range1[0] <= range2[1] - tolerance or \
            range1[0] >= range2[0] + tolerance and range1[1] <= range2[1] - tolerance) \

def check_overlap(c1, c2, tolerance=1.0e-4):
    return check_overlapping_intervals([c1["x_min"], c1["x_max"]], [c2["x_min"], c2["x_max"]], tolerance=tolerance) and \
           check_overlapping_intervals([c1["y_min"], c1["y_max"]], [c2["y_min"], c2["y_max"]], tolerance=tolerance)

def get_overlapping_corridors(seq1, seq2):
    overlapping_pairs = []
    for i, c1 in enumerate(seq1):
        for j, c2 in enumerate(seq2):
            if check_overlap(c1, c2):
                overlapping_pairs.append([i, j])
    return overlapping_pairs

def process_data(data_files):
    corridor_times = []
    lower_bounds = []
    upper_bounds = []

    my_d = 412
    for d, data in enumerate(data_files):
        if d == my_d:
            print(f"Data file {my_d}!")
            print(f"vmax: {data['parameters']['v_max']}")
            print(f"amax: {data['parameters']['a_max']}")
            print(f"tx_sol (0): {data['parametrization']['t_x_sol'][0]}")
            print(f"ty_sol (0): {data['parametrization']['t_y_sol'][0]}")
            print(f"tx_sol (1): {data['parametrization']['t_x_sol'][1]}")
            print(f"ty_sol (1): {data['parametrization']['t_y_sol'][1]}")
            print(f"waypoint (1): {data['parametrization']['waypoints'][1]}")
        # if d > my_d:
        #     exit()
            
        ct, lb, ub = get_corridor_times(data)
        corridor_times.append(ct)
        lower_bounds.append(lb)
        upper_bounds.append(ub)

        for i in range(len(ct)):
            if lb[i] > ct[i] + 1.0e-4:
                print(f"Lower bound {lb[i]} is greater than corridor time {ct[i]} for corridor {i} in data file {d}")

            if ub[i] < ct[i] - 1.0e-4:
                print(f"Upper bound {ub[i]} is smaller than corridor time {ct[i]} for corridor {i} in data file {d}")

    return corridor_times, lower_bounds, upper_bounds

def visualize(corridor_times, lower_bounds, upper_bounds, data_files):
    # plot the results
    fig, ax = plt.subplots()
    bar_width = 0.5/len(corridor_times)
    for i in range(len(corridor_times)):
        yerr = np.array([np.array(corridor_times[i]) - np.array(lower_bounds[i]),
                        np.array(upper_bounds[i]) - np.array(corridor_times[i])])
        for j in range(len(corridor_times[i])):
            yerr[0][j] = max(0, yerr[0][j])
            yerr[1][j] = max(0, yerr[1][j])

        ax.bar(np.array(range(len(corridor_times[i])))-bar_width*len(corridor_times)/2 + i*bar_width,
               corridor_times[i], yerr=yerr, 
               capsize=5, label='Times spent in corridors', width=bar_width)
        ax.set_xlabel('Corridor index')
        ax.set_ylabel('Time (s)')
        ax.set_title('Time spent in each corridor')
        # ax.legend()

    # create the same figure but show cumulative times
    fig, ax = plt.subplots()
    for i in range(len(corridor_times)):
        corridor_times[i] = np.cumsum(corridor_times[i])
        lower_bounds[i] = np.cumsum(lower_bounds[i])
        upper_bounds[i] = np.cumsum(upper_bounds[i])
        yerr = np.array([np.array(corridor_times[i]) - np.array(lower_bounds[i]),
                        np.array(upper_bounds[i]) - np.array(corridor_times[i])])
        ax.bar(np.array(range(len(corridor_times[i])))-bar_width*len(corridor_times)/2 + i*bar_width,
               upper_bounds[i] - lower_bounds[i], bottom=0*(lower_bounds[i] - corridor_times[i]), 
               capsize=5, label='Times spent in corridors', width=bar_width)
        ax.set_xlabel('Corridor index')
        ax.set_ylabel('Time (s)')
        ax.set_title('Uncertainty propagation of time spent in corridor')
        ax.legend()

    # plt.show()

def visualize_overlap(data_files, cumulative_lower_bounds, cumulative_upper_bounds):
    # check if there is overlap in the corridors
    overlapping_pairs = get_overlapping_corridors(data_files[0]['corridor_sequence']['sequence'],
                                                    data_files[1]['corridor_sequence']['sequence'])
    valid_pairs = [True for _ in overlapping_pairs]
    for i, pair in enumerate(overlapping_pairs):
        # get time ranges of both pairs
        range1 = [cumulative_lower_bounds[0][pair[0]], cumulative_upper_bounds[0][pair[0]]]
        range2 = [cumulative_lower_bounds[1][pair[1]], cumulative_upper_bounds[1][pair[1]]]
        print(f"Corridor {pair[0]} of trajectory 1: {range1}")
        print(f"Corridor {pair[1]} of trajectory 2: {range2}")

        # check if they overlap
        if check_overlapping_intervals(range1, range2, tolerance=1.0e-4):
            print(f"Corridor {pair[0]} of trajectory 1 overlaps with corridor {pair[1]} of trajectory 2")
        else:
            valid_pairs[i] = False
        
    # visualize
    xmin = 1.0e10; xmax = -1.0e10; ymin = 1.0e10; ymax = -1.0e10
    plt.figure()
    plt.plot(data_files[0]['corridor_sequence']['start']['x'], 
             data_files[0]['corridor_sequence']['start']['y'], 'ko')
    plt.plot(data_files[0]['corridor_sequence']['dest']['x'], 
             data_files[0]['corridor_sequence']['dest']['y'], 'ko', fillstyle='none')
    plt.plot(data_files[1]['corridor_sequence']['start']['x'], 
             data_files[1]['corridor_sequence']['start']['y'], 'ko')
    plt.plot(data_files[1]['corridor_sequence']['dest']['x'], 
             data_files[1]['corridor_sequence']['dest']['y'], 'ko', fillstyle='none')
    for i, c in enumerate(data_files[0]['corridor_sequence']['sequence']):
        rect = Rectangle((c['x_min'], c['y_min']), c['x_max'] - c['x_min'], c['y_max'] - c['y_min'],
                         linewidth=1, edgecolor='g', facecolor='g', alpha=0.2)
        plt.gca().add_patch(rect)
            
        xmin = min(xmin, c['x_min']); xmax = max(xmax, c['x_max'])
        ymin = min(ymin, c['y_min']); ymax = max(ymax, c['y_max'])

    for c in data_files[1]['corridor_sequence']['sequence']:
        rect = Rectangle((c['x_min'], c['y_min']), c['x_max'] - c['x_min'], c['y_max'] - c['y_min'],
                         linewidth=1, edgecolor='r', facecolor='r', alpha=0.2)
        plt.gca().add_patch(rect)
        xmin = min(xmin, c['x_min']); xmax = max(xmax, c['x_max'])
        ymin = min(ymin, c['y_min']); ymax = max(ymax, c['y_max'])

        plt.plot(data_files[1]['trajectory']['px'],
                 data_files[1]['trajectory']['py'], 'r', alpha=0.5, lw=2)

    for i in range(len(overlapping_pairs)):
        if not valid_pairs[i]:
            continue
        # get a random color:
        ec = np.random.rand(3,)
        c = data_files[0]['corridor_sequence']['sequence'][overlapping_pairs[i][0]]
        rect = Rectangle((c['x_min'], c['y_min']), c['x_max'] - c['x_min'], c['y_max'] - c['y_min'],
                         linewidth=1, edgecolor=ec,  facecolor='none', lw=2)
        plt.gca().add_patch(rect)

        c = data_files[1]['corridor_sequence']['sequence'][overlapping_pairs[i][1]]
        rect = Rectangle((c['x_min'], c['y_min']), c['x_max'] - c['x_min'], c['y_max'] - c['y_min'],
                         linewidth=1, edgecolor=ec, facecolor='none', lw=2)
        plt.gca().add_patch(rect)
    
    xwidth = xmax - xmin; ywidth = ymax - ymin
    plt.xlim(xmin - 0.1*xwidth, xmax + 0.1*xwidth)
    plt.ylim(ymin - 0.1*ywidth, ymax + 0.1*ywidth)
    plt.axis('equal')
    # plt.show()

def analyze_bounds(corridor_times, lower_bounds, upper_bounds):
    # get lower_limit error
    lower_limit_errors = np.array([])
    for i in range(len(corridor_times)):
        lower_limit_errors = np.hstack([lower_limit_errors, np.array(lower_bounds[i]) - np.array(corridor_times[i])])
    lower_limit_errors = np.array(lower_limit_errors)

    # get upper_limit error
    upper_limit_errors = np.array([])
    for i in range(len(corridor_times)):
        upper_limit_errors = np.hstack([upper_limit_errors, np.array(upper_bounds[i]) - np.array(corridor_times[i])])
    upper_limit_errors = np.array(upper_limit_errors)

    # get the interval sizes
    interval_sizes = np.array([])
    for i in range(len(corridor_times)):
        interval_sizes = np.hstack([interval_sizes, np.array(upper_bounds[i]) - np.array(lower_bounds[i])])
    interval_sizes = np.array(interval_sizes)

    # get the mean and std of the errors
    lower_limit_mean = np.mean(lower_limit_errors)
    lower_limit_std = np.std(lower_limit_errors)
    upper_limit_mean = np.mean(upper_limit_errors)
    upper_limit_std = np.std(upper_limit_errors)
    interval_mean = np.mean(interval_sizes)
    interval_std = np.std(interval_sizes)

    max_error = max(max(lower_limit_errors), max(upper_limit_errors))

    # visualize with two histograms
    lower_color = 'green'
    upper_color = 'red'
    fig, ax = plt.subplots(3,1, figsize=(6, 6))
    ax[0].hist(lower_limit_errors, bins=50, color=lower_color, alpha=0.5, label='Lower limit error')
    ax[0].axvline(lower_limit_mean, color=lower_color, linestyle='dashed', linewidth=1)
    ax[0].axvline(0, color='k', lw=2)
    ax[0].set_xlabel('Error (s)')
    ax[0].set_ylabel('Frequency')
    ax[0].set_xlim(-1.1*max_error, 1.1*max_error)
    ax[0].legend()
    
    ax[1].hist(upper_limit_errors, bins=50, color=upper_color, alpha=0.5, label='Upper limit error')
    ax[1].axvline(upper_limit_mean, color=upper_color, linestyle='dashed', linewidth=1)
    ax[1].axvline(0, color='k', lw=2)
    ax[1].set_xlabel('Error (s)')
    ax[1].set_ylabel('Frequency')
    ax[1].set_xlim(-1.1*max_error, 1.1*max_error)
    ax[1].legend()
    
    ax[2].hist(interval_sizes, bins=50, color='blue', alpha=0.5, label='Interval sizes')
    ax[2].axvline(interval_mean, color='blue', linestyle='dashed', linewidth=1)
    ax[2].legend()

    plt.tight_layout()
    plt.savefig('src-python/figures/corridor_time_bounds.png', dpi=300)

def visualize_corridors_with_time(seq1, lb1, ub1, seq2, lb2, lb3):
    lb1 = np.cumsum(lb1)
    ub1 = np.cumsum(ub1)
    lb2 = np.cumsum(lb2)
    lb3 = np.cumsum(lb3)

    import pyvista as pv
    plotter = pv.Plotter()
    for i, c in enumerate(seq1):
        cuboid = pv.Box(bounds=(c['x_min'], c['x_max'], c['y_min'], c['y_max'], lb1[i], ub1[i]))
        plotter.add_mesh(cuboid, color='green', opacity=0.2, show_edges=True)
    
    for i, c in enumerate(seq2):
        cuboid = pv.Box(bounds=(c['x_min'], c['x_max'], c['y_min'], c['y_max'], lb2[i], lb3[i]))
        plotter.add_mesh(cuboid, color='red', opacity=0.2, show_edges=True)

    # Show the plot
    plotter.show()

import random
# test the minimum-time BVP solver
# for i in range(20):
#     p0 = 0*random.uniform(-10, 10)
#     pf = random.uniform(-10, 10)
#     v0 = random.uniform(-10, 10)
#     vf = random.uniform(-10, 10)
#     v_max = max(np.abs(random.uniform(0, 10)), np.abs(v0), np.abs(vf))
#     a_max = random.uniform(0, 10)
#     traj = solve_BVP_1D_min_time(p0, pf, v0, vf, v_max, a_max)
#     if np.any(np.array(traj['tt']) < 0):
#         print(f"Negative time found!: {traj}")
#     visualize_traj(traj)

# test the fixed-time BVP solver
# for i in range(20):
#     p0 = random.uniform(-10, 10)
#     pf = random.uniform(-10, 10)
#     v0 = random.uniform(-10, 10)
#     vf = random.uniform(-10, 10)
#     v_max = max(np.abs(random.uniform(0, 10)), np.abs(v0), np.abs(vf))
#     a_max = random.uniform(0, 10)
#     traj = solve_BVP_1D_fixed_time(p0, pf, v0, vf, 100, v_max, a_max)
#     # print(f"a_max: {traj['a_max']}")
#     # print(f"aa: {traj['aa']}")
#     visualize_traj(traj)

# visualize dependency of T* on a_max
# for i in range(20):
#     T_values = []
#     a_max_values = []
#     p0 = random.uniform(-10, 10)
#     pf = random.uniform(-10, 10)
#     v0 = random.uniform(-10, 10)
#     vf = random.uniform(-10, 10)
#     v_max = max(np.abs(random.uniform(0, 10)), np.abs(v0), np.abs(vf))
#     a_max = random.uniform(0.2, 10)
#     for i in range(1000):
#         traj = solve_BVP_1D_min_time(p0, pf, v0, vf, v_max, a_max)
#         T_values.append(traj['T'])
#         a_max_values.append(a_max)
#         a_max = random.uniform(0.2, 10)

#     plt.figure()
#     plt.plot(a_max_values, T_values, '.')
#     plt.xlabel('a_max')
#     plt.ylabel('T*')
#     plt.ylim([0, 100])

#     print(traj)
#     plt.show()
    
# debugging
# problematic_traj = {'T': np.float64(1.2223110367773882), 'tt': [np.float64(0.6968002441227598), np.float64(0.5255107926546285), np.float64(0.0)], 'aa': [5.194186636936474, 0, -5.194186636936474], 'phat': np.float64(-2.919644461351359), 'vhat': np.float64(9.536162300271386), 'p0': -8.303476454400522, 'pf': 2.0917117479474427, 'v0': 5.916851783634874, 'vf': 9.536162300271386, 'v_max': np.float64(9.536162300271386), 'a_max': 5.194186636936474}
# import time
# for a_max in np.linspace(2.6, 2.8, 10):
#     traj = solve_BVP_1D_min_time(problematic_traj['p0'], problematic_traj['pf'],
#                                  problematic_traj['v0'], problematic_traj['vf'],
#                                  problematic_traj['v_max'], a_max)
#     plt.close()
#     visualize_traj(traj, TIME_PLOT=False, BLOCK=True)
#     # time.sleep(1.5)

# test 2D case
# for i in range(20):
#     p0x = random.uniform(-10, 10)
#     pfx = random.uniform(-10, 10)
#     v0x = random.uniform(-10, 10)
#     vfx = random.uniform(-10, 10)
#     p0y = random.uniform(-10, 10)
#     pfy = random.uniform(-10, 10)
#     v0y = random.uniform(-10, 10)
#     vfy = random.uniform(-10, 10)
#     v_max = max(np.abs(random.uniform(0, 10)), np.abs(v0x), np.abs(vfx), np.abs(v0y), np.abs(vfy))
#     a_max = random.uniform(0, 10)
#     traj_2d = solve_BVP_2D(p0x, p0y, pfx, pfy, v0x, v0y, vfx, vfy, v_max, a_max)
#     visualize_traj_2D(traj_2d)

test_solve_BVP_2D()

exit()

# load the example_trajectory.json file from build/output
data_files = []
file_available = True
file_counter = 0
while file_available:
    try:
        with open(f'build/output/example_trajectory{file_counter}.json', 'r') as f:
            data = json.load(f)
            # if np.sum(data["parametrization"]["t_x_sol"][1]) == 0 and \
            #     data["parametrization"]["nb_corridors"] > 1:
            #      print("Warning: trajectory is a single arc, skipping")
            # else:
            #     data_files.append(data)
            data_files.append(data)
            file_counter += 1
    except FileNotFoundError:
        file_available = False

corridor_times, lower_bounds, upper_bounds = process_data(data_files)
idx0 = 19
idx1 = 412
idx_for_vis = [idx0, idx1]
ct = [corridor_times[i] for i in idx_for_vis]
lb = [lower_bounds[i] for i in idx_for_vis]
ub = [upper_bounds[i] for i in idx_for_vis]
df = [data_files[i] for i in idx_for_vis]
visualize(ct, lb, ub, df)
visualize_overlap(df, [np.cumsum(l) for l in lb], [np.cumsum(u) for u in ub])

analyze_bounds(corridor_times, lower_bounds, upper_bounds)

sequence1 = data_files[idx0]['corridor_sequence']['sequence']
sequence2 = data_files[idx1]['corridor_sequence']['sequence']
lb1 = lower_bounds[idx0]
ub1 = upper_bounds[idx0]
lb2 = lower_bounds[idx1]
ub2 = upper_bounds[idx1]
# visualize_corridors_with_time(sequence1,lb1, ub1, sequence2, lb2, ub2)
        
plt.show()

