# This file is part of OMG-tools.
#
# OMG-tools -- Optimal Motion Generation-tools
# Copyright (C) 2016 Ruben Van Parys & Tim Mercy, KU Leuven.
# All rights reserved.
#
# OMG-tools is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA


# This example tests a separate multiframeproblem. Normally, a schedulerproblem creates
# a multiframeproblem, and solves it with a receding horizon. This example allows testing
# the behavior of a multiframeproblem on itself. Beware that this problem will not solve
# completely if using simulator.run(), this is because the amount of frames that are coupled
# stays the same. E.g. when coupling two frames, normally the first frame is dropped
# (by the scheduler) as soon as the vehicle enters the second frame, this is not possible
# when considering only a single multiframeproblem. Therefore, the vehicle cannot leave
# the first frame.

# The frames that are considered here are of the type 'corridor', i.e. frames that are as
# large as possible, without containing any stationary obstacle.

# export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/local/lib/" (in the terminal you're running this script from!)
# ln -s libcoinhsl.so libhsl.so (only once)


import sys
sys.path.append('python-benchmark/')
from omgtools import *
import matplotlib.pyplot as plt
import time   

def omg_example(corridors, start, goal, v_max, a_max, veh_w, veh_h, 
                dump_to_json=False, file_name="", start_vel=[0, 0]):
    # create vehicle
    # time_a = time.time()
    vehicle = Holonomic(shapes=Rectangle(veh_w, veh_h, 0), 
                        options={'syslimit': 'norm_inf', 'stop_tol': 1.e-7},
                        bounds={'vxmax': v_max, 'vxmin':-v_max, 
                                'vymax': v_max, 'vymin':-v_max,
                                'axmax': a_max, 'axmin':-a_max, 
                                'aymax': a_max, 'aymin':-a_max})
    # time_b = time.time()
    # print(f"Time for creating vehicle: {round(time_b-time_a,3)}s")

    # time_a = time.time()
    # create environment
    vehicle.set_initial_conditions(state=start, input=start_vel)
    vehicle.set_terminal_conditions(goal)

    rooms = []
    for corridor in corridors:
        xmin, xmax, ymin, ymax = corridor
        width = xmax - xmin
        height = ymax - ymin
        position = [(xmin + xmax)/2, (ymin + ymax)/2]
        room = {'shape': Rectangle(width=width, height=height), 
                'position': position, 'draw':True}
        rooms.append(room)

    # time_b = time.time()
    # print(f"Time for creating rooms: {round(time_b-time_a,3)}s")
    
    # time_a = time.time()
    environment = Environment(room=rooms)
    # time_b = time.time()
    # print(f"Time for creating environment: {round(time_b-time_a,3)}s")

    # Warning: if you use fill room, then also fill the empty rooms with []
    for room in rooms:
        environment.fill_room(room, [])

    # make problem
    # time_a = time.time()
    multiframeproblem=MultiFrameProblem(vehicle, environment, 
                                        n_frames=len(rooms))
    multiframeproblem.set_options({'solver_options': 
        {'ipopt': {'ipopt.linear_solver': 'ma27', 'ipopt.print_level':0}}}) # hsl solvers required
    # multiframeproblem.set_options({'solver_options':
    #                                {'ipopt': {'ipopt.print_level': 5}}})
    multiframeproblem.init()
    # time_b = time.time()
    # print(f"Time for creating multiframe problem: {round(time_b-time_a,3)}s")

    # simulate the problem
    # time_a = time.time()
    deployer = Deployer(multiframeproblem)
    # time_b = time.time()
    # print(f"Time for creating simulator: {round(time_b-time_a,3)}s")

    # run it!
    # trajectories = deployer.update(0)
    deployer.update(0)
    comp_time = multiframeproblem.update_times

    # t = []
    t0 = 0
    # x, dx, ddx = [], [], []
    # y, dy, ddy = [], [], []

    try:
        for spline, seg_time in zip(vehicle.result_spline_segments, vehicle.segment_times):
            # for i in range(0, N):
            #     t.append(t0 + seg_time*i/N)
                # x.append(float(spline[0](i/N)))
                # dx.append(float(spline[0].derivative(1)(i/N)/seg_time))
                # ddx.append(float(spline[0].derivative(2)(i/N)/seg_time**2))
                # y.append(float(spline[1](i/N)))
                # dy.append(float(spline[1].derivative(1)(i/N)/seg_time))
                # ddy.append(float(spline[1].derivative(2)(i/N)/seg_time**2))
            t0 += seg_time

        if dump_to_json:
            import json
            t0 = 0
            N = 100
            px = []
            py = []
            vx = []
            vy = []
            t = []
            for spline, seg_time in zip(vehicle.result_spline_segments, vehicle.segment_times):
                for i in range(0, N):
                    t.append(t0 + seg_time*i/N)
                    px.append(float(spline[0](i/N)))
                    vx.append(float(spline[0].derivative(1)(i/N)/seg_time))
                    # ddx.append(float(spline[0].derivative(2)(i/N)/seg_time**2))
                    py.append(float(spline[1](i/N)))
                    vy.append(float(spline[1].derivative(1)(i/N)/seg_time))
                    # ddy.append(float(spline[1].derivative(2)(i/N)/seg_time**2))
                t0 += seg_time
                    
            j = {"trajectory": {"t":t, "px": px, "py": py, "vx": vx, "vy": vy}}
            with open(file_name, 'w') as f:
                json.dump(j, f, indent=4)
            print(f"Tf: {t[-1]:.3f}")
    except:
        t0 = 0    

    # time_b = time.time()
    # print(f"Time for creating simulator: {round(time_b-time_a,3)}s")
    solver_time = comp_time
    # total_time = time_b-time_a
    travel_time = t0
    print(f"Tf: {travel_time:.3f}\n")
    # print(f"solver_time: {solver_time}")

    return solver_time[-1], travel_time


# corridors = [(0, 1, 3, 5), (0, 2, 3, 4), (1, 2, 2, 4), (1, 3, 2, 3), 
#              (2, 3, 1, 3), (2, 4, 1, 2), (3, 4, 0, 2), (3, 5, 0, 1)]
# start = [0.5, 4.5]
# destination = [4.5, 0.5]

# traj = omg_example(corridors, start, destination, 2.0, 6.0, 0.115, 0.115)

# problematic example:
# corridors = [(0, 1, 0, 2), (0, 3, 1, 2), (2, 3, 1, 5), (2, 5, 4, 5),
#              (4, 5, 2, 5), (4, 6, 2, 3), (5, 6, 0, 3), (4, 6, 0, 1)]
# start = [0.5, 0.5]
# destination = [4.5, 0.5]
# traj = omg_example(corridors, start, destination)
