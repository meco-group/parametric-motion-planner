# Fast Time-Optimal Motion Planner for Holonomic Vehicles

![image](doc/illustrative-figures/hardware/symposium.jpg)

This repository contains code to compute time-optimal trajectories for holonomic vehicles with low computation time.
The user defines a set of parameters describing the vehicle and an environment to plan in.

## Overview of the approach
The motion planner constructs a corridor sequence simplifying the environment representation and will plan a dynamically feasible trajectory through these corridors. This can be done either using an OCP-solver or using the parametric motion primitives. Based on heuristics and the corridor layout, these motion primitives represent the solution up to some degrees of freedom. These are fixed by solving an optimization problem.
<p float="center">
    <img src="doc/illustrative-figures/code-overview.drawio.svg" width=600/>
</p>

## Validation of the approach
This approach has been validated in simulation and on real hardware.
<p float="center">
  <img src="doc/illustrative-figures/simulation-results/traj_064.png" width="200" />
  <img src="doc/illustrative-figures/simulation-results/traj_131.png" width="200" /> 
  <img src="doc/illustrative-figures/simulation-results/traj_252.png" width="200" />
  <img src="doc/illustrative-figures/simulation-results/traj_446.png" width="200" />
</p>
<!-- ![image](doc/illustrative-figures/simulation-results/traj_064.png)
![image](doc/illustrative-figures/simulation-results/traj_131.png)
![image](doc/illustrative-figures/simulation-results/traj_252.png)
![image](doc/illustrative-figures/simulation-results/traj_446.png) -->

![image](doc/illustrative-figures/hardware/demo.gif)

## Online replanning in dynamic environments
By simulating moving obstacles, a simple event-based replanning scheme can be implemented. When the current corridors are found not to be obstacle-free, the motion planner is triggered to replan.
![image](doc/illustrative-figures/simulation-results/dynamic-simulation.gif)