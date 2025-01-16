from copy import copy
from ruckig import InputParameter, OutputParameter, Result, Ruckig
import numpy as np
import pdb

def joint_traj_ruckig(n_dof, cycle_rate, waypoints, inter_pts = 100):
    waypoints_np = np.loadtxt(waypoints)
    waypoints = waypoints_np.tolist()
    otg = Ruckig(n_dof, cycle_rate, inter_pts)
    inp = InputParameter(n_dof)
    out = OutputParameter(n_dof, inter_pts)
    inp.current_position = waypoints[0]
    inp.current_velocity = [0, 0, 0, 0, 0, 0]
    inp.current_acceleration = [0, 0, 0, 0, 0, 0]
    inp.intermediate_positions = waypoints[1:-1]
    inp.target_position = waypoints[-1]
    inp.target_velocity = [0, 0, 0, 0, 0, 0]
    inp.target_acceleration = [0, 0, 0, 0, 0, 0]
    inp.max_velocity = [5, 5, 5, 5, 5, 5]
    inp.max_acceleration = [100, 100, 100, 100, 100, 100]
    inp.max_jerk = [5000,5000,5000,5000,5000,5000] 
    print('\t'.join(['t'] + [str(i) for i in range(otg.degrees_of_freedom)]))
    first_output, out_list = None, []
    res = Result.Working
    with open('output.txt', 'w') as file:
        while res == Result.Working:
            res = otg.update(inp, out)
            file.write('\t'.join([f'{p:0.15f}' for p in out.new_position]) + '\n')
            print('\t'.join([f'{out.time:0.6f}'] + [f'{p:0.15f}' for p in out.new_position]))
            out_list.append(copy(out))
            out.pass_to_input(inp)
            if not first_output:
                first_output = copy(out)
 
        print(f'Calculation duration: {first_output.calculation_duration:0.1f} [µs]')
        print(f'Trajectory duration: {first_output.trajectory.duration:0.4f} [s]')
 
def car_traj_ruckig(n_dof, cycle_rate, waypoints, inter_pts = 10000):
    waypoints_np = np.loadtxt(waypoints)
    waypoints = waypoints_np.tolist()

    otg = Ruckig(n_dof, cycle_rate, inter_pts)
    inp = InputParameter(n_dof)
    out = OutputParameter(n_dof, inter_pts)

    inp.current_position = waypoints[0]
    inp.current_velocity = [0, 0, 0]
    inp.current_acceleration = [0, 0, 0]

    inp.intermediate_positions = waypoints[1:-1]

    inp.target_position = waypoints[-1]
    inp.target_velocity = [0, 0, 0]
    inp.target_acceleration = [0, 0, 0]

    inp.max_velocity = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
    inp.max_acceleration = [1, 1, 1, 1, 1, 1]
    inp.max_jerk = [5,5,5,5,5,5]
    print('\t'.join(['t'] + [str(i) for i in range(otg.degrees_of_freedom)]))
 
    first_output, out_list = None, []
    res = Result.Working
    while res == Result.Working:
        res = otg.update(inp, out)
 
        print('\t'.join([f'{out.time:0.6f}'] + [f'{p:0.6f}' for p in out.new_position]))
        out_list.append(copy(out))
 
        out.pass_to_input(inp)
 
        if not first_output:
            first_output = copy(out)
 
    print(f'Calculation duration: {first_output.calculation_duration:0.1f} [µs]')
    print(f'Trajectory duration: {first_output.trajectory.duration:0.4f} [s]')