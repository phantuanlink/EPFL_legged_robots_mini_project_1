import optuna
import numpy as np
from functools import partial
from optuna.trial import Trial
from env.simulation import QuadSimulator, SimulationOptions

from profiles import FootForceProfile

from quadruped_jump import (
    nominal_position,
    gravity_compensation,
    apply_force_profile,
    virtual_model,
    ControllerParameters,
    JumpMode
)

import matplotlib.pyplot as plt
import random

params_ = ControllerParameters()
jumpmode_ = JumpMode.FORWARD
single_jump = True
N_LEGS = 4
N_JOINTS = 3


def quadruped_jump_optimization(optim_run):
    # Initialize simulation
    # Feel free to change these options! (except for control_mode and timestep)
    sim_options = SimulationOptions(
        on_rack=False,  # Whether to suspend the robot in the air (helpful for debugging)
        render=False,  # Whether to use the GUI visualizer (slower than running in the background)
        record_video=False,  # Whether to record a video to file (needs render=True)
        tracking_camera=True,  # Whether the camera follows the robot (instead of free)
    )
    simulator = QuadSimulator(sim_options)
    print("Hip offsets:", simulator.get_hip_offsets())
    params_.set_neutral_position(simulator)
    params_.reset_integrator()
    params_.set_time_step(sim_options.timestep)

    # Create a maximization problem
    objective = partial(evaluate_jumping, simulator=simulator)
    sampler = optuna.samplers.TPESampler(seed=random.randint(1,1000))
    study = optuna.create_study(
        study_name="Quadruped Jumping Optimization",
        sampler=sampler,
        direction="maximize",
    )

    # Run the optimization
    # You can change the number of trials here
    study.optimize(objective, n_trials=50)

    # Close the simulation
    simulator.close()

    # Log the results
    print("Best value:", study.best_value)
    print("Best params:", study.best_params)

    # OPTIONAL: add additional functions here (e.g., plotting, recording to file)
    # E.g., cycling through all the evaluated parameters and values:
    for trial in study.get_trials():
        optim_run[0].append(trial.number)  # Number of the trial
        #trial.params  # Used parameters
        if optim_run[1] == []:
            optim_run[1].append(trial.value)  # Resulting objective function value
        else:
            optim_run[1].append(max(trial.value, optim_run[1][-1]))


def evaluate_jumping(trial: Trial, simulator: QuadSimulator) -> float:

    # TODO: pick optimization variables
    # The following function creates an optimization variable with given name and lower and upper bounds
    # You can then plug in the value in your controller
    k_vmc = trial.suggest_float(name="k_vmc", low=500, high=2000)

    # Reset the simulation
    simulator.reset()

    # Extract simulation options
    sim_options = simulator.options

    # Determine number of jumps to simulate
    n_jumps = 10  # Feel free to change this number

    # TODO: set parameters for the foot force profile here
    f1 = 0.5
    if (jumpmode_ == JumpMode.FORWARD):
        Fx = trial.suggest_float("Fx", low=0, high=300)
        Fy = 0
        Fz = trial.suggest_float("Fz", low=200, high=500)
        f0 = trial.suggest_float("f0", low=1.4, high=4.5)
        if not single_jump:
            f1 = trial.suggest_float("f1", low=0.1, high=0.6)
            Fx = trial.suggest_float("Fx", low=100, high=300)
            f0 = trial.suggest_float("f0", low=2, high=4.5)
    elif (jumpmode_ == JumpMode.SIDE):
        Fx = 0
        Fy = trial.suggest_float("Fy", low=0, high=300)  # negative Fy to jump to the positive Y direction
        Fz = trial.suggest_float("Fz", low=200, high=500)
        f0 = trial.suggest_float("f0", low=1.4, high=4.5)

    elif (jumpmode_ == JumpMode.TWIST):
        Fx = 0
        Fy = trial.suggest_float("Fy", low=0, high=300)  # negative Fy to jump to the positive Y direction
        Fz = trial.suggest_float("Fz", low=200, high=700)
        f0 = trial.suggest_float("f0", low=1.4, high=4.5)



    force_profile = FootForceProfile(single_jump=single_jump,f0=f0, f1=f1, Fx=Fx, Fy=Fy, Fz=Fz)
    jump_duration = force_profile.impulse_duration() + force_profile.idle_duration()
    n_steps = int((n_jumps * jump_duration + force_profile.idle_duration()) / sim_options.timestep)

    accumulated_roll = 0.0
    accumulated_pitch = 0.0
    accumulated_yaw = 0.0
    yaw_rate_ = 0.0
    _, _, yaw = simulator.get_base_orientation_roll_pitch_yaw()
    initial_pos = simulator.get_base_position().copy()

    objective_value = 0.0


    for _ in range(n_steps):
        # Step the oscillator
        force_profile.step(sim_options.timestep)

        # Compute torques as motor targets (reuses your controller functions)
        # OPTIONAL: add potential extra controller parameters here
        tau = np.zeros(N_JOINTS * N_LEGS)
        tau += nominal_position(simulator, params = params_)
        tau += apply_force_profile(simulator, force_profile, jump_mode=jumpmode_)
        tau += gravity_compensation(simulator)

        # If touching the ground, add virtual model
        on_ground = simulator.get_foot_contacts().all()
        if on_ground:
            tau += virtual_model(simulator, k_vmc)

        # Set the motor commands and step the simulation
        simulator.set_motor_targets(tau)
        simulator.step()

        roll_, pitch_, yaw_ = simulator.get_base_orientation_roll_pitch_yaw()
        accumulated_roll += np.abs(roll_)
        accumulated_pitch += np.abs(pitch_)
        accumulated_yaw += np.abs(yaw_)

        new_yaw = simulator.get_base_orientation_roll_pitch_yaw()[2]
        yaw_rate_ += (new_yaw - yaw) / sim_options.timestep
        yaw = new_yaw

        

    # TODO: implement an objective function and return its value
    # Note: the objective function is maximized!
    if jumpmode_ == JumpMode.FORWARD:
        #print(f"Furthest jump value: X={simulator.get_base_position()[0]}")
        objective_value += np.abs(simulator.get_base_position()[0])/(n_steps*sim_options.timestep) # maximize forward distance
        objective_value *=  1-np.abs(simulator.get_base_orientation_roll_pitch_yaw()[2]/(np.pi))
    elif jumpmode_ == JumpMode.SIDE:
        #print(f"Furthest jump value: Y={simulator.get_base_position()[1]}")
        objective_value += np.abs(simulator.get_base_position()[1]) # maximize sideway distance
        objective_value /= (np.abs(simulator.get_base_position()[0])) # penalize for forward movement
    elif jumpmode_ == JumpMode.TWIST:
        objective_value += accumulated_yaw / (2*np.pi)
        objective_value /= (np.linalg.norm(simulator.get_base_position() - initial_pos))**2
 
    objective_value *= simulator.get_foot_contacts().all()
    if not single_jump:
        objective_value /= accumulated_yaw

    return objective_value


if __name__ == "__main__":
    optim_array = []
    N_TRIALS = 1
    for i in range(N_TRIALS):
        optim_run = [[],[]]
        quadruped_jump_optimization(optim_run)
        optim_array.append(optim_run)

    optim_trials = np.array(optim_array[0][0])
    optim_values = np.array([optim_array[k][1] for k in range(N_TRIALS)])

    optim_values_mean = np.mean(optim_values, axis=0)
    optim_values_std = np.std(optim_values, axis=0)


    plt.figure()
    for i in range(N_TRIALS):
        plt.plot(optim_trials,optim_values[i],label=str(i))
    plt.show()


    plt.figure(figsize=(12, 5), layout='constrained')
    plt.title('Continuous forwards jump')
    plt.xlabel('Iteration')
    plt.ylabel('Objective value')
    plt.plot(optim_trials, optim_values_mean, ls='-', color='navy', lw=3)
    plt.fill_between(optim_trials, optim_values_mean + optim_values_std, optim_values_mean - optim_values_std, color='royalblue',
                     alpha=0.4)
    plt.grid(True)
    #plt.savefig('continuous_forwards_jump1.svg')
    plt.show()
