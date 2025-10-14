import numpy as np
from env.simulation import QuadSimulator, SimulationOptions

from profiles import FootForceProfile
import matplotlib.pyplot as plt

N_LEGS = 4
N_JOINTS = 3
TAU = []
TIME = []

kpJoint = np.array([1,1,1])*100
kdJoint = np.array([0.1,0.1,0.1])*10

kpCartesian = np.diag([300,300,200])*0.1
kdCartesian = np.diag([20,20,20])

k_virtual = 1000


def quadruped_jump():
    # Initialize simulation
    # Feel free to change these options! (except for control_mode and timestep)
    sim_options = SimulationOptions(
        on_rack=False,  # Whether to suspend the robot in the air (helpful for debugging)
        render=True,  # Whether to use the GUI visualizer (slower than running in the background)
        record_video=False,  # Whether to record a video to file (needs render=True)
        tracking_camera=True,  # Whether the camera follows the robot (instead of free)
    )
    simulator = QuadSimulator(sim_options)

    # Determine number of jumps to simulate
    n_jumps = 1  # Feel free to change this number
    jump_duration = 5.0  # TODO: determine how long a jump takes
    n_steps = int(n_jumps * jump_duration / sim_options.timestep)

    # TODO: set parameters for the foot force profile here
    force_profile = FootForceProfile(f0=1, f1=0, Fx=0, Fy=0, Fz=1)

    for _ in range(n_steps):

        # If the simulator is closed, stop the loop
        if not simulator.is_connected():
            break

        # Step the oscillator
        force_profile.step(sim_options.timestep)

        # Compute torques as motor targets
        # The convention is as follows:
        # - A 1D array where the torques for the 3 motors follow each other for each leg
        # - The first 3 elements are the hip, thigh, calf torques for the FR leg.
        # - The order of the legs is FR, FL, RR, RL (front/rear,right/left)
        # - The resulting torque array is therefore structured as follows:
        # [FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf, RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf]
        tau = np.zeros(N_JOINTS * N_LEGS)
        # TODO: implement the functions below, and add potential controller parameters as function parameters here
        tau += nominal_position(simulator, Kp=kpCartesian, Kd=kdCartesian)
        # tau += apply_force_profile(simulator, force_profile)
        tau += gravity_compensation(simulator)
        print(tau)

        # If touching the ground, add virtual model
        on_ground = simulator.get_foot_contacts().all()  # TODO: how do we know we're on the ground?
        # on_ground = True  # TODO: how do we know we're on the ground?
        if on_ground:
            tau += virtual_model(simulator, k=k_virtual)

        # Set the motor commands and step the simulation
        simulator.set_motor_targets(tau)
        simulator.step()

        TAU.append(tau)
        TIME.append(sim_options.timestep)

    # Close the simulation
    simulator.close()

    # OPTIONAL: add additional functions here (e.g., plotting)
    # Plot the force profile

def nominal_position(
    simulator: QuadSimulator,Kp,Kd
    # OPTIONAL: add potential controller parameters here (e.g., gains)
) -> np.ndarray:
    # All motor torques are in a single array
    tau = np.zeros(N_JOINTS * N_LEGS)
    for leg_id in range(N_LEGS):

        # TODO: compute nominal position torques for leg_id
        tau_i = np.zeros(3)
        # print("current angles shape:", current_angles.shape)
        # print("current velocities shape:", current_velocities.shape)


        J, foot_pos = simulator.get_jacobian_and_position(leg_id)

        nominal_position = np.array([0,0,-0.18])  # TODO: define the nominal foot position in the leg frame
        current_position = foot_pos  # TODO: get the current foot position in the leg frame
        current_velocity = J @ simulator.get_motor_velocities(leg_id)  # TODO: get the current foot velocity in the leg frame

        tau_i = J.T @ (Kp @ (nominal_position - current_position) - Kd @ current_velocity)

        # Store in torques array
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i
    return tau

def virtual_model(
    simulator: QuadSimulator, k
    # OPTIONAL: add potential controller parameters here (e.g., gains)
) -> np.ndarray:
    # All motor torques are in a single array
    tau = np.zeros(N_JOINTS * N_LEGS)
    R = simulator.get_base_orientation_matrix()
    R_2 = np.array([[1,1,-1,-1],[-1,1,-1,1],[0,0,0,0]])
    P = R@R_2
    F = np.array([[0,0,0,0],[0,0,0,0],k*np.array([0,0,1])@P])
    for leg_id in range(N_LEGS):

        # TODO: compute virtual model torques for leg_id
        tau_i = np.zeros(3)
        J, foot_pos = simulator.get_jacobian_and_position(leg_id)
        # desired_vel = np.array([0, 0, 0])
        # current_vel = simulator.get_foot_velocity(leg_id) Non-existent
        tau_i = J.T @ F.T[leg_id]
        # Store in torques array
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i

    return tau

def gravity_compensation(
    simulator: QuadSimulator,
    # OPTIONAL: add potential controller parameters here (e.g., gains)
) -> np.ndarray:
    # All motor torques are in a single array
    tau = np.zeros(N_JOINTS * N_LEGS)
    for leg_id in range(N_LEGS):

        # TODO: compute gravity compensation torques for leg_id
        tau_i = np.zeros(3)
        force_gravity = np.array([0, 0, -9.81 * simulator.get_mass() / N_LEGS])
        J, foot_pos = simulator.get_jacobian_and_position(leg_id)
        tau_i = J.T @ force_gravity

        # Store in torques array
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i

    return tau

def apply_force_profile(
    simulator: QuadSimulator,
    force_profile: FootForceProfile,
    # OPTIONAL: add potential controller parameters here (e.g., gains)
) -> np.ndarray:
    # All motor torques are in a single array
    tau = np.zeros(N_JOINTS * N_LEGS)
    for leg_id in range(N_LEGS):

        # TODO: compute force profile torques for leg_id
        tau_i = np.zeros(3)
        J, foot_pos = simulator.get_jacobian_and_position(leg_id)
        force_vector = force_profile.force()
        tau_i = J.T @ force_vector

        # Store in torques array
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i

    return tau

if __name__ == "__main__":
    quadruped_jump()

    TAU = np.array(TAU)
    TIME = np.array(TIME)

    print(TAU.shape)
    plt.plot(np.cumsum(TIME), TAU)
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (Nm)")
    plt.title("Torque vs Time")
    plt.grid()
    plt.show()
