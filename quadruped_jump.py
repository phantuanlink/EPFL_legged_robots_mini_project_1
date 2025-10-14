import numpy as np
from env.simulation import QuadSimulator, SimulationOptions

from profiles import FootForceProfile
from enum import Enum

N_LEGS = 4
N_JOINTS = 3
on_rack = False  # Whether to suspend the robot in the air (helpful for debugging)
global_time_step = 0
vmc = True  # Whether to use virtual model control


class JumpMode(Enum):
    FORWARD = (-150.0, 0.0, -400.0)
    SIDE = (-200.0, -200.0, -500.0)
    TWIST = (0.0, 300.0, -1000.0)

    def force(self, scale: float = 1.0) -> np.ndarray:
        """Return the Fx,Fy,Fz as a numpy array optionally scaled."""
        return np.array(self.value, dtype=float) * float(scale)

jump_mode = JumpMode.SIDE

class ControllerParameters:
    def __init__(self):
        self.KpJoint = np.diag([200.0, 200.0, 200.0])
        self.KdJoint = np.diag([0.5, 0.5, 0.5])
    
        ##### Cartesian impedance gains
        self.KpCartesian = np.diag([1000.0, 1000.0, 1000.0])
        self.KiCartesian = np.diag([0.0, 500.0, 500.0])
        self.KdCartesian = np.diag([20.0, 20.0, 20.0])

        # Robot height
        self.h_des = 0.20
        self.dt = 0.001

        self.k_vmc = 1000.0  # Virtual model control gain

        # Per-leg integrator state (initialized to zeros)
        self.foot_error_integral = np.zeros((N_LEGS, 3), dtype=float)
        self.integrator_reset_tol = 0.01

        # --- Force-profile (oscillator) state ---
        # f0: impulse-phase frequency, f1: idle-phase frequency
        self.force_f0 = 1.5
        self.force_f1 = 0.2
        # amplitude vector (Fx, Fy, Fz) in the frame expected by your Jacobian
        self.force_amp = np.array([0.0, 0.0, -200.0], dtype=float)
        # phase theta [0, 2*pi)
        self.theta = 0.0
        # whether the oscillator is enabled
        self.force_profile_enabled = False

        
    def set_neutral_position(self ,simulator: QuadSimulator,):
        self.foots_neutral = simulator.get_hip_offsets()
        self.foots_neutral[:, -1] -= self.h_des
        self.foots_neutral[:, 0] = 0
        print("Neutral foot positions:", self.foots_neutral)

    def reset_integrator(self):
        self.foot_error_integral = np.zeros((N_LEGS, 3), dtype=float)

    def set_time_step(self, dt:float):
        self.dt = dt

params = ControllerParameters()


def quadruped_jump():
    # Initialize simulation
    # Feel free to change these options! (except for control_mode and timestep)
    sim_options = SimulationOptions(
        on_rack=on_rack,  # Whether to suspend the robot in the air (helpful for debugging)
        render=True,  # Whether to use the GUI visualizer (slower than running in the background)
        record_video=False,  # Whether to record a video to file (needs render=True)
        tracking_camera=True,  # Whether the camera follows the robot (instead of free)
    )
    simulator = QuadSimulator(sim_options)
    print("Hip offsets:", simulator.get_hip_offsets())
    params.set_neutral_position(simulator)
    params.reset_integrator()
    params.set_time_step(sim_options.timestep)

    jump_mode = JumpMode.SIDE
    Fx, Fy, Fz = jump_mode.force(scale=0.8)
    force_profile = FootForceProfile(f0=3.0, f1=0.2, Fx=Fx, Fy=Fy, Fz=Fz)

    # Determine number of jumps to simulate
    n_jumps = 10  # Feel free to change this number
    jump_duration = force_profile.impulse_duration() + force_profile.idle_duration()  # TODO: determine how long a jump takes
    n_steps = int(n_jumps * jump_duration / sim_options.timestep)

    # TODO: set parameters for the foot force profile here

    global global_time_step
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
        tau += nominal_position(simulator)

        #if (not on_rack) or (force_profile.phase() < np.pi):
        #tau += gravity_compensation(simulator)

        # If touching the ground, add virtual model
        on_ground = simulator.get_foot_contacts().all()
        if global_time_step % 500 == 0:
            print(f"On ground: {on_ground}")
        
        if on_ground:
            tau += apply_force_profile(simulator, force_profile)

        if on_ground and vmc:
            tau += virtual_model(simulator)

        # Set the motor commands and step the simulation
        simulator.set_motor_targets(tau)
        simulator.step()
        global_time_step += 1

    # Close the simulation
    simulator.close()

    # OPTIONAL: add additional functions here (e.g., plotting)


def nominal_position(
    simulator: QuadSimulator,
    # OPTIONAL: add potential controller parameters here (e.g., gains)
) -> np.ndarray:
    # All motor torques are in a single array
    tau = np.zeros(N_JOINTS * N_LEGS)
    for leg_id in range(N_LEGS):

        # TODO: compute nominal position torques for leg_id
        tau_i = np.zeros(3)
        J, cur_pos = simulator.get_jacobian_and_position(leg_id)
        nominal_foot_pos = params.foots_neutral[leg_id]

        # Joint PD TODO:
        #solve ik to get nominal joint angles
        # nominal_joint_angles = ik_numerical(np.zeros(3), nominal_foot_pos, leg_id)
        # cur_joint_angles = simulator.get_motor_angles(leg_id)
        tau_i += params.KdJoint @ (0 - simulator.get_motor_velocities(leg_id))
        
        # Cartesian PD
        err = nominal_foot_pos - cur_pos
        err_norm = np.linalg.norm(err)

        if err_norm < params.integrator_reset_tol:
            params.foot_error_integral[leg_id].fill(0.0)
        else:
            params.foot_error_integral[leg_id] += err * params.dt
        tau_i +=  J.T @ (params.KpCartesian @ err +
                         #params.KiCartesian @ params.foot_error_integral[leg_id] + 
                         params.KdCartesian @ (0 - (J @ simulator.get_motor_velocities(leg_id))))
        
        if global_time_step % 500 == 0:
            print(f"Mass of robot: {simulator.get_mass()}")
            print(f"Leg {leg_id} cur_pos: {cur_pos}, nominal_foot_pos: {nominal_foot_pos}")
        # Store in torques array
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i
    return tau


def virtual_model(
    simulator: QuadSimulator,
    # OPTIONAL: add potential controller parameters here (e.g., gains)
) -> np.ndarray:
    # All motor torques are in a single array
    tau = np.zeros(N_JOINTS * N_LEGS)
    for leg_id in range(N_LEGS):
        # TODO: compute virtual model torques for leg_id
        tau_i = np.zeros(3)

        Rotation_matrix = simulator.get_base_orientation_matrix()
        C = np.array([[ 1,  1, -1, -1],
                  [-1,  1, -1,  1],
                  [ 0,  0,  0,  0]], dtype=float)
        P = Rotation_matrix @ C
        Fz = params.k_vmc * P[2, leg_id]

        # clamp vertical force to avoid spikes
        F_i = np.array([0.0, 0.0, Fz])  # world-frame vertical force at foot

        # Get linear Jacobian (world) for this foot.
        # If your API returns a 6xN geometric J, take Jv = J[0:3, actuated_dofs].
        J, _= simulator.get_jacobian_and_position(leg_id)  # expected shape (3,3) for a 3-DoF leg
        tau_i = J.T @ F_i


        # Store in torques array
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i

    return tau


def gravity_compensation(
    simulator: QuadSimulator,
    # OPTIONAL: add potential controller parameters here (e.g., gains)
) -> np.ndarray:
    # All motor torques are in a single array
    tau = np.zeros(N_JOINTS * N_LEGS)

    #TODO: Nathan -- for now, i just naively assume weight is equally distributed on each leg
    for leg_id in range(N_LEGS):
        gravity_compensation = np.array([0, 0, -9.81 * simulator.get_mass() / N_LEGS])
        J, _ = simulator.get_jacobian_and_position(leg_id)
        gravity_compensation += J.T @ gravity_compensation

        # Store in torques array
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = gravity_compensation

    return tau


def apply_force_profile(
    simulator: QuadSimulator,
    force_profile: FootForceProfile,
    # OPTIONAL: add potential controller parameters here (e.g., gains)
) -> np.ndarray:
    # All motor torques are in a single array
    tau = np.zeros(N_JOINTS * N_LEGS)
    for leg_id in range(N_LEGS):
        J, _ = simulator.get_jacobian_and_position(leg_id)
        F_inst = force_profile.force()
        if (jump_mode == JumpMode.TWIST) and (leg_id>=2):
            F_inst[1] *= -1  # flip y-force for rear legs in twist mode
        elif (jump_mode == JumpMode.SIDE):
            if (F_inst[1] < 0 and leg_id % 2 == 0) or (F_inst[1] > 0 and leg_id % 2 == 1):
                F_inst[1] *= 5  # flip y-force for left legs in side jump
        tau_i = J.T @ F_inst
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i

    return tau


if __name__ == "__main__":
    quadruped_jump()
