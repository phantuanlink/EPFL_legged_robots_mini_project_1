import numpy as np
from env.simulation import QuadSimulator, SimulationOptions

from profiles import FootForceProfile
from enum import Enum
import matplotlib.pyplot as plt

N_LEGS = 4
N_JOINTS = 3
on_rack = False  # Whether to suspend the robot in the air (helpful for debugging)
global_time_step = 0
vmc = True  # Whether to use virtual model control

"""
optimiazation results:

Forward jump:
Best params: {'k_vmc': 1173.2834329945651, 
'Fx': -272.25032373170086,
Fy': 2.477624562524529, 
'Fz': -811.5666868704336, 
'f0': 1.8245074761316986

Side jump:
Best params: {
'k_vmc': 795.7053405, 
'Fx': -100.56688520, 
'Fy': -32.5900604, 
'Fz': -355.44503760, 
'f0': 1.565263
}

Twist jump:
k_vmc': 943.8208111839183, 
'Fx': 64.41275590430618, 
'Fy': -737.9591640525873, 
'Fz': -1147.0883123971944, 
'f0': 3.7442627484118094

}
"""

class JumpMode(Enum):
    FORWARD = (-272.25032373170086, 2.477, -811.5666868704336)
    SIDE = (-100.56688520, -32.5900604, -355.44503760)
    TWIST = (64.41275590430618, -737.9591640525873, -1147.0883123971944)

    def force(self, scale: float = 1.0) -> np.ndarray:
        """Return the Fx,Fy,Fz as a numpy array optionally scaled."""
        return np.array(self.value, dtype=float) * float(scale)
    
    def f0(self) -> float:
        """Return the f0 frequency for this jump mode."""
        if self == JumpMode.FORWARD:
            return 1.824507
        elif self == JumpMode.SIDE:
            return 1.565263
        elif self == JumpMode.TWIST:
            return 3.744262
        else:
            return 1.0  # default value
    
jump_mode = JumpMode.TWIST

class ControllerParameters:
    def __init__(self):
        self.KpJoint = np.diag([10.0, 200.0, 200.0])
        self.KdJoint = np.diag([0.5, 0.5, 0.5])
    
        ##### Cartesian impedance gains
        self.KpCartesian = np.diag([1000.0, 1000.0, 1000.0]) * 2
        self.KiCartesian = np.diag([0.0, 800.0, 800.0])
        self.KdCartesian = np.diag([30.0, 30.0, 30.0])

        self.h_des = 0.25  ####  Robot height
        self.x_offset_nominal_pos = -0.05
        self.y_offset_nominal_pos = 0.1
        self.dt = 0.001

        self.k_vmc = 943.8208111839183  # Virtual model control gain

        # Per-leg integrator state (initialized to zeros)
        self.foot_error_integral = np.zeros((N_LEGS, 3), dtype=float)
        self.integrator_reset_tol = 0.005

        
    def set_neutral_position(self ,simulator: QuadSimulator,):
        self.foots_neutral = simulator.get_hip_offsets().copy()
        self.foots_neutral[:, -1] -= self.h_des

        self.foots_neutral[0, 1] = -self.y_offset_nominal_pos
        self.foots_neutral[1, 1] = self.y_offset_nominal_pos
        self.foots_neutral[2, 1] = -self.y_offset_nominal_pos
        self.foots_neutral[3, 1] = self.y_offset_nominal_pos

        self.foots_neutral[:, 0] = 0

        # self.foots_neutral[0, 0] = 0
        # self.foots_neutral[1, 0] = 0
        # self.foots_neutral[2, 0] = self.x_offset_nominal_pos
        # self.foots_neutral[3, 0] = self.x_offset_nominal_pos


        print("Neutral foot positions:", self.foots_neutral)

    def reset_integrator(self):
        self.foot_error_integral = np.zeros((N_LEGS, 3), dtype=float)

    def set_time_step(self, dt:float):
        self.dt = dt

params_ = ControllerParameters()


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
    params_.set_neutral_position(simulator)
    params_.reset_integrator()
    params_.set_time_step(sim_options.timestep)

    Fx, Fy, Fz = jump_mode.force(scale=1.0)
    f0 = jump_mode.f0()
    print(f"Using jump mode {jump_mode.name} with forces Fx={Fx}, Fy={Fy}, Fz={Fz}")
    force_profile = FootForceProfile(f0=f0, f1=0.3, Fx=Fx, Fy=Fy, Fz=Fz)

    # Determine number of jumps to simulate
    n_jumps = 15  # Feel free to change this number
    jump_duration = force_profile.impulse_duration() + force_profile.idle_duration()
    n_steps = int((n_jumps * jump_duration ) / sim_options.timestep)
    forces_history = np.zeros((n_steps, 3), dtype=float)

    # TODO: set parameters for the foot force profile here
    global global_time_step
    for _ in range(n_steps):
        # If the simulator is closed, stop the loop
        if not simulator.is_connected():
            break

        # Step the oscillator
        force_profile.step(sim_options.timestep)
        forces_history[global_time_step, :] = force_profile.force()

        # Compute torques as motor targets
        # The convention is as follows:
        # - A 1D array where the torques for the 3 motors follow each other for each leg
        # - The first 3 elements are the hip, thigh, calf torques for the FR leg.
        # - The order of the legs is FR, FL, RR, RL (front/rear,right/left)
        # - The resulting torque array is therefore structured as follows:
        # [FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf, RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf]
        tau = np.zeros(N_JOINTS * N_LEGS)

        # TODO: implement the functions below, and add potential controller parameters as function parameters here
        tau += nominal_position(simulator, params = params_)
        tau += gravity_compensation(simulator)

        # If touching the ground, add virtual model
        on_ground = simulator.get_foot_contacts().all()      
        if on_ground and vmc:
            tau += virtual_model(simulator, params_.k_vmc)
            # roll, pitch, _ = simulator.get_base_orientation_roll_pitch_yaw()
            # upright = (abs(roll) < np.deg2rad(3) and abs(pitch) < np.deg2rad(3))
            # if upright:
            #     print("Robot is standing upright")


        tau += apply_force_profile(simulator, force_profile)

        # Set the motor commands and step the simulation
        simulator.set_motor_targets(tau)
        simulator.step()
        global_time_step += 1

    # Close the simulation
    simulator.close()

    # Plot forces vs time (use only recorded portion in case we broke early)
    # Plot Fx, Fy, Fz across the full n_steps
    # try:
    #     t = np.arange(n_steps) * sim_options.timestep
    #     plt.figure(figsize=(9, 5))
    #     plt.plot(t, forces_history[:, 0], label='Fx', linestyle='-')
    #     plt.plot(t, forces_history[:, 1], label='Fy', linestyle='--')
    #     plt.plot(t, forces_history[:, 2], label='Fz', linestyle=':')
    #     plt.xlabel('Time (s)')
    #     plt.ylabel('Force (N)')
    #     plt.title('Force profile (Fx, Fy, Fz) from force_profile.force()')
    #     plt.legend()
    #     plt.grid(True)
    #     plt.tight_layout()
    #     plt.show()
    # except Exception as e:
    #     print("Plotting failed:", e)

    # OPTIONAL: add additional functions here (e.g., plotting)


def nominal_position(
    simulator: QuadSimulator,
    params: ControllerParameters,
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
        tau_i += params.KdJoint @ (0 - simulator.get_motor_velocities(leg_id))
        
        # Cartesian PD
        err = nominal_foot_pos - cur_pos
        err_norm = np.linalg.norm(err)

        if err_norm > params.integrator_reset_tol:
        #     params.foot_error_integral[leg_id].fill(0.0)
        # else:
            params.foot_error_integral[leg_id] += err * params.dt
        tau_i +=  J.T @ (params.KpCartesian @ err +
                         # params.KiCartesian @ params.foot_error_integral[leg_id] + 
                         params.KdCartesian @ (0 - (J @ simulator.get_motor_velocities(leg_id))))
        
        # if global_time_step % 1000 == 0:
        #     print("prams.KpCartesian:", params.KpCartesian)
        #     print("prams.KdCartesian:", params.KdCartesian)
        #     print(f"Mass of robot: {simulator.get_mass()}")
        #     print(f"Leg {leg_id} cur_pos: {cur_pos}, nominal_foot_pos: {nominal_foot_pos}")
        # Store in torques array
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i
    return tau


def virtual_model(
    simulator: QuadSimulator,
    k_vmc: float,
    # OPTIONAL: add potential controller parameters here (e.g., gains)
) -> np.ndarray:
    # All motor torques are in a single array
    tau = np.zeros(N_JOINTS * N_LEGS)
    for leg_id in range(N_LEGS):
        tau_i = np.zeros(3)

        Rotation_matrix = simulator.get_base_orientation_matrix()
        C = np.array([[ 1,  1, -1, -1],
                  [-1,  1, -1,  1],
                  [ 0,  0,  0,  0]], dtype=float)
        P = Rotation_matrix @ C
        Fz = k_vmc * P[2, leg_id]

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

    for leg_id in range(N_LEGS):
        gravity_compensation = -np.array([0, 0, 9.81 * simulator.get_mass() / N_LEGS])
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
    scale = np.ones(N_LEGS)

    for leg_id in range(N_LEGS):
        J, _ = simulator.get_jacobian_and_position(leg_id)
        F_inst = force_profile.force()
        if (jump_mode == JumpMode.TWIST) and (leg_id>=2):
            F_inst[1] *= -1  # flip y-force for rear legs in twist mode
        elif (jump_mode == JumpMode.SIDE):
            if (F_inst[1] < 0 and leg_id % 2 == 0) or (F_inst[1] > 0 and leg_id % 2 == 1):
                # F_inst[1] *= 1.5  # flip y-force for left legs in side jump
                F_inst[1] *= 1.0  # flip y-force for left legs in side jump

        # if ((jump_mode == JumpMode.FORWARD or jump_mode == JumpMode.SIDE)) and leg_id < 2:
        #     scale[leg_id] = 1.0 # reduce front leg force in forward jump

        tau_i = J.T @ F_inst * scale[leg_id]
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i

    return tau


if __name__ == "__main__":
    quadruped_jump()
