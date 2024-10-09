import matplotlib.pyplot as plt
import numpy as np 

class PIDController:
    """
    Command accelerations to the drone to follow the subject according to the
    desired settings and avoid the obstacle.
    """

    def __init__(self, params):
      self.params = params

      self.planner_params = {'future_timesteps_n': 1100,
                             'collision_avoidance_factor': 2.0}

      self.controller_gains = {'gain_p': 5,
                               'gain_i': 0.001,
                               'gain_d': 8.0}

      self.subject_velocity_last = np.array([0, 0])
      self.error_last = 0
      self.integral_term_last = 0

    def compute_accel(
        self,
        t: float,
        drone_position : np.ndarray,
        drone_velocity : np.ndarray,
        subject_position : np.ndarray,
        subject_velocity : np.ndarray,
        obstacle_position : np.ndarray,
        obstacle_velocity : np.ndarray
      ) -> np.ndarray:
        """
        Return the desired acceleration commands of the drone.
        """

        desired_position = self.compute_desired_position(
            dt=self.params['dt'],
            drone_position=drone_position,
            subject_position=subject_position,
            subject_velocity=subject_velocity,
            subject_velocity_last=self.subject_velocity_last,
            obstacle_position=obstacle_position,
            desired_range_to_subject=self.params['desired_range_to_subject'],
            collision_avoidance_factor=self.planner_params['collision_avoidance_factor'],
            future_timesteps_n=self.planner_params['future_timesteps_n']
            )

        error = desired_position - drone_position

        proportional, integral, derivative  = self.compute_PID_outputs(
            dt=self.params['dt'],
            error=error,
            gain_p=self.controller_gains['gain_p'],
            gain_i=self.controller_gains['gain_i'],
            gain_d=self.controller_gains['gain_d'],
            integral_term_last=self.integral_term_last,
            error_last=self.error_last,
            )

        overall_control_accel = proportional + integral + derivative

        self.error_last = error
        self.integral_term_last = integral
        self.subject_velocity_last = subject_velocity

        return overall_control_accel

    def compute_desired_position(
        self,
        dt : float,
        drone_position : np.ndarray,
        subject_position : np.ndarray,
        subject_velocity : np.ndarray,
        subject_velocity_last: np.ndarray,
        obstacle_position : np.ndarray,
        desired_range_to_subject : float,
        collision_avoidance_factor : float,
        future_timesteps_n : int,
      ) -> np.ndarray:

        """
        Return the desired position of the drone.
        """

        subject_acceleration_last = subject_velocity - subject_velocity_last
        desired_velocity = subject_velocity + subject_acceleration_last * dt * future_timesteps_n

        vector_obstacle_to_drone = drone_position - obstacle_position
        collision_inflation = collision_avoidance_factor / np.linalg.norm(vector_obstacle_to_drone)

        desired_position = subject_position + desired_range_to_subject * desired_velocity / np.linalg.norm(desired_velocity)
        desired_position += collision_inflation * vector_obstacle_to_drone / np.linalg.norm(vector_obstacle_to_drone)

        return desired_position

    def compute_PID_outputs(
        self,
        dt : float,
        error : np.ndarray,
        gain_p : float,
        gain_i : float,
        gain_d : float,
        integral_term_last : float,
        error_last : float,
      ) -> np.ndarray:

        """
        Return the (proportional, integral, and derivative) terms for a PID controller,
        based on the tracking error.
        """
        proportional = gain_p * error
        integral = gain_i * error * dt + integral_term_last
        derivative = gain_d * (error - error_last) / dt

        return np.array([proportional, integral, derivative])
