import math
import time
import ntcore
from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Translation2d

import config
from units.SI import seconds
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import constants
from field_odometry import FieldOdometry

class TrajectoryCalculator:
    """
    TODO: FIND DRAG COEFFICIENT!!!!!!

    Game-piece trajectory calculator that updates based on odometry and vision data.

    Inputs:
    c = drag coefficient
    a = projectile area (m^2)
    m = projectile mass (kg)
    rho_air = air density (kg/m^3)
    g = acceleration due to gravity (m/s^2)
    v0 = initial velocity of shooter flywheel (m/s) config
    delta_x = distance from shooter to target (COULD BE IN ODOMETRY) (m)
    y = height of target (COULD BE IN ODOMETRY) (m) const
    tol = tolerance of error in distance to target (m)
    """
    delta_z: float
    speaker_z: float = constants.speaker_z
    distance_to_target: float
    def __init__(self, odometry: FieldOdometry, elevator):
        self.odometry = odometry
        self.speaker = constants.speaker_location
        self.k = 0.5 * constants.c * constants.rho_air * constants.a
        self.distance_to_target = 0
        self.delta_z = 0
        self.shoot_angle = 0
        self.elevator = elevator
    def calculate_angle_no_air(self, distance_to_target: float, delta_z) -> float:
        """
        Calculates the angle of the trajectory without air resistance.
        """

        phi0 = np.arctan(delta_z / distance_to_target)
        result_angle = (
            0.5 * np.arcsin(np.sin(phi0) * constants.g * distance_to_target * np.cos(phi0) / (config.v0_flywheel ** 2))
            + 0.5 * phi0
        )
        return result_angle

    def update(self) -> float:
        """
        Updates the trajectory calculator with current data.
        """
        self.distance_to_target = self.odometry.getPose().translation().distance(self.speaker)
        self.delta_z = self.speaker_z - self.elevator.get_height()
        self.shoot_angle = self.calculate_angle_no_air(self.distance_to_target, self.delta_z)






    def get_theta(self) -> float:
        """
        Returns the angle of the trajectory.
        """
        return self.phi0

    def deriv(self, t, u):
        x, xdot, z, zdot = u
        speed = np.hypot(xdot, zdot)
        xdotdot = -self.k / constants.m * speed * xdot
        zdotdot = -self.k / constants.m * speed * zdot - constants.g
        return xdot, xdotdot, zdot, zdotdot

# Initial conditions: x0, v0_x, z0, v0_z.
u0 = 0, v0 * np.cos(phi0), 0., v0 * np.sin(phi0)
# Integrate up to tf unless we hit the target sooner.
t0, tf = 0, 60

def hit_target(t, u):
    # We've hit the target if the z-coordinate is 0.
    return desiredX-u[0]
# Stop the integration when we hit the target.
hit_target.terminal = True
# We must be moving downwards (don't stop before we begin moving upwards!)
hit_target.direction = -1

#def max_height(t, u):
#    # The maximum height is obtained when the z-velocity is zero.
#    return u[3]

soln = solve_ivp(deriv, (t0, tf), u0, method='DOP853',dense_output=True,
                 events=(hit_target))
print(soln)
#print('Time to target = {:.2f} s'.format(soln.t_events[0][0]))
#print('Time to highest point = {:.2f} s'.format(soln.t_events[1][0]))

# A fine grid of time points from 0 until impact time.
t = np.linspace(0, soln.t_events[0][0], 100)

# Retrieve the solution for the time grid and plot the trajectory.
sol = soln.sol(t)
x, z = sol[0], sol[2]
print('Final height to target, zfinal = {:.2f} m'.format(z[-1]))
print('Final distance to target, xfinal = {:.2f} m'.format(x[-1]))
print('Initial Angle: {:.2f} degrees'.format(phi0*180/math.pi))
#print('Maximum height, zmax = {:.2f} m'.format(max(z)))
plt.plot(x, z)
#plt.plot()
plt.xlabel('x /m')
plt.ylabel('z /m')
plt.show()