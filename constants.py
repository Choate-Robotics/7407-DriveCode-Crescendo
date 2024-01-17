from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Translation2d
c = 0.47 # drag coefficient
a = 14*0.0254*2*0.0254 # projectile volume (m^3)
m = 0.235301 # projectile mass (kg)
rho_air = 1.28 # air density (kg/m^3)
g = 9.8 # acceleration due to gravity (m/s^2)
speaker_z = 1.7 # height of target (m) CHANGE THIS
speaker_location = Translation2d(0, 0)