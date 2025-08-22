import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint, quad

# Mass
# Assume ~2lbs = 0.907kg
m = 1

# Gravity
g = 9.8

# Density of water
rho = 1000

# Volume
# (0.0762 (3in) / 2)^2 * pi * 0.3048 (12in)
v = 0.001389

# Drag
b = 50

Kp = 10   # Increased proportional gain
Ki = 1   # Keep integral gain
Kd = 1   # Increased derivative gain

Zref = -5
Zstart = 0

# Define the differential equation
def model(y, t):
    z, vel, integral_error = y
    error = Zref - z
    # Integral term accumulation
    integral_dot = error
    
    # Control force (PID controller)
    # For depth control: positive control force means upward thrust
    control_force = Kp * error + Ki * integral_error + Kd * vel
    
    # Dynamics equation (corrected physics)
    # Gravity pulls down (-), buoyancy pushes up (+), drag opposes motion (+)
    accel = (-g + rho * v * g - b * vel + control_force) / m
    
    dydt = [vel, accel, integral_dot]
    return dydt

# Initial condition and time points
y0 = [Zstart, 0, 0]
t = np.linspace(0, 100, 1000)

# Solve the ODE
solution = odeint(model, y0, t)

# Plot the solution
plt.figure(figsize=(12, 4))

plt.subplot(1, 3, 1)
plt.plot(t, solution[:, 0])
plt.xlabel('Time')
plt.ylabel('Position (z)')
plt.title('Position vs Time')
plt.grid(True)
plt.axhline(y=Zref, color='r', linestyle='--', label='Reference')
plt.axhline(y=(Zref - Zstart)*0.632, color='b', linestyle='-.', label='63.2%')
plt.legend()

plt.subplot(1, 3, 2)
plt.plot(t, solution[:, 1])
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.title('Velocity vs Time')
plt.grid(True)

plt.subplot(1, 3, 3)
plt.plot(t, solution[:, 2])
plt.xlabel('Time')
plt.ylabel('Integral Error')
plt.title('Integral Error vs Time')
plt.grid(True)

plt.tight_layout()
plt.show()
