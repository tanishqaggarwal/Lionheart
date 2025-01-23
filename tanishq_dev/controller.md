=main control algorithm=
commanded position -> rate limit, accel limit -> desired position
desired position - sensed position = pos error -> P controller -> desired velocity
desired velocity - sensed velocity = vel err -> PI controller -> desired force/torque (depending on application)

text on attitude control: Spacecraft Attitude Dynamics and Control

=translation= : 3 main control (x, y, z)
=rotation= : 3 main control (r, p, y)

to translate desired force and torque: convert to least-squares problem on thrust

A1 (t1, t2, t3, ... t6) = (f1, f2, f3)
A2 (m1, m2, m3, ... m6) = (tau1, tau2, tau3)

minimize ||(f1, f2, f3, tau1, tau2, tau3) - (f1c, f2c, f3c, tau1c, tau2c, tau3c)||_2

m1 = r1 x t1 => m1 = R1 t1