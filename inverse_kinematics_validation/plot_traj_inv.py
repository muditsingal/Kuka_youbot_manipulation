from sympy import *
import sympy
import numpy as np
import math
import matplotlib.pyplot as plt


l = 0.235   # X-Distance between center of robot and wheel
w = 0.15    # Y-Distance between center of robot and wheel
r = 0.0475  # Wheel radius

# Initializing all the symbols
th1, th2, th3, th4, th5 = sympy.symbols("Theta1, Theta2, Theta3, Theta4, Theta5")
ths = sympy.Matrix([th1, th2, th3, th4, th5])

o = list()
z = list()
x_s = []
y_s = []
z_s = []



pi = 3.141592

# Function to compute the wheel velocities from the desired robot chassis velocity
def get_wheel_vel(vx, vy, wz):
    vx_r = vx/r
    vy_r = vy/r
    wz_r = wz/r
    H_chassis = np.array([ [-l-w, 1, -1], [l+w, 1, 1], [l+w, 1, -1], [-l-w, 1, 1] ])
    chassis_vel_r = np.array([wz_r, -vy_r, -vx_r])
    wheel_vels_temp = np.matmul(H_chassis, chassis_vel_r)
    return wheel_vels_temp


# Function to calculate transformation matrix from DH parameters
def get_transformation_matrix(theta_sym, a_i, alpha_i, d_i):
    T_i = sympy.Matrix([[cos(theta_sym), -cos(alpha_i)*sin(theta_sym), sin(alpha_i)*sin(theta_sym),  a_i*cos(theta_sym)],
                        [sin(theta_sym), cos(alpha_i)*cos(theta_sym), -sin(alpha_i)*cos(theta_sym), a_i*sin(theta_sym)],
                        [0, sin(alpha_i), cos(alpha_i), d_i],
                        [0, 0, 0, 1]])

    return T_i

# Function to compute the next point in a circular trajectory rotated about x-axis, i.e. in the y-z plane
def get_next_traj_param_x(theta):
    r = 0.18
    #r = 0.5/math.cos(pi/6)
    th_dot = 0.4
    x_d = 0
    y_d = r*math.cos(theta)*th_dot
    z_d = -r*math.sin(theta)*th_dot
    th_x_d = -th_dot
    th_y_d = 0
    th_z_d = 0
    return [x_d, y_d, z_d, th_x_d, th_y_d, th_z_d]
    

# Initializing joint angles to very small angles to avoid singularities while computing inverse of Jacobian
q_s = np.array([0.0001, 0.0001, 0.0001, 0.0001, 0.0001])


# Computing individual transformation matrices
T1_0 = get_transformation_matrix(- pi/2, -0.1662, 0, 0)

T2_1 = get_transformation_matrix(th1, -0.0331, -pi/2, 0.2454)

T3_2 = get_transformation_matrix(th2 + pi/2, -0.155, 0, 0)

T4_3 = get_transformation_matrix(th3, -0.1349, 0, 0)

T5_4 = get_transformation_matrix(th4 + pi/2, 0, -pi/2, 0)

T6_5 = get_transformation_matrix(th5, 0, 0, 0.1939)

# Multiplying transformation matrices to get end effector transformation w.r.t. base frame
T2_0 = T1_0*T2_1
T3_0 = T2_0*T3_2
T4_0 = T3_0*T4_3
T5_0 = T4_0*T5_4
T6_0 = T5_0*T6_5

# Function to make an evaluatable function to get end-effector transformation matrix given joint angles
eval_T_ef = lambdify(ths, T6_0, "numpy")

# Getting the translation components from transformation matrices
o.append([0,0,0])                           # o_0
o.append([T1_0[3], T1_0[4+3], T1_0[8+3]])   # o_1
o.append([T2_0[3], T2_0[4+3], T2_0[8+3]])   # o_2
o.append([T4_0[3], T4_0[4+3], T4_0[8+3]])   # o_3
o.append([T5_0[3], T5_0[4+3], T5_0[8+3]])   # o_4
o.append([T6_0[3], T6_0[4+3], T6_0[8+3]])   # o_5 or end effector frame's origin

o_6 = [T6_0[3], T6_0[4+3], T6_0[8+3]]       # End effector frame's origin

# Getting the z-axes from the transformation matrices
z.append([0,0,1])                           # z_0
z.append([T1_0[2], T1_0[4+2], T1_0[8+2]])   # z_1
z.append([T2_0[2], T2_0[4+2], T2_0[8+2]])   # z_2
z.append([T4_0[2], T4_0[4+2], T4_0[8+2]])   # z_3
z.append([T5_0[2], T5_0[4+2], T5_0[8+2]])   # z_4
z.append([T6_0[2], T6_0[4+2], T6_0[8+2]])   # z_5 or end effector frame's z-axis

# Initializing the Jacobian matrix and then assigning the partial derivatives to the J_v part of J
J = sympy.zeros(6,5)
for i in range(5):
    J[i] = o_6[0].diff(ths[i])
    J[i+5] = o_6[1].diff(ths[i])
    J[i+10] = o_6[2].diff(ths[i])

# Assigning the J_w part of J
for i in range(5):
    J[i+15] = z[i][0]
    J[i+20] = z[i][1]
    J[i+25] = z[i][2]

eval_J = lambdify(ths, J, "numpy")

q_s[1] = -pi/6
q_s[2] = -pi/6
q_s[3] = -pi/6
    

z_dot_traj = -0.05
t=0
dt = 0.05
z_traj = 0



# Move arm to grip block, move along z-axis
while (t < 3 and z_traj > -0.211):
    J_curr = eval_J(q_s[0], q_s[1], q_s[2], q_s[3], q_s[4])
    J_curr_pinv = np.linalg.pinv(J_curr)
    X_dot = np.array([0, 0, z_dot_traj, 0, 0, 0])
    q_s_dot = np.matmul(J_curr_pinv, X_dot)

    T_ef_curr = eval_T_ef(q_s[0], q_s[1], q_s[2], q_s[3], q_s[4])
    x_s.append(T_ef_curr[0][3])
    y_s.append(T_ef_curr[1][3])
    z_s.append(T_ef_curr[2][3])

    q_s = q_s + q_s_dot*dt
    t += dt
    z_traj += z_dot_traj*dt


# Plotting the x, y, and z points traced by the end-effector
ax = plt.axes(projection='3d')
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
ax.set_xlim(-0.2,0.2)
ax.set_ylim(0.4,0.8)


ax.scatter3D(x_s, y_s, z_s)

plt.show()