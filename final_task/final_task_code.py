#python
from sympy import *
import sympy
import numpy as np
import math

l = 0.235   # X-Distance between center of robot and wheel
w = 0.15    # Y-Distance between center of robot and wheel
r = 0.0475  # Wheel radius
# The offset of robot chassis to the block, so that it is in the reachable workspace of the robotic arm
x_offset = -0.01
y_offset = 0.4500
opening = 1

dist_offset = math.sqrt(x_offset**2 + y_offset**2)

# Initializing all the symbols
th1, th2, th3, th4, th5 = sympy.symbols("Theta1, Theta2, Theta3, Theta4, Theta5")
ths = sympy.Matrix([th1, th2, th3, th4, th5])

o = list()
z = list()

pi = 3.141592

# Function to get rotation matrix from the transformation matrix
def get_rot(T):
    return np.array([ [T[0][0], T[0][1], T[0][2]], [T[1][0], T[1][1], T[1][2]], [T[2][0], T[2][1], T[2][2]] ])
    
# Function to get translation vector from the transformation matrix
def get_p_vec(T):
    return np.array([T[0][3], T[1][3], T[2][3] ])

# Function to compute the inverse of a transformation matrix
def get_inv_transform(T):
    rot = get_rot(T)
    rot_t = rot.T
    p_vec = get_p_vec(T)
    p_vec_t = -np.matmul(rot_t, p_vec)
    return np.array([ np.r_[rot_t[0], p_vec_t[0]], np.r_[rot_t[1], p_vec_t[1]], np.r_[rot_t[2], p_vec_t[2]], [0, 0, 0, 1] ])

# Function to compute the wheel velocities from the desired robot chassis velocity
def get_wheel_vel(vx, vy, wz):
    vx_r = vx/r
    vy_r = vy/r
    wz_r = wz/r
    H_chassis = np.array([ [-l-w, 1, -1], [l+w, 1, 1], [l+w, 1, -1], [-l-w, 1, 1] ])
    chassis_vel_r = np.array([wz_r, -vy_r, -vx_r])
    wheel_vels_temp = np.matmul(H_chassis, chassis_vel_r)
    return wheel_vels_temp

# Function to compute the remaining distance as per the current position, destination position and the x,y-offset provided
def get_rem_dist(target_obj, robot_obj):
    target_position = sim.getObjectPosition(target_obj, robot_obj)
    x_diff = target_position[0] - x_offset
    y_diff = target_position[1] - y_offset
    rem_dist = math.sqrt( (x_diff**2) + (y_diff**2) )
    return rem_dist
    
# A helper function to account for the x and y-offset provided
def adjust_chassis_position(target_position):
    adjusted_chassis_tgt = [0,0,0]
    adjusted_chassis_tgt[0] = target_position[0] - x_offset
    adjusted_chassis_tgt[1] = target_position[1] - y_offset
    adjusted_chassis_tgt[2] = target_position[2]
    return adjusted_chassis_tgt

# Function to calculate transformation matrix from DH parameters
def get_transformation_matrix(theta_sym, a_i, alpha_i, d_i):
    T_i = sympy.Matrix([[cos(theta_sym), -cos(alpha_i)*sin(theta_sym), sin(alpha_i)*sin(theta_sym),  a_i*cos(theta_sym)],
                        [sin(theta_sym), cos(alpha_i)*cos(theta_sym), -sin(alpha_i)*cos(theta_sym), a_i*sin(theta_sym)],
                        [0, sin(alpha_i), cos(alpha_i), d_i],
                        [0, 0, 0, 1]])

    return T_i

# Function to get the euler angles from any rotation matrix
def get_angles_from_rot_mat(R):
    th_x = math.atan2( R[2][1], R[2][2])
    th_y = math.atan2(-R[2][0], math.sqrt( R[2][1]**2 + R[2][2]**2 ) )
    th_z = math.atan2( R[1][0], R[0][0])
    return [th_x, th_y, th_z]

# Function to convert linear and angular velocity vectors into X_dot vector of end-effector velocities
def get_ef_vel_vec(lin_vel, ang_vel):
    return [lin_vel[0], lin_vel[1], lin_vel[2], ang_vel[0], ang_vel[1], ang_vel[2]]


# Function to compute the next point in a circular trajectory rotated about x-axis, i.e. in the y-z plane
def get_next_traj_param_x(theta):
    r = 0.3288
    #r = 0.5/math.cos(pi/6)
    th_dot = 0.4
    x_d = 0
    y_d = r*math.cos(theta)*th_dot
    z_d = -r*math.sin(theta)*th_dot
    th_x_d = -th_dot
    th_y_d = 0
    th_z_d = 0
    return [x_d, y_d, z_d, th_x_d, th_y_d, th_z_d]
    
    
# Function to actuate the gripper based on the state of 'opening'
def gripper_actuation():
    global opening
    j1=sim.getObject('./youBotGripperJoint1')
    j2=sim.getObject('./youBotGripperJoint2')
    if opening:
        sim.setJointTargetVelocity(j2,-0.04)
        sim.setJointTargetPosition(j1, sim.getJointPosition(j2)*0.5 + 0.5)
    else:
        sim.setJointTargetVelocity(j2,0.04)
        sim.setJointTargetPosition(j1, sim.getJointPosition(j2)*-0.5 - 0.5)
        
       
    
# sysCall_init has to be called to initiallize the simulation in coppeliaSim
def sysCall_init():
    t = 0

# sysCall_thread is a function the coppeliaSim uses for updating simulation in a threaded manner
def sysCall_thread():
    sim.setThreadAutomaticSwitch(True) # allow automatic switches of threads
    global opening
    
    # getting the gripper joints and setting to initial position
    gripper_joint1 = sim.getObject('./youBotGripperJoint1')
    gripper_joint2 = sim.getObject('./youBotGripperJoint2')
    sim.setJointTargetPosition(gripper_joint1, 0.5)

    # getting the robot base frame and target objects
    robot_obj = sim.getObject('./youBot_ref')
    target_obj = sim.getObject('/target_obj')
    final_obj = sim.getObject('/final_obj')
    
    # getting the target position w.r.t. the current position of robot and the target object (cube)
    target_position = sim.getObjectPosition(target_obj, robot_obj)
    target_position = adjust_chassis_position(target_position)
    target_orientation = sim.getObjectOrientation(target_obj, robot_obj)

    wheel_joints = {}
    arm_joints = {}
    
    # Getting the wheel joint objects
    wheel_joints[0]=sim.getObject('./rollingJoint_rl')
    wheel_joints[1]=sim.getObject('./rollingJoint_rr')
    wheel_joints[2]=sim.getObject('./rollingJoint_fr')
    wheel_joints[3]=sim.getObject('./rollingJoint_fl')
    
    # Initializing joint angles to very small angles to avoid singularities while computing inverse of Jacobian
    q_s = np.array([0.0001, 0.0001, 0.0001, 0.0001, 0.0001])

    # Getting the arm joints of the robot
    for i in range(5):
        arm_joints[i]=sim.getObject('./youBotArmJoint' + str(i))
        


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

    print("target positoin is "+ str(target_position))
    print("target orientation is " + str(target_orientation))
    
    # Computing the necessary components for translation of robot's chassis towards the block
    dt = 0.1
    path1_time = 8
    t = 0
    x_diff = target_position[0]
    y_diff = target_position[1]
    angle_diff = target_orientation[2]
    v_x = x_diff/path1_time
    v_y = y_diff/path1_time
    w_z = angle_diff/path1_time

    total_dist = math.sqrt( (x_diff**2) + (y_diff**2) )

    wheel_vels = get_wheel_vel(v_x, v_y, w_z)
    
    # Setting the robotic arm to home configuration
    for i in range(5):
        sim.setJointTargetPosition(arm_joints[i], q_s[i])

    # Setting the wheel velocities as computed above
    for i in range(4):
        sim.setJointTargetVelocity(wheel_joints[i], wheel_vels[i])

    total_dist = get_rem_dist(target_obj, robot_obj)
    
    # Keeping the robot going until the robot is close enough to the block
    while (total_dist > 0.04 and t < 15):
        total_dist = get_rem_dist(target_obj, robot_obj)
        #print(t)
        sim.wait(0.1)
        t += 0.1
        
    print("Destination reached!")

    # Setting wheel velocities to zero when robot is at the desired position
    for i in range(4):
        sim.setJointTargetVelocity(wheel_joints[i], 0)

    
    target_position = sim.getObjectPosition(target_obj, robot_obj)
    target_orientation = sim.getObjectOrientation(target_obj, robot_obj)
    ef_init_transform = eval_T_ef(q_s[0], q_s[1], q_s[2], q_s[3], q_s[4])
    
    t = 0
    th_dot_traj = 0.4
    t=0
    dt = 0.05
    th_traj = 0
    
    # Rotate arm to configure
    while (t < 20 and th_traj < 2*pi/3):
        J_curr = eval_J(q_s[0], q_s[1], q_s[2], q_s[3], q_s[4])
        J_curr_pinv = np.linalg.pinv(J_curr)
        X_dot = get_next_traj_param_x(th_traj)
        q_s_dot = np.matmul(J_curr_pinv, X_dot) 
        for i in range(5):
            sim.setJointTargetPosition(arm_joints[i], q_s[i])
        q_s = q_s + q_s_dot*dt
        t += dt
        th_traj += th_dot_traj*dt
        sim.wait(0.05)
        
    
    sim.wait(2)
    print("Gripper orientation complete")
    
    # setting trajectory parameters to move arm along -z axis
    z_dot_traj = -0.1
    t=0
    dt = 0.05
    z_traj = 0
    
    # Move arm to grip block, move along z-axis
    while (t < 2.25 and z_traj > -0.211):
        J_curr = eval_J(q_s[0], q_s[1], q_s[2], q_s[3], q_s[4])
        J_curr_pinv = np.linalg.pinv(J_curr)
        X_dot = np.array([0, 0, z_dot_traj, 0, 0, -0.42])
        q_s_dot = np.matmul(J_curr_pinv, X_dot)
        for i in range(5):
            sim.setJointTargetPosition(arm_joints[i], q_s[i])
        q_s = q_s + q_s_dot*dt
        t += dt
        z_traj += z_dot_traj*dt
        sim.wait(0.05)

    sim.wait(1)
    
    t = 0
    dt = 0.1
    
    # Gripping the block
    opening = 0
    gripper_actuation()
    sim.wait(3)
    
    # setting trajectory parameters to move arm along -z axis
    z_dot_traj = 0.1
    t=0
    dt = 0.05
    z_traj = 0
    
    # Move arm with griped block, move along +z-axis
    while (t < 5 and z_traj < 0.10):
        J_curr = eval_J(q_s[0], q_s[1], q_s[2], q_s[3], q_s[4])
        J_curr_pinv = np.linalg.pinv(J_curr)
        X_dot = np.array([0, 0, z_dot_traj, 0, 0, 0])
        q_s_dot = np.matmul(J_curr_pinv, X_dot)
        for i in range(5):
            sim.setJointTargetPosition(arm_joints[i], q_s[i])
        q_s = q_s + q_s_dot*dt
        t += dt
        z_traj += z_dot_traj*dt
        sim.wait(0.05)
        
        
    # Computing the necessary components for translation of robot's chassis towards the block
    dt = 0.1
    path1_time = 8
    t = 0
    final_position = sim.getObjectPosition(final_obj, robot_obj)
    final_position = adjust_chassis_position(final_position)
    final_orientation = sim.getObjectOrientation(final_obj, robot_obj)

    x_diff = final_position[0]
    y_diff = final_position[1]
    angle_diff = final_orientation[2]
    v_x = x_diff/path1_time
    v_y = y_diff/path1_time
    w_z = angle_diff/path1_time

    total_dist = math.sqrt( (x_diff**2) + (y_diff**2) )

    wheel_vels = get_wheel_vel(v_x, v_y, w_z)

    # Setting the wheel velocities as computed above
    for i in range(4):
        sim.setJointTargetVelocity(wheel_joints[i], wheel_vels[i])

    total_dist = get_rem_dist(final_obj, robot_obj)
    print("total_dist " + str(total_dist))
    
    # Keeping the robot going until the robot is close enough to the block
    while (total_dist > 0.1 and t < 15):
        total_dist = get_rem_dist(final_obj, robot_obj)
        print(total_dist)
        sim.wait(0.1)
        t += 0.1
    
    for i in range(4):
        sim.setJointTargetVelocity(wheel_joints[i], 0)
        

    
    
    # setting trajectory parameters to move arm along z axis
    z_dot_traj = -0.1
    t=0
    dt = 0.05
    z_traj = 0
    
    # Move arm with griped block, move along +z-axis
    while (t < 2.5 and z_traj > -0.10):
        J_curr = eval_J(q_s[0], q_s[1], q_s[2], q_s[3], q_s[4])
        J_curr_pinv = np.linalg.pinv(J_curr)
        X_dot = np.array([0, 0, z_dot_traj, 0, 0, -0.42])
        q_s_dot = np.matmul(J_curr_pinv, X_dot)
        for i in range(5):
            sim.setJointTargetPosition(arm_joints[i], q_s[i])
        q_s = q_s + q_s_dot*dt
        t += dt
        z_traj += z_dot_traj*dt
        sim.wait(dt)
        
    sim.wait(1)
    # Releasing the block
    opening = 1
    gripper_actuation()
    sim.wait(3)
    
    # setting trajectory parameters to move arm along z axis
    z_dot_traj = 0.1
    t=0
    dt = 0.05
    z_traj = 0
    
    # Move arm with griped block, move along +z-axis
    while (t < 2.5 and z_traj < 0.10):
        J_curr = eval_J(q_s[0], q_s[1], q_s[2], q_s[3], q_s[4])
        J_curr_pinv = np.linalg.pinv(J_curr)
        X_dot = np.array([0, 0, z_dot_traj, 0, 0, 0])
        q_s_dot = np.matmul(J_curr_pinv, X_dot)
        for i in range(5):
            sim.setJointTargetPosition(arm_joints[i], q_s[i])
        q_s = q_s + q_s_dot*dt
        t += dt
        z_traj += z_dot_traj*dt
        sim.wait(dt)
        
    print("Task done!")
    
    sim.pauseSimulation()