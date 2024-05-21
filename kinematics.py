import pybullet as p
import time
from numpy import random, rad2deg, deg2rad, set_printoptions, array, linalg, size
from numpy.linalg import norm
import argparse

# Set speed to reseti to initial position
RESET_SPEED = 0.05
# Acccuracy (vector distance) to consider the target positon reached
ACCURACY = 15

# Once the coordinate system is fixed and all that, 
# when the code is extracted into a compact version for my project
# that can be called and returns the angles, this will move into global_static_vars
ANGLE_SHIFT_WRIST_Z = 56
ANGLE_SHIFT_WRIST_X = 120

init_pos = {    'r_shoulder_z':-20,
                'r_shoulder_y':70,
                'r_arm_x':30,
                'r_elbow_y':60,
                'r_wrist_z':0,
                'r_wrist_x':0,
                'r_indexfinger_x':-90,
            }

set_printoptions(precision=3)
set_printoptions(suppress=True)

def get_joints_limits(robot_id, num_joints):
        """
        Identify limits, ranges and rest poses of individual robot joints. Uses data from robot model.

        Returns:
            :return [joints_limits_l, joints_limits_u]: (list) Lower and upper limits of all joints
            :return joints_ranges: (list) Ranges of movement of all joints
            :return joints_rest_poses: (list) Rest poses of all joints
        """
        joints_limits_l, joints_limits_u, joints_ranges, joints_rest_poses, joint_names, link_names, joint_indices = [], [], [], [], [], [], []
        for jid in range(num_joints):
            joint_info = p.getJointInfo(robot_id, jid)
            q_index = joint_info[3]
            joint_name = joint_info[1]
            link_name = joint_info[12]
            if q_index > -1: # Fixed joints have q_index -1
                joint_names.append(joint_info[1].decode("utf-8"))
                link_names.append(joint_info[12].decode("utf-8"))
                joint_indices.append(joint_info[0])
                joints_limits_l.append(joint_info[8])
                joints_limits_u.append(joint_info[9])
                joints_ranges.append(joint_info[9] - joint_info[8])
                joints_rest_poses.append((joint_info[9] + joint_info[8])/2)
            if link_name.decode("utf-8") == 'endeffector':
                end_effector_index = jid
            
        return [joints_limits_l, joints_limits_u], joints_ranges, joints_rest_poses, end_effector_index, joint_names, link_names, joint_indices


def speed_control(initial, target, duration):
    speed_to_reach = (abs((float(initial) - float(target)) / float(1260*duration)))
    return speed_to_reach

def spin_simulation(steps):
    for i in range(steps):
        p.stepSimulation()
        time.sleep(0.01
        )

def ik(position,speed=1):

    max_iterations = 100
    residual_threshold = 0.001
    state = []
    finished = False
    success = False 

    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=90, cameraPitch=-40, cameraTargetPosition=[0, 0, 0])

    # Load the URDF robot a create scene
    robot_id = p.loadURDF("./nico_upper_rh6d_r.urdf", [0, 0, 0])
    
    num_joints = p.getNumJoints(robot_id)
    joints_limits, joints_ranges, joints_rest_poses, end_effector_index, joint_names, link_names, joint_indices = get_joints_limits(robot_id, num_joints)

    # Set initial position
    joints_rest_poses = [deg2rad(init_pos[joint]) for joint in joint_names]

    for i in range(len(joint_indices)):
        p.resetJointState(robot_id, joint_indices[i], joints_rest_poses[i])
    spin_simulation(50)

    target_position = position
 
    ik_solution = p.calculateInverseKinematics(
        robot_id,
        end_effector_index,
        target_position,
        lowerLimits=joints_limits[0],
        upperLimits=joints_limits[1],
        jointRanges=joints_ranges,
        restPoses=joints_rest_poses,
        maxNumIterations=max_iterations,
        residualThreshold=residual_threshold
    )

    for j in range(len(joint_indices)):
        p.resetJointState(robot_id, joint_indices[j], ik_solution[j])
    spin_simulation(100)

    #Calculate IK solution error

    (x,y,z), (a,b,c,d),_,_,_,_ = p.getLinkState(robot_id, end_effector_index) 
    print('SimNico final_pos: {}'.format([x,y,z])) 

    IKdiff = (array(target_position) - array([x,y,z]))
    print('SimNico position error: {}'.format(IKdiff))

    print('SimNico final joint angles: {}'.format(rad2deg(array(ik_solution))))  
    
    p.disconnect()
    
    return ik_solution
    
    
def dk(joints, speed=1): #six joints

    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=90, cameraPitch=-40, cameraTargetPosition=[0, 0, 0])

    # Load the URDF robot a create scene
    robot_id = p.loadURDF("./nico_upper_rh6d_r.urdf", [0, 0, 0])
    
    num_joints = p.getNumJoints(robot_id)
    joints_limits, joints_ranges, joints_rest_poses, end_effector_index, joint_names, link_names, joint_indices = get_joints_limits(robot_id, num_joints)

    # Set initial position
    joints_rest_poses = [deg2rad(init_pos[joint]) for joint in joint_names]

    for i in range(len(joint_indices)):
        p.resetJointState(robot_id, joint_indices[i], joints_rest_poses[i])
    spin_simulation(50)

    ik_solution = deg2rad(joints)

    for j in range(len(joint_indices)):
        p.resetJointState(robot_id, joint_indices[j], ik_solution[j])
    spin_simulation(100)
    
    #Calculate IK solution error

    (x,y,z), (a,b,c,d),_,_,_,_ = p.getLinkState(robot_id, end_effector_index) 
    print('SimNico final_pos: {}'.format([x,y,z])) 
    print('SimNico final joint angles: {}'.format(rad2deg(array(ik_solution))))  
    
    p.disconnect()
    
    return (x,y,z)

if __name__ == "__main__":
    #angles0 = [ -25.0, 84.0, 47.0, 94.0, -59.0, 114.0 ] # Carlo-Matilde initial position
    angles0 = [ init_pos[key] for key in init_pos ][:-1]
    print('initial angles:',angles0)
    print('----------------------direct kinematics-------------------------')
    point = dk(angles0)
    print(point)
    print('----------------------inverse kinematics-------------------------')
    angles = ik(point)
    print('----------------------direct kinematics-------------------------')
    point1 = dk(rad2deg(angles))
    print(point1)
    # point1 == point
    print(norm(array(point1)-array(point)),'should be zero') # it should be 0, but it is 0.338m for Carlo-Matilde and 0.173m for Michal's initial position
    