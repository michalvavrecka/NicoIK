import pybullet as p
import time
from numpy import random, rad2deg, deg2rad, set_printoptions, array, linalg
import argparse
import matplotlib.pyplot as plt
import pandas as pd

#from TouchAgent import TouchAgent

# Set speed to reseti to initial position
RESET_SPEED = 0.05
# Acccuracy (vector distance) to consider the target positon reached
ACCURACY = 10


# Once the coordinate system is fixed and all that, 
# when the code is extracted into a compact version for my project
# that can be called and returns the angles, this will move into global_static_vars
ANGLE_SHIFT_WRIST_Z = 56
ANGLE_SHIFT_WRIST_X = 120

init_pos = { # standard position
                'l_shoulder_z':0.0,
                'l_shoulder_y':0.0,
                'l_arm_x':0.0,
                'l_elbow_y':89.0,
                'l_wrist_z':0.0,
                'l_wrist_x':-56.0,
                'l_thumb_z':-57.0,
                'l_thumb_x':-180.0,
                'l_indexfinger_x':-180.0,
                'l_middlefingers_x':-180.0,
                'r_shoulder_z':-20,
                'r_shoulder_y':70,
                'r_arm_x':30,
                'r_elbow_y':60,
                'r_wrist_z':0,
                'r_wrist_x':0,
                'r_thumb_z':-180.0,
                'r_thumb_x':-180.0,
                'r_indexfinger_x':-90,
                'r_middlefingers_x':180.0,
                'head_z':0.0,
                'head_y':0.0
            }

set_printoptions(precision=3)
set_printoptions(suppress=True)


def target_random():
    
    target_position = [0.30+(0.25*random.rand()),0.0+(-0.25*random.rand()), 0.08]  # Write your own method for end effector position here
    #return [0.25, -0.2, 0.15]
    return target_position



def target_joints(index):
    
    calibration_matrix =   [[ 27.502, 90, 34.999, 114.998, 90.,-22.5],
                            [ 16.997,  79.502,  29.449, 125.503,  89.764, -32.994],
                            [ 10.99,   68.98,   29.452, 136.007,  89.762, -40.419],
                            [ 10.98,   58.476,  29.439, 138.99,   89.761, -40.423],
                            [ 10.98,   47.733,  29.438, 138.991,  89.761, -40.423]]

    if index >= len(calibration_matrix):
        index = 1
    return calibration_matrix[index]

def target_experiment(index):
    
    calibration_matrix =   [[0.45, -.05, 0.062],
                            [0.38, -0.0, 0.042],
                            [0.50, 0.05, 0.083],
                            [0.65, 0.03, 0.085],
                            [0.58, -.08, 0.09],
                            [0.43, -.14, 0.06],
                            [0.36, -.075, 0.035]]
    
    if index >= len(calibration_matrix):
        index = 1
    return calibration_matrix[index]  

def get_joints_limits(robot_id, num_joints,arg_dict):
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
            if arg_dict["left"]:
                if link_name.decode("utf-8") == 'endeffectol':
                    end_effector_index = jid
            else:
                if link_name.decode("utf-8") == 'endeffector':
                #if link_name.decode("utf-8") == 'endeffector':
                    end_effector_index = jid
            
        return [joints_limits_l, joints_limits_u], joints_ranges, joints_rest_poses, end_effector_index, joint_names, link_names, joint_indices


def match_joints(init_pos, joint_names):
    actuated_joint_names = []
    actuated_joint_init_pos = []
    for name in init_pos.keys():
        if name in joint_names:
            actuated_joint_names.append((name))
            actuated_joint_init_pos.append(init_pos[name])

    return actuated_joint_names, actuated_joint_init_pos


def speed_control(initial, target, duration):
    
        speed_to_reach = (abs((float(initial) - float(target)) / float(1260*duration)))
        return speed_to_reach

def spin_simulation(steps):
    for i in range(steps):
        p.stepSimulation()
        time.sleep(0.01
        )

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--position", nargs=3, type=float, help="Target position for the robot end effector as a list of three floats.")
    parser.add_argument("-j", "--joints", nargs=6, type=float, help="Target joint angles for the robot end effector as a list of six floats.")
    parser.add_argument("-f", "--file", type=str, help="Target position for the robot end effector as a list of three floats.")
    parser.add_argument("-l", "--left", action="store_true", help="If set, use left hand IK")
    parser.add_argument("-i", "--initial", action="store_true", help="If set, reset the robot to the initial position after each postion")
    parser.add_argument("-t", "--trajectory", action="store_true", help="Store coordinates from simulation into text file")
    parser.add_argument("-e", "--experiment", action="store_true", help="If set, execute experiments positions")
    parser.add_argument("-s", "--speed", type=float, default = 1, help="Speed of arm movement in simulator")
    parser.add_argument("-d", "--duration", type=float, default = 1, help="Duration of movement in si/real robot")
    parser.add_argument("-ts", "--trajectory_steps", type=int, default=5, help="Number of steps in each trajectory")
    arg_dict = vars(parser.parse_args())


    p.connect(p.DIRECT)
    
    # Load the URDF robot a create scene
    if arg_dict["left"]:
        robot_id = p.loadURDF("./nico_upper_rh6d_l.urdf", [0, 0, 0])
    else:
        robot_id = p.loadURDF("./nico_upper_rh6d_r.urdf", [0, 0, 0])
    
    num_joints = p.getNumJoints(robot_id)
    joints_limits, joints_ranges, joints_rest_poses, end_effector_index, joint_names, link_names, joint_indices = get_joints_limits(robot_id, num_joints,arg_dict)

    for i in range(len(joint_indices)):
        p.resetJointState(robot_id, joint_indices[i], joints_rest_poses[i])
    spin_simulation(50)

    # IK paramenters
    max_iterations = 100
    residual_threshold = 0.001

    # Statistics
    state = []
    finished = False
    
    #if arg_dict["file"]:
    #    open(arg_dict["file"]) as data


    for i in range(30):
    #while True: 
        # Target position
        if arg_dict["position"]:
            target_position = arg_dict["position"]
        elif arg_dict["experiment"]:
            target_position = target_experiment(i)
        elif arg_dict["joints"]:
            target_position = target_joints(i)
        else:
            target_position = target_random()

        #Reset robot to initial position
        if arg_dict["initial"]:
            for j in range(len(joint_indices)):
                p.resetJointState(robot_id, joint_indices[j], joints_rest_poses[j])
            spin_simulation(20)
        
        #target_orientation = target_position + [1]
        # Perform IK
        #ik_solution = p.calculateInverseKinematics(robot_id, end_effector_index, target_position,
        #                                           targetOrientation=target_orientation,
        #                                        maxNumIterations=max_iterations,
        #                                        residualThreshold=residual_threshold)        
        ik_solution = p.calculateInverseKinematics(robot_id,
                                                       end_effector_index,
                                                       target_position,
                                                       lowerLimits=joints_limits[0],
                                                       upperLimits=joints_limits[1],
                                                       jointRanges=joints_ranges,
                                                       restPoses=joints_rest_poses,
                                                       maxNumIterations=max_iterations,
                                                       residualThreshold=residual_threshold)

        trajectory = []
        
        if arg_dict["trajectory"]:
            for j in range(len(joint_indices)):
                                    
                p.setJointMotorControl2(robot_id, joint_indices[j],
                                        p.POSITION_CONTROL, ik_solution[j],
                                        maxVelocity=arg_dict["speed"],
                                        force=500,
                                        positionGain=0.7,
                                        velocityGain=0.3)
            step = 1
            while not finished:
                for j in range(len(joint_indices)):
                    state.append(p.getJointState(robot_id,joint_indices[j])[0])
                simdiff = rad2deg(array(ik_solution)) - rad2deg(array(state))
                print('SimNICO, Step: {}, Position: {}'.format(step, ['{:.2f}'.format(diff) for diff in rad2deg(array(state))],end='\r')) 
                if linalg.norm(simdiff) <= ACCURACY:
                    finished = True
                if arg_dict["trajectory"]:
                    trajectory.append(state)
                spin_simulation(1)
                state = []
                step += 1
            #SAVE TRAJECTORY AS TEXT FILE

            filename = 'trajectories/trajectory_'+str(i)+'.txt'
            with open(filename, 'w') as f:
                steps = arg_dict["trajectory_steps"]
                i_dif = (len(trajectory) - 1) / (steps - 1)
                for j in range(steps):
                    if int(j*i_dif) != int((j-1)*i_dif):
                        f.write("%s\n" % rad2deg(trajectory[int(j*i_dif)]))

            finished = False

        else:
            for j in range(len(joint_indices)):
                p.resetJointState(robot_id, joint_indices[j], ik_solution[j])
            spin_simulation(100)

        
        #Calculate IK solution error

        (x,y,z), (a,b,c,d),_,_,_,_ = p.getLinkState(robot_id, end_effector_index) 
        IKdiff = (array(target_position) - array([x,y,z]))
        print('SimNico target_pos: {}'.format(target_position)) 
        print('SimNico IK error: {}'.format(IKdiff))
        print('SimNico joint angles: {}'.format(rad2deg(array(ik_solution))))  
        

    p.disconnect()


if __name__ == "__main__":
    main()
