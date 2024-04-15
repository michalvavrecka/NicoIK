import pybullet as p
import time
from numpy import random, rad2deg, deg2rad, set_printoptions
import argparse

from nicomotion.Motion import Motion
from utils.nicodummy import DummyRobot


DEFAULT_SPEED = 0.08
SIMDELAY = 0.5
SIMREALDELAY = 3
RESETDELAY = 4
# Once the coordinate system is fixed and all that, 
# when the code is extracted into a compact version for my project
# that can be called and returns the angles, this will move into global_static_vars
ANGLE_SHIFT_WRIST_Z = 56
ANGLE_SHIFT_WRIST_X = 120
REALJOINTS = ['r_shoulder_z','r_shoulder_y','r_arm_x','r_elbow_y','r_wrist_z','r_wrist_x']
FIXEDJOINTS = ['r_indexfinger_x']

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
                'r_thumb_z':-57.0,
                'r_thumb_x':180.0,
                'r_indexfinger_x':-180.0,
                'r_middlefingers_x':180.0,
                'head_z':0.0,
                'head_y':0.0
            }


def target():
    
    target_position = [0.25+(0.3*random.rand()),0.25+(-0.5*random.rand()), 0.05]  # Write your own method for end effector position here
    #return [0.25, -0.2, 0.15]
    return target_position

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
                joint_names.append(joint_info[1])
                link_names.append(joint_info[12])
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


def get_real_joints(robot,joints):
    last_position= []
    for k in joints:
        actual=robot.getAngle(k)
        print("{} : {}, ".format(k,actual),end="")
        last_position.append(actual)
    print("")
    return last_position

def init_robot():
    motorConfig = './nico_humanoid_upper_rh7d_ukba.json'
    try:
        robot = Motion(motorConfig=motorConfig)
    except:
        robot = DummyRobot()
        print('motors are not operational')
    
    time.sleep(RESETDELAY)
    return robot

def reset_robot(robot, init_pos):
    
    initial_position = get_real_joints(robot,REALJOINTS)
    for k in safe.keys():
        robot.setAngle(k,init_pos[k],DEFAULT_SPEED)
    print ('Robot reseting')
    time.sleep(RESETDELAY)
    final_position = get_real_joints(robot,REALJOINTS)
    #print(initial_position - final_position)
    #input("Press key to continue...")
    return robot, initial_position, final_position

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--position", nargs=3, type=float, help="Target position for the robot end effector as a list of three floats.")
    parser.add_argument("-rr", "--real_robot", action="store_true", help="If set, execute action on real robot.")
    parser.add_argument("-a", "--animate", action="store_true", help="If set, the animation of motion is shown.")
    parser.add_argument("-g", "--gui", action="store_true", help="If set, turn the GUI on")
    parser.add_argument("-l", "--left", action="store_true", help="If set, use left hand IK")
    parser.add_argument("-i", "--initial", action="store_true", help="If set, reset the robot to the initial position after each postion")
    arg_dict = vars(parser.parse_args())
    

    set_printoptions(precision=1)
    set_printoptions(suppress=True)


    # GUI initialization
    if arg_dict["gui"]:
        p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=90, cameraPitch=-40, cameraTargetPosition=[0, 0, 0])
    else:
        p.connect(p.DIRECT)
    
    # Load the URDF robot a create scene
    if arg_dict["left"]:
        robot_id = p.loadURDF("./nico_upper_rh6d_l.urdf", [0, 0, 0])
    else:
        robot_id = p.loadURDF("./nico_upper_rh6d_r.urdf", [0, 0, 0])
    #Create table mesh
    p.createMultiBody(baseVisualShapeIndex=p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[.3,.45,0.02], rgbaColor=[0.6,0.6,0.6,1]),
                          baseCollisionShapeIndex= p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[.3,.45,0.02]), baseMass=0,basePosition=[0.27,0,0.02])
    #Create tablet mesh
    p.createMultiBody(baseVisualShapeIndex=p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[.16,.26,0.01], rgbaColor=[0,0,0.0,1]),
                          baseCollisionShapeIndex= p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[.16,.26,0.01]), baseMass=0,basePosition=[0.41,0,0.035])
    
    num_joints = p.getNumJoints(robot_id)
    joints_limits, joints_ranges, joints_rest_poses, end_effector_index, joint_names, link_names, joint_indices = get_joints_limits(robot_id, num_joints,arg_dict)
    # Custom intital position
    
    #joints_rest_poses = deg2rad([-15, 68, 2.8, 56.4, 0.0, 11.0, -70.0])
    
    # Real robot initialization
    if arg_dict["real_robot"]:
        robot = init_robot()
    else:
        robot = None


    # IK paramenters
    max_iterations = 100
    residual_threshold = 0.001

    # Get the joint indices of the arm
    
    matching_values = {name: init_pos[name] for name in joint_names if name in init_pos}
    print(matching_values)


    while True:
        #Reset robot to initial position
        if arg_dict["initial"]:
            if arg_dict["real_robot"]:
                robot, inpos, finpos = reset_robot(robot,init_pos)

            for i in range(len(joint_indices)):
                p.resetJointState(robot_id, joint_indices[i], joints_rest_poses[i])
        # Target position
        if arg_dict["position"]:
            target_position = arg_dict["position"]
        else:
            target_position = target()
        
        #Create goal dot
        p.createMultiBody(baseVisualShapeIndex=p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.01, rgbaColor=[0,0,1,.7]),
                          baseCollisionShapeIndex= -1, baseMass=0,basePosition=target_position)
        

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
        
        
        if arg_dict["animate"]:
            for i in range(len(joint_indices)):
                p.setJointMotorControl2(robot_id, joint_indices[i], p.POSITION_CONTROL, ik_solution[i])
                
            for _ in range(100):
                # Set joint angles to the IK solution
                time.sleep(.05)
                p.stepSimulation()

        else:
            for i in range(len(joint_indices)):    
                p.resetJointState(robot_id, joint_indices[i], ik_solution[i])
            time.sleep(SIMDELAY)
        
        
        
        
        print(rad2deg(ik_solution))
        
        
        
        
        
        if arg_dict["real_robot"]:
            #Set fingers of hand
            robot.setAngle('r_indexfinger_x', -180.0, DEFAULT_SPEED)
            robot.setAngle('r_middlefingers_x', 180.0, DEFAULT_SPEED)
            robot.setAngle('r_thumb_z', -57.0, DEFAULT_SPEED)
            robot.setAngle('r_thumb_x', 180.0, DEFAULT_SPEED)

            for i,realjoint in enumerate(REALJOINTS):
                degrees = rad2deg(ik_solution[i])
                if realjoint == 'r_wrist_z':
                    degrees += ANGLE_SHIFT_WRIST_Z
                elif realjoint == 'r_wrist_x':
                    degrees += ANGLE_SHIFT_WRIST_X    
                robot.setAngle(realjoint, degrees,DEFAULT_SPEED)
            time.sleep(SIMREALDELAY)
            # Send joint angles to real robot
        
    p.disconnect()

if __name__ == "__main__":
    
    main()
