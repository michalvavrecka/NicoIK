import pybullet as p
import time
from numpy import random, rad2deg, deg2rad, set_printoptions, array, linalg
import argparse
import matplotlib.pyplot as plt
import pandas as pd

from nicomotion.Motion import Motion
from utils.nicodummy import DummyRobot


# Set speed to reseti to initial position
RESET_SPEED = 0.01
# Acccuracy (vector distance) to consider the target positon reached
ACCURACY = 5


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
                'r_indexfinger_x':0,
                'r_middlefingers_x':180.0,
                'head_z':0.0,
                'head_y':0.0
            }

set_printoptions(precision=3)
set_printoptions(suppress=True)


def target():
    
    target_position = [0.30+(0.25*random.rand()),0.0+(-0.25*random.rand()), 0.08]  # Write your own method for end effector position here
    #return [0.25, -0.2, 0.15]
    return target_position

def target_calibration(index):
    
    calibration_matrix =   [[0.245, -0.260, 0.043],
                            [0.305, -0.260, 0.043],
                            [0.365, -0.260, 0.043],
                            [0.365, -0.130, 0.043],
                            [0.365, 0.000, 0.043],
                            [0.305, 0.00, 0.043],
                            [0.245, 0.00, 0.043],
                            [0.245, -0.130, 0.043],
                            [0.245, -0.260, 0.043]]
    
    if index >= len(calibration_matrix):
        exit()
                          # Write your own method for end effector position here
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


def get_real_joints(robot,joints):
    
    last_position= []
    
    for k in joints:
        actual=robot.getAngle(k)
        #print("{} : {}, ".format(k,actual),end="")
        last_position.append(actual)
    #print("")
    
    return last_position

def match_joints(init_pos, joint_names):
    actuated_joint_names = []
    actuated_joint_init_pos = []
    for name in init_pos.keys():
        if name in joint_names:
            actuated_joint_names.append((name))
            actuated_joint_init_pos.append(init_pos[name])

    return actuated_joint_names, actuated_joint_init_pos

def init_robot():
    motorConfig = './nico_humanoid_upper_rh7d_ukba.json'
    try:
        robot = Motion(motorConfig=motorConfig)
        print('Robot initialized')
    except:
        robot = DummyRobot()
        print('Motors are not operational')
    
    return robot

def reset_robot(robot, init_pos):

    for k in init_pos.keys():
        robot.setAngle(k,init_pos[k],RESET_SPEED)

    return robot

def reset_actuated(robot, actuated_joints, actuated_initpos):

    for joint, initpos in zip(actuated_joints, actuated_initpos):
        robot.setAngle(joint,initpos,RESET_SPEED)    
    return robot

def speed_control(initial, target, duration):
    
        speed_to_reach = (abs((float(initial) - float(target)) / float(1260*duration)))
        return speed_to_reach

def spin_simulation(steps):
    for i in range(steps):
        p.stepSimulation()
        time.sleep(0.01
        )

def check_execution (robot, joints, target):
    tic = time.time()
    distance = 100

    while distance > ACCURACY:
        actual = get_real_joints(robot,joints)
        #print(timestamp)
        diff = array(target) - array(actual)
        distance = linalg.norm(diff)
        print('Duration: {:.2f}, Error: {:.2f}'.format(time.time()-tic,distance), end='\r')
        time.sleep(0.1)
    toc = time.time()
    return toc-tic



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--position", nargs=3, type=float, help="Target position for the robot end effector as a list of three floats.")
    parser.add_argument("-rr", "--real_robot", action="store_true", help="If set, execute action on real robot.")
    parser.add_argument("-a", "--animate", action="store_true", help="If set, the animation of motion is shown.")
    parser.add_argument("-g", "--gui", action="store_true", help="If set, turn the GUI on")
    parser.add_argument("-l", "--left", action="store_true", help="If set, use left hand IK")
    parser.add_argument("-i", "--initial", action="store_true", help="If set, reset the robot to the initial position after each postion")
    parser.add_argument("-t", "--trajectory", action="store_true", help="Store coordinates from simulation into text file")
    parser.add_argument("-c", "--calibration", action="store_true", help="If set, execute calibration positions")
    parser.add_argument("-s", "--speed", type=float, default = 1, help="Speed of arm movement in simulator")
    parser.add_argument("-d", "--duration", type=float, default = 1, help="Duration of movement in si/real robot")
    arg_dict = vars(parser.parse_args())

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
    
    actuated_joints,actuated_initpos = match_joints(init_pos,joint_names)


    # Real robot initialization and setting all joints
    if arg_dict["real_robot"]:
        robot = init_robot()
        robot = reset_robot(robot,init_pos)
        time = check_execution(robot, init_pos.keys(), list(init_pos.values()))
        print ('Robot reset in {:.2f} seconds.'.format(time))  
        actual_position = get_real_joints(robot,actuated_joints)
        for i in range(len(joint_indices)):
            p.resetJointState(robot_id, joint_indices[i], deg2rad(actual_position[i]))
        spin_simulation(50)

    else:
        for i in range(len(joint_indices)):
            p.resetJointState(robot_id, joint_indices[i], joints_rest_poses[i])
        spin_simulation(50)

    # IK paramenters
    max_iterations = 100
    residual_threshold = 0.001

    # Statistics
    IKstat=[]
    JointIstat=[]
    TimeIstat=[]
    JointGstat=[]
    TimeGstat=[]
    state = []
    trajectory = []
    finished = False
    
    for i in range(30):
    #while True: 
        # Target position
        if arg_dict["position"]:
            target_position = arg_dict["position"]
        elif arg_dict["calibration"]:
            target_position = target_calibration(i)
        else:
            target_position = target()
        
        #Create goal dot
        p.createMultiBody(baseVisualShapeIndex=p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.01, rgbaColor=[0,0,1,.7]),
                          baseCollisionShapeIndex= -1, baseMass=0,basePosition=target_position)
        

        #Reset robot to initial position
        if arg_dict["initial"]:
            if arg_dict["real_robot"]:
                robot = reset_actuated(robot,actuated_joints,actuated_initpos)
                time = check_execution(robot, actuated_joints, actuated_initpos)
                reset_pos = get_real_joints(robot,actuated_joints)
                difference = array(actuated_initpos) - array(reset_pos)
                print('RealNICO init: {:.2f}s, Error: {}'.format(time, ['{:.2f}'.format(diff) for diff in difference])) 
                JointIstat.append(difference)
                TimeIstat.append(time)
            for i in range(len(joint_indices)):
                p.resetJointState(robot_id, joint_indices[i], joints_rest_poses[i])
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
        
        
        if arg_dict["animate"]:
            for i in range(len(joint_indices)):
                                    
                p.setJointMotorControl2(robot_id, joint_indices[i],
                                        p.POSITION_CONTROL, ik_solution[i],
                                        maxVelocity=arg_dict["speed"],
                                        positionGain=0.7,
                                        velocityGain=0.3)
            step = 1
            while not finished:
                for i in range(len(joint_indices)):
                        state.append(p.getJointState(robot_id,joint_indices[i])[0])
                simdiff = rad2deg(array(ik_solution)) - rad2deg(array(state))
                print('SimNICO, Step: {}, Error: {}'.format(step, ['{:.2f}'.format(diff) for diff in simdiff])) 
                if linalg.norm(simdiff) <= ACCURACY:
                    finished = True
                if arg_dict["trajectory"]:
                    trajectory.append(state)
                spin_simulation(1)
                state = []
                step += 1
            #sAVE TRAJECTORY AS TEXT FILE 
            if arg_dict["trajectory"]:
                filename = 'trajectory_'+str(i)+'.txt'
                with open(filename, 'w') as f:
                    for item in trajectory:
                        f.write("%s\n" % item)

            finished = False

        else:
            for i in range(len(joint_indices)):    
                p.resetJointState(robot_id, joint_indices[i], ik_solution[i])
            spin_simulation(100)

        
        #Calculate IK solution error

        (x,y,z), (a,b,c,d),_,_,_,_ = p.getLinkState(robot_id, end_effector_index) 
        IKdiff = (array(target_position) - array([x,y,z]))
        print('SimNico target_pos: {}'.format(target_position)) 
        print('SimNico IK error: {}'.format(IKdiff)) 
        IKstat.append(IKdiff)
        
        if arg_dict["real_robot"]:
            
            targetdeg = []

            for i,realjoint in enumerate(actuated_joints):
                degrees = rad2deg(ik_solution[i])
                speed = speed_control(actual_position[i], degrees, arg_dict["duration"])
                if realjoint == 'r_wrist_z':
                    degrees += ANGLE_SHIFT_WRIST_Z
                elif realjoint == 'r_wrist_x':
                    degrees += ANGLE_SHIFT_WRIST_X  
                robot.setAngle(realjoint, degrees,speed)
                targetdeg.append(degrees)

            time = check_execution(robot, actuated_joints, targetdeg)     
            final_pos = get_real_joints(robot,actuated_joints)
            difference = array(targetdeg) - array(final_pos)
            print('RealNICO goal: {:.2f}s, Error: {}'.format(time, ['{:.2f}'.format(diff) for diff in difference])) 
            JointGstat.append(difference)
            TimeGstat.append(time)


    # Create a new figure
    di = pd.DataFrame({'JointIstat': JointIstat})
    dg = pd.DataFrame({'JointGstat': JointGstat})
    dt = pd.DataFrame({'TimeIstat': TimeIstat,'TimeGstat': TimeGstat})
    dik = pd.DataFrame({'IKstat': IKstat})
    # Create boxplots
    #di.boxplot(column=['JointIstat'])

    # Show the figure
    di.to_csv('i_joint_stat.csv', index=False)
    dg.to_csv('g_joint_stat.csv', index=False)
    dt.to_csv('time_stat.csv', index=False)
    dik.to_csv('ik_stat.csv', index=False)  
    #plt.savefig('boxplot.png')

    input('Press enter to exit')

    p.disconnect()



if __name__ == "__main__":
    
    main()
