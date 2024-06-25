import pybullet as p
import time
from numpy import random, rad2deg, deg2rad, set_printoptions, array, linalg, round, any
import argparse
import matplotlib.pyplot as plt
import pandas as pd
import os
from sim_height_calculation import calculate_z
import calibration_matrices

# from TouchAgent import TouchAgent

# Set speed to reseti to initial position
RESET_SPEED = 0.05
# Acccuracy (vector distance) to consider the target positon reached
ACCURACY = 3
# Delay between simulation steps
SIM_STEP_DELAY = 0.01
# Amount of decimals to round to when writing trajectory to file
DECIMALS = 10

# Once the coordinate system is fixed and all that,
# when the code is extracted into a compact version for my project
# that can be called and returns the angles, this will move into global_static_vars
ANGLE_SHIFT_WRIST_Z = 56
ANGLE_SHIFT_WRIST_X = 120

init_pos = {  # standard position
    'l_shoulder_z': -25.0,
    'l_shoulder_y': 0.0,
    'l_arm_x': 30.0,
    'l_elbow_y': 89.0,
    'l_wrist_z': 0.0,
    'l_wrist_x': -180.0,
    'l_thumb_z': -57.0,
    'l_thumb_x': -180.0,
    'l_indexfinger_x': -180.0,
    'l_middlefingers_x': -180.0,
    'r_shoulder_z': -20,
    'r_shoulder_y': 70,
    'r_arm_x': 30,
    'r_elbow_y': 60,
    'r_wrist_z': 0,
    'r_wrist_x': 0,
    'r_thumb_z': -180.0,
    'r_thumb_x': -180.0,
    'r_indexfinger_x': -90,
    'r_middlefingers_x': 180.0,
    'head_z': 0.0,
    'head_y': 0.0
}

set_printoptions(precision=3)
set_printoptions(suppress=True)

def get_joints_limits(robot_id, num_joints, arg_dict):
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
        if q_index > -1:  # Fixed joints have q_index -1
            joint_names.append(joint_info[1].decode("utf-8"))
            link_names.append(joint_info[12].decode("utf-8"))
            joint_indices.append(joint_info[0])
            joints_limits_l.append(joint_info[8])
            joints_limits_u.append(joint_info[9])
            joints_ranges.append(joint_info[9] - joint_info[8])
            joints_rest_poses.append((joint_info[9] + joint_info[8]) / 2)
        if arg_dict["left"]:
            if link_name.decode("utf-8") == 'endeffectol':
                end_effector_index = jid
        else:
            if link_name.decode("utf-8") == 'endeffector':
                # if link_name.decode("utf-8") == 'endeffector':
                end_effector_index = jid

    return [joints_limits_l,
            joints_limits_u], joints_ranges, joints_rest_poses, end_effector_index, joint_names, link_names, joint_indices


def get_real_joints(robot, joints):
    last_position = []

    for k in joints:
        actual = robot.getAngle(k)
        # print("{} : {}, ".format(k,actual),end="")
        last_position.append(actual)
    # print("")

    return last_position


def match_joints(init_pos, joint_names):
    actuated_joint_names = []
    actuated_joint_init_pos = []
    for name in init_pos.keys():
        if name in joint_names:
            actuated_joint_names.append((name))
            actuated_joint_init_pos.append(init_pos[name])

    return actuated_joint_names, actuated_joint_init_pos


def reset_robot(robot, init_pos):
    for k in init_pos.keys():
        robot.setAngle(k, init_pos[k], RESET_SPEED)

    return robot


def reset_actuated(robot, actuated_joints, actuated_initpos):
    for joint, initpos in zip(actuated_joints, actuated_initpos):
        robot.setAngle(joint, initpos, RESET_SPEED)
    return robot


def speed_control(initial, target, duration):
    speed_to_reach = (abs((float(initial) - float(target)) / float(1260 * duration)))
    return speed_to_reach


def sim_speed_control(initial, target, duration):
    return abs(float(initial) - float(target)) / float(0.425 * duration)


def spin_simulation(steps):
    for i in range(steps):
        p.stepSimulation()
        time.sleep(0.01)


def check_response(robot, joints, joints_state_before, tic):
    while True:
        actual = get_real_joints(robot, joints)
        diff = array(joints_state_before) - array(actual)
        if any(diff):
            break
        print('Duration: {:.2f}'.format(time.time() - tic), end='\r')
        time.sleep(0.01)
    toc = time.time()
    return toc - tic


def check_execution(robot, joints, target, verbose):
    tic = time.time()
    distance = 100
    step = 0
    while distance > ACCURACY:
        actual = get_real_joints(robot, joints)
        # print(timestamp)
        diff = array(target) - array(actual)
        distance = linalg.norm(diff)
        if verbose:
            print('RealNICO Step: {}, Time: {:.2f}, JointDeg: {}'.format(step, time.time() - tic, ['{:.2f}'.format(act) for act in actual]))
        else:
            print('Duration: {:.2f}, Error: {:.2f}'.format(time.time() - tic, distance), end='\r')
        time.sleep(0.01)
        step += 1
    toc = time.time()
    return toc - tic


def to_formatted_str(number):
    number = str(number)

    decimal_point_index = number.find('.')
    if decimal_point_index >= 0:
        number += '0' * (DECIMALS - (len(number) - decimal_point_index - 1))

    return number


def write_first_line(file):
    offset1, offset2 = 7 * (DECIMALS + 6), DECIMALS + 20
    first_line = "JOINTS ANGLES (DEGREES)" + " " * (offset1 - 23) + "DURATION (SECONDS)" + " " * (offset2 - 18) + "END EFFECTOR POSITION (CARTEZIAN)"
    file.write(first_line + "\n")


def write_line(file, joint_angles, duration, end_effector_coords):
    offset1, offset2 = 7 * (DECIMALS + 6), DECIMALS + 20

    length = file.write("%s " % ','.join(list(map(to_formatted_str, round(rad2deg(joint_angles), DECIMALS)))))
    file.write(" " * (offset1 - length))
    length = file.write("%s " % to_formatted_str(round(duration, DECIMALS)))
    file.write(" " * (offset2 - length))
    file.write("%s\n" % ','.join(list(map(to_formatted_str, round(end_effector_coords, DECIMALS)))))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", action="store_true", help="Show detail informtion about robot position in terminal")
    parser.add_argument("-p", "--position", nargs=3, type=float, help="Target position for the robot end effector as a list of three floats.")
    parser.add_argument("-j", "--joints", nargs=6, type=float, help="Target joint angles for the robot end effector as a list of six floats.")
    parser.add_argument("-rr", "--real_robot", action="store_true", help="If set, execute action on real robot.")
    parser.add_argument("-a", "--animate", action="store_true", help="If set, the animation of motion is shown.")
    parser.add_argument("-g", "--gui", action="store_true", help="If set, turn the GUI on")
    parser.add_argument("-f", "--file", type=str, help="Target position for the robot end effector as a list of three floats.")
    parser.add_argument("-l", "--left", action="store_true", help="If set, use left hand IK")
    parser.add_argument("-i", "--initial", action="store_true", help="If set, reset the robot to the initial position after each postion")
    parser.add_argument("-t", "--trajectory", type=str, help="If set, execute trajectory positions from the text file with corresponding path")
    parser.add_argument("-c", "--calibration", action="store_true", help="If set, execute calibration positions")
    parser.add_argument("-e", "--experiment", action="store_true", help="If set, execute experiments positions")
    parser.add_argument("-s", "--speed", type=float, default=1, help="Speed of arm movement in simulator")
    parser.add_argument("-d", "--duration", type=float, default=3, help="Duration of movement in si/real robot")
    parser.add_argument("-ts", "--trajectory_steps", type=int, default=5, help="Number of steps in each trajectory")
    parser.add_argument("-st", "--save_trajectory", action="store_true", help="Store coordinates from simulation into text file")
    parser.add_argument("-cz", "--calculate_z_value", action="store_true", help="If set, the z value is calculated from calibration grid")
    parser.add_argument("-ss", "--save_statistics", action="store_true", help="If set, joint difference and time statistics are saved into text files")
    arg_dict = vars(parser.parse_args())

    # TouchAgent()
    # TouchAgent.clean()

    # GUI initialization
    if arg_dict["gui"]:
        p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=90, cameraPitch=-40, cameraTargetPosition=[0, 0, 0])
    else:
        p.connect(p.DIRECT)

    # Load the URDF robot a create scene
    if arg_dict["left"]:
        robot_id = p.loadURDF("./nico_upper_rh6d_l.urdf", [0, 0, 0])
    else:
        robot_id = p.loadURDF("./nico_upper_rh6d_r.urdf", [0, 0, 0])
    # Create table mesh
    p.createMultiBody(baseVisualShapeIndex=p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[.3, .45, 0.02],
                                                               rgbaColor=[0.6, 0.6, 0.6, 1]),
                      baseCollisionShapeIndex=p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[.3, .45, 0.02]),
                      baseMass=0, basePosition=[0.27, 0, -0.005])
    # Create tablet mesh
    p.createMultiBody(baseVisualShapeIndex=p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[.16, .26, 0.01],
                                                               rgbaColor=[0, 0, 0.0, 1]),
                      baseCollisionShapeIndex=p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                                     halfExtents=[.16, .26, 0.01]), baseMass=0,
                      basePosition=[0.41, 0, 0.008])

    num_joints = p.getNumJoints(robot_id)
    joints_limits, joints_ranges, joints_rest_poses, end_effector_index, joint_names, link_names, joint_indices = get_joints_limits(
        robot_id, num_joints, arg_dict)
    # Custom intital position

    # joints_rest_poses = deg2rad([-15, 68, 2.8, 56.4, 0.0, 11.0, -70.0])

    actuated_joints, actuated_initpos = match_joints(init_pos, joint_names)

    # Real robot initialization and setting all joints
    if arg_dict["real_robot"]:
        from nicomotion.Motion import Motion
        motorConfig = './nico_humanoid_upper_rh7d_ukba.json'
        try:
            robot = Motion(motorConfig=motorConfig)
            print('Robot initialized')
        except:
            print('Motors are not operational')
            exit()
        # robot = init_robot()
        robot = reset_robot(robot, init_pos)
        time_res = check_execution(robot, init_pos.keys(), list(init_pos.values()), arg_dict["verbose"])
        print('Robot reset in {:.2f} seconds.'.format(time_res))
        actual_position = get_real_joints(robot, actuated_joints)
    if not arg_dict["initial"]:
        for i in range(len(joint_indices)):
            p.resetJointState(robot_id, joint_indices[i], deg2rad(actuated_initpos[i]))
    #spin_simulation(50)

    # IK paramenters
    max_iterations = 100
    residual_threshold = 0.001

    # Statistics
    TargetPstat = []        # target position x, y
    TimeRstat = []          # response time
    IKstat = []
    JointIstat = []
    TimeIstat = []
    JointGstat = []
    TimeGstat = []
    state = []
    sim_time_errors = []
    finished = False

    movement_duration = arg_dict["duration"]

    # if arg_dict["file"]:
    #    open(arg_dict["file"]) as data

    # Reads trajectory points from file
    trajectory_joints, trajectory_durations, trajectory_end_effector_positions = [], [], []
    if arg_dict["trajectory"] is not None:
        file_name = arg_dict['trajectory']
        with open(file_name, 'r') as f:
            content = f.read().split('\n')[1:-1]

        if content:
            for pos in content:
                joints, duration, end_effector_position = pos.split()
                trajectory_joints.append(list(map(float, joints.split(','))))
                trajectory_durations.append(float(duration))
                trajectory_end_effector_positions.append(list(map(float, end_effector_position.split(','))))
            print("Movement duration is being overwritten by a value in trajectory file.")
        else:
            print("File empty")

    # Creates needed directories if not created yet
    if arg_dict["save_trajectory"]:
        if not os.path.exists("trajectories"):
            os.mkdir("trajectories")
        if not os.path.exists("linear_trajectories"):
            os.mkdir("linear_trajectories")
    
    if arg_dict["save_statistics"]:
        if not os.path.exists("statistics"):
            os.mkdir("statistics")

    for i in range(40):
        # ik_solution = tuple()
        # while True:
        # Target position
        if arg_dict["position"]:
            target_position = arg_dict["position"]
        elif arg_dict["calibration"]:
            target_position = calibration_matrices.target_calibration(i)
        elif arg_dict["experiment"]:
            target_position = calibration_matrices.target_experiment(i)
        elif arg_dict["trajectory"] is not None:
            index = i % len(trajectory_end_effector_positions)
            target_position = trajectory_end_effector_positions[index]
            # ik_solution = deg2rad(trajectory_joints[index])
            movement_duration = trajectory_durations[index]
        # elif arg_dict["file"]:
        #    target_position = data[i]
        #    print target_position
        elif arg_dict["joints"]:
            target_position = target_joints(i)
        else:
            target_position = target_random()

        if arg_dict["calculate_z_value"]:
            target_position[2] = calculate_z(target_position[0], target_position[1])
        
        TargetPstat.append(target_position[:2])

        # Create goal dot
        p.createMultiBody(
            baseVisualShapeIndex=p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.01, rgbaColor=[0, 0, 1, .7]),
            baseCollisionShapeIndex=-1, baseMass=0, basePosition=target_position)

        # Reset robot to initial position
        if arg_dict["initial"]:
            

            resetsim_pos = []
            for j in range(len(joint_indices)):
                p.resetJointState(robot_id, joint_indices[j], deg2rad(actuated_initpos[j]))
            for j in range(len(joint_indices)):
                    resetsim_pos.append(p.getJointState(robot_id, joint_indices[j])[0])
            simdifference = array(actuated_initpos) - array(rad2deg(resetsim_pos))
            (x, y, z), (a, b, c, d), _, _, _, _ = p.getLinkState(robot_id, end_effector_index)
            if arg_dict["verbose"]:
                print('SimNICO init: 0s \n Error: {} \n Goal: {} \n Real: {} \n InitPos: {}'.format(
                                                                    ['{:.2f}'.format(diff) for diff in simdifference],
                                                                    ['{:.2f}'.format(goal) for goal in actuated_initpos],
                                                                    ['{:.2f}'.format(sim) for sim in rad2deg(resetsim_pos)],
                                                                    ['{:.2f}'.format(simpos) for simpos in [x, y, z]]))
            if arg_dict["real_robot"]:
                robot = reset_actuated(robot, actuated_joints, actuated_initpos)
                time_res = check_execution(robot, actuated_joints, actuated_initpos, arg_dict["verbose"])
                reset_pos = get_real_joints(robot, actuated_joints)
                difference = array(actuated_initpos) - array(reset_pos)
                if arg_dict["verbose"]:
                    print('RealNICO init: {:.2f}s \n Error: {} \n Goal: {} \n Real: {}'.format(time_res,
                                                                    ['{:.2f}'.format(diff) for diff in difference],
                                                                    ['{:.2f}'.format(goal) for goal in actuated_initpos],
                                                                    ['{:.2f}'.format(real) for real in reset_pos]))
                    input('Compare real and sim position visually')
                JointIstat.append(difference)
                TimeIstat.append(time)
            
            # spin_simulation(20)

        # target_orientation = target_position + [1]
        # Perform IK
        # ik_solution = p.calculateInverseKinematics(robot_id, end_effector_index, target_position,
        #                                           targetOrientation=target_orientation,
        #                                        maxNumIterations=max_iterations,
        #                                        residualThreshold=residual_threshold)

        # if not len(ik_solution):            # If we are reading trajectory from the file, we don't need to calculate
        ik_solution = p.calculateInverseKinematics(robot_id,
                                                   end_effector_index,
                                                   target_position,
                                                   lowerLimits=joints_limits[0],
                                                   upperLimits=joints_limits[1],
                                                   jointRanges=joints_ranges,
                                                   maxNumIterations=max_iterations,
                                                   residualThreshold=residual_threshold)


        trajectory = []

        save_trajectory_joints, save_trajectory_durations, save_trajectory_end_effector_coords = [], [], []

        if arg_dict["animate"]:
            for j in range(len(joint_indices)):
                speed = sim_speed_control(p.getJointState(robot_id, joint_indices[j])[0], ik_solution[j],
                                          movement_duration)

                p.setJointMotorControl2(robot_id, joint_indices[j],
                                        p.POSITION_CONTROL, ik_solution[j],
                                        maxVelocity=speed,
                                        force=500,
                                        positionGain=0.7,
                                        velocityGain=0.3)

            tic = time.perf_counter()

            step = 1
            while not finished:
                for j in range(len(joint_indices)):
                    state.append(p.getJointState(robot_id, joint_indices[j])[0])
                simdiff = rad2deg(array(ik_solution)) - rad2deg(array(state))
                if arg_dict["verbose"]:
                    print('SimNICO, Step: {}, JointDeg: {}'.format(step, ['{:.2f}'.format(pos) for pos in rad2deg(state)], end='\n'))
                if linalg.norm(simdiff) <= ACCURACY:
                    finished = True

                # Saving trajectory points in lists for writing into file
                if arg_dict["save_trajectory"]:
                    save_trajectory_joints.append(state)
                    save_trajectory_durations.append(time.perf_counter() - tic)
                    save_trajectory_end_effector_coords.append(p.getLinkState(robot_id, end_effector_index)[0])
                spin_simulation(1)
                step += 1
                last_state = state
                state = []
                if step > 350:
                    print('ANIMATION MODE FAILED - NEEDS DEBUGGGING')
                    finished = True

            toc = time.perf_counter()

            # Saving the last point of trajectory
            if arg_dict["save_trajectory"]:
                for j in range(len(joint_indices)):
                    state.append(p.getJointState(robot_id, joint_indices[j])[0])
                save_trajectory_joints.append(state)
                save_trajectory_durations.append(toc - tic)
                save_trajectory_end_effector_coords.append(p.getLinkState(robot_id, end_effector_index)[0])
                state = []

            # print('Sim time using step count:', step * SIM_STEP_DELAY)

            # print("sim time using time module:", toc - tic)

            sim_time_error = abs(arg_dict['duration'] - step * SIM_STEP_DELAY)
            # print('Sim time error using step count:', sim_time_error)
            sim_time_errors.append(sim_time_error)

            # WRITING TRAJECTORY POINTS TO TEXT FILE
            if arg_dict["save_trajectory"]:
                steps = arg_dict["trajectory_steps"]

                filename = 'trajectories/trajectory_' + str(i) + '.txt'
                with open(filename, 'w') as f:
                    write_first_line(f)
                    if steps > 1:
                        i_dif = (len(save_trajectory_joints) - 1) / (steps - 1)
                        for j in range(steps):
                            index = int(j * i_dif)
                            if index != int((j - 1) * i_dif):
                                duration = 1
                                if j != 0:
                                    duration = save_trajectory_durations[index] - save_trajectory_durations[
                                        int((j - 1) * i_dif)]

                                write_line(f, save_trajectory_joints[index], duration,
                                           save_trajectory_end_effector_coords[index])
                    else:
                        print("Trajectory steps must be greater than 1")
                        quit()

                filename = 'linear_trajectories/trajectory_' + str(i) + '.txt'
                with open(filename, 'w') as f:
                    write_first_line(f)
                    duration = arg_dict['duration'] / (steps - 1)
                    write_line(f, save_trajectory_joints[0], 1, save_trajectory_end_effector_coords[0])

                    end_effector_coords = list(save_trajectory_end_effector_coords[0])
                    end_effector_target_coords = save_trajectory_end_effector_coords[-1]
                    end_effector_step = ((end_effector_target_coords[0] - end_effector_coords[0]) / (steps - 1),
                                         (end_effector_target_coords[1] - end_effector_coords[1]) / (steps - 1),
                                         (end_effector_target_coords[2] - end_effector_coords[2]) / (steps - 1))
                    for j in range(steps - 2):
                        for k in range(3):
                            end_effector_coords[k] += end_effector_step[k]

                        ik_solution = p.calculateInverseKinematics(robot_id,
                                                                   end_effector_index,
                                                                   end_effector_coords,
                                                                   lowerLimits=joints_limits[0],
                                                                   upperLimits=joints_limits[1],
                                                                   jointRanges=joints_ranges,
                                                                   restPoses=joints_rest_poses,
                                                                   maxNumIterations=max_iterations,
                                                                   residualThreshold=residual_threshold)

                        write_line(f, ik_solution, duration, end_effector_coords)

                    write_line(f, save_trajectory_joints[-1], duration, save_trajectory_end_effector_coords[-1])

                filename = 'test_trajectories/trajectory_' + str(i) + '.txt'
                with open(filename, 'w') as f:
                    write_first_line(f)
                    for j in range(len(save_trajectory_joints)):
                        duration = 1
                        if j != 0:
                            duration = save_trajectory_durations[j] - save_trajectory_durations[j - 1]

                        write_line(f, save_trajectory_joints[j], duration, save_trajectory_end_effector_coords[j])

            finished = False

        else:
            for j in range(len(joint_indices)):
                p.resetJointState(robot_id, joint_indices[j], ik_solution[j])
            #spin_simulation(10)
    
        # Calculate IK solution error

        (x, y, z), (a, b, c, d), _, _, _, _ = p.getLinkState(robot_id, end_effector_index)
        IKdiff = (array(target_position) - array([x, y, z]))
        # print('SimNico target_pos: {}'.format(target_position))
        # print('SimNico IK error: {}'.format(IKdiff))
        IKstat.append(IKdiff)
        simdiff = rad2deg(array(ik_solution)) - rad2deg(array(last_state))
        if arg_dict["verbose"]:
            print('SimNICO goal: {:.2f}s \n Error: {} \n Goal: {} \n Real: {} \n PosError: {} \n GoalPos: {} \n RealPos: {}'.format((toc - tic),
                                                                    ['{:.2f}'.format(diff) for diff in simdiff],
                                                                    ['{:.2f}'.format(goal) for goal in rad2deg(ik_solution)],
                                                                    ['{:.2f}'.format(real) for real in rad2deg(last_state)],
                                                                    ['{:.2f}'.format(posdiff) for posdiff in IKdiff],
                                                                    ['{:.2f}'.format(goalpos) for goalpos in target_position],
                                                                    ['{:.2f}'.format(realpos) for realpos in [x, y, z]]))
            
        if arg_dict["real_robot"]:

            targetdeg = []
            joint_values = get_real_joints(robot, actuated_joints)
            tic = time.time()

            print('movement_duration: {}'.format(movement_duration))

            for i, realjoint in enumerate(actuated_joints):
                degrees = rad2deg(ik_solution[i])
                speed = speed_control(actual_position[i], degrees, movement_duration)
                if realjoint == 'r_wrist_z':
                    degrees += ANGLE_SHIFT_WRIST_Z
                elif realjoint == 'r_wrist_x':
                    degrees += ANGLE_SHIFT_WRIST_X
                
                robot.setAngle(realjoint, degrees, speed)
                targetdeg.append(degrees)
            
            time_response = check_response(robot, actuated_joints, joint_values, tic)
            print('RealNICO response time: {:.2f}s'.format(time_response))
            TimeRstat.append(time_response)
            
            # time.sleep(arg_dict['duration'])
            time_ex = check_execution(robot, actuated_joints, targetdeg,arg_dict["verbose"])
            # time_ex= (arg_dict['duration'])
            # time.sleep(2)
            final_pos = get_real_joints(robot, actuated_joints)
            difference = array(targetdeg) - array(final_pos)
            if arg_dict["verbose"]:
                print('RealNICO goal: {:.2f}s \n Error: {} \n Goal: {} \n Real: {}'.format(time_res,
                                                                    ['{:.2f}'.format(diff) for diff in difference],
                                                                    ['{:.2f}'.format(goal) for goal in targetdeg],
                                                                    ['{:.2f}'.format(real) for real in final_pos]))
                input('Compare real and sim position visually')
            JointGstat.append(difference)
            TimeGstat.append(time_ex)
        
    

    # filename = 'sim_time_errors/sim_time_error_with_duration_' + str(arg_dict['duration']) + '.txt'
    # with open(filename, 'w') as f:
    #     for sim_time_error in sim_time_errors:
    #         f.write("%s\n" % sim_time_error)

    if arg_dict["save_statistics"]:
        filename = 'statistics/10_steps_position_target.txt'
        # filename = '../jurajgavura/BP/exp_random/random_sw_duration_3/random_position_target.txt'
        with open(filename, 'w') as f:
            for targetP in TargetPstat:
                f.write("%s\n" % ', '.join(map(str, round(targetP, 3))))
        
        filename = 'statistics/10_steps_joint_diffs.txt'
        # filename = '../jurajgavura/BP/exp_random/random_sw_duration_3/random_joint_diffs.txt'
        with open(filename, 'w') as f:
            for joint_diff in JointGstat:
                f.write("%s\n" % ' '.join(map(str, round(joint_diff, 3))))
        
        filename = 'statistics/10_steps_time_response.txt'
        # filename = '../jurajgavura/BP/exp_random/random_sw_duration_3/random_time_response.txt'
        with open(filename, 'w') as f:
            for timeR in TimeRstat:
                f.write("%s\n" % '{:.2f}'.format(timeR))

        filename = 'statistics/10_steps_time_execution.txt'
        # filename = '../jurajgavura/BP/exp_random/random_sw_duration_3/random_time_execution.txt'
        with open(filename, 'w') as f:
            for timeG in TimeGstat:
                f.write("%s\n" % '{:.2f}'.format(timeG))

    # Create a new figure
    # di = pd.DataFrame({'JointIstat': JointIstat})
    # dg = pd.DataFrame({'JointGstat': JointGstat})
    # dt = pd.DataFrame({'TimeIstat': TimeIstat, 'TimeGstat': TimeGstat})
    # dik = pd.DataFrame({'IKstat': IKstat})
    # Create boxplots
    # di.boxplot(column=['JointIstat'])

    # Show the figure
    # di.to_csv('i_joint_stat.csv', index=False)
    # dg.to_csv('g_joint_stat.csv', index=False)
    # dt.to_csv('time_stat.csv', index=False)
    # dik.to_csv('ik_stat.csv', index=False)
    # plt.savefig('boxplot.png')

    input('Press enter to exit')

    p.disconnect()


if __name__ == "__main__":
    main()
