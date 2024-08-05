import pybullet as p
import time
from numpy import random, rad2deg, deg2rad, set_printoptions, array, linalg, size
import argparse
from ik_solver import rad2nicodeg, nicodeg2rad, match_joints


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
            if link_name.decode("utf-8") == 'endeffector':
                #if link_name.decode("utf-8") == 'endeffector':
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

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--position", nargs=3, type=float, help="Target position for the robot end effector as a list of three floats.")
    parser.add_argument("-j", "--joints", nargs=6, type=float, help="Target joint angles for the robot end effector as a list of six floats.")
    parser.add_argument("-s", "--speed", type=float, default = 1, help="Speed of arm movement in simulator")
    parser.add_argument("-d", "--duration", type=float, default = 1, help="Duration of movement in si/real robot")
    parser.add_argument("-t", "--trajectory", type=int, help="Number of steps in each trajectory")
    parser.add_argument("-pp", "--print_pos_only", action="store_true", help="If set, print only the final end effector position")
    arg_dict = vars(parser.parse_args())

    max_iterations = 100
    residual_threshold = 0.001
    state = []
    finished = False
    success = False 

    p.connect(p.DIRECT)
    #p.connect(p.GUI)
    #p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    #p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=90, cameraPitch=-40, cameraTargetPosition=[0, 0, 0])

    # Load the URDF robot a create scene
    robot_id = p.loadURDF("./nico_upper_rh6d_r.urdf", [0, 0, 0])
    
    num_joints = p.getNumJoints(robot_id)
    joints_limits, joints_ranges, joints_rest_poses, end_effector_index, joint_names, link_names, joint_indices = get_joints_limits(robot_id, num_joints,arg_dict)

    actuated_joints, actuated_initpos = match_joints(init_pos, joint_names)

    # Set initial position
    joints_rest_poses = [nicodeg2rad(joint, init_pos[joint]) for joint in joint_names]

    for i in range(len(joint_indices)):
        p.resetJointState(robot_id, joint_indices[i], joints_rest_poses[i])
    spin_simulation(50)

    if arg_dict["position"]:
        target_position = arg_dict["position"]
     
        ik_solution = p.calculateInverseKinematics(robot_id,
                                                    end_effector_index,
                                                    target_position,
                                                    lowerLimits=joints_limits[0],
                                                    upperLimits=joints_limits[1],
                                                    jointRanges=joints_ranges,
                                                    restPoses=joints_rest_poses,
                                                    maxNumIterations=max_iterations,
                                                    residualThreshold=residual_threshold)
    elif arg_dict["joints"]:
        ik_solution = nicodeg2rad(actuated_joints, arg_dict["joints"])



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
            simdiff = rad2nicodeg(actuated_joints, array(ik_solution)) - rad2nicodeg(actuated_joints, array(state))
            (x,y,z), (a,b,c,d),_,_,_,_ = p.getLinkState(robot_id, end_effector_index) 
            print('SimNICO, Step: {}, Joint angles: {}, Pos: {}'.format(step, ['{:.2f}'.format(diff) for diff in rad2nicodeg(actuated_joints, array(state))], array([x,y,z]),end='\r'))
            #print(linalg.norm(simdiff))
            if linalg.norm(simdiff) <= ACCURACY:
                finished = True
                success = True
            if step > 300:
                print('Solution not found, trajectory not saved')
                finished = True
                success = False
            trajectory.append(state)
            spin_simulation(1)
            state = []
            step += 1
        #SAVE TRAJECTORY AS TEXT FILE
        if success:
            filename = 'trajectories/trajectory.txt'
            with open(filename, 'w') as f:
                steps = arg_dict["trajectory"]
                i_diff = (len(trajectory) - 1) / (steps - 1)
                for j in range(steps):
                    if int(j*i_diff) != int((j-1)*i_diff):
                        f.write("%s\n" % rad2nicodeg(actuated_joints, trajectory[int(j*i_diff)]))

        finished = False

    else:
        for j in range(len(joint_indices)):
            p.resetJointState(robot_id, joint_indices[j], ik_solution[j])
        spin_simulation(100)

    
    #Calculate IK solution error

    (x,y,z), (a,b,c,d),_,_,_,_ = p.getLinkState(robot_id, end_effector_index)

    if arg_dict["print_pos_only"]:
        print(x, y, z)
    else:
        print('SimNico final_pos: {}'.format([x,y,z]))
        if arg_dict["position"]:
            IKdiff = (array(target_position) - array([x,y,z]))
            print('SimNico position error: {}'.format(IKdiff))
        print('SimNico final joint angles: {}'.format(rad2nicodeg(actuated_joints, array(ik_solution))))
    
    p.disconnect()


if __name__ == "__main__":
    main()
