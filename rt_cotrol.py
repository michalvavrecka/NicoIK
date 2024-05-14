import pybullet as p
import time
from numpy import random, rad2deg, deg2rad, set_printoptions, array, linalg
import argparse

# Set speed to reseti to initial position
RESET_SPEED = 0.05



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

def sim_speed_control(initial, target, duration):
    return abs(float(initial) - float(target)) / float(0.425*duration)

def spin_simulation(steps):
    for i in range(steps):
        p.stepSimulation()
        time.sleep(SIM_STEP_DELAY)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-l", "--left", action="store_true", help="If set, use left hand IK")
    parser.add_argument("-rr", "--real_robot", action="store_true", help="If set, execute action on real robot.")
    arg_dict = vars(parser.parse_args())

    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=90, cameraPitch=-40, cameraTargetPosition=[0, 0, 0])
    
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

    for i in range ):
                        joints[i] = p.addUserDebugParameter(joints[i], env.action_space.low[i], env.action_space.high[i], env.env.robot.init_joint_poses[i])

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
        #robot = init_robot()
        robot = reset_robot(robot,init_pos)
        time_res = check_execution(robot, init_pos.keys(), list(init_pos.values()))
        print ('Robot reset in {:.2f} seconds.'.format(time_res))  
        actual_position = get_real_joints(robot,actuated_joints)
        for i in range(len(joint_indices)):
            p.resetJointState(robot_id, joint_indices[i], deg2rad(actual_position[i]))
        spin_simulation(50)

    else:
        for i in range(len(joint_indices)):
            p.resetJointState(robot_id, joint_indices[i], joints_rest_poses[i])
        spin_simulation(50)

    # IK paramenters
    state = []
    sim_time_errors = []
    finished = False

    movement_duration = 0.1

    while True: 
        
        if arg_dict["real_robot"]:
            #for i,realjoint in enumerate(actuated_joints):
            #    degrees = rad2deg(actuated_initpos[i])
            #    speed = speed_control(actual_position[i], degrees, movement_duration)
            #    if realjoint == 'r_wrist_z':
            #        degrees += ANGLE_SHIFT_WRIST_Z
            #    elif realjoint == 'r_wrist_x':
            #        degrees += ANGLE_SHIFT_WRIST_X  
            #robot.setAngle(realjoint, degrees,speed)
            robot = reset_actuated(robot,actuated_joints,actuated_initpos)
            time_res = check_execution(robot, actuated_joints, actuated_initpos)
            reset_pos = get_real_joints(robot,actuated_joints)
            difference = array(actuated_initpos) - array(reset_pos)
            print('RealNICO init: {:.2f}s, Error: {}'.format(time_res, ['{:.2f}'.format(diff) for diff in difference])) 
        for j in range(len(joint_indices)):
            p.resetJointState(robot_id, joint_indices[j], joints_rest_poses[j])
        # spin_simulation(20)
        
        ik_solution = get_sliders_values()

        for j in range(len(joint_indices)):
            speed = sim_speed_control(p.getJointState(robot_id,joint_indices[j])[0], ik_solution[j], movement_duration)

            p.setJointMotorControl2(robot_id, joint_indices[j],
                                    p.POSITION_CONTROL, ik_solution[j],
                                    maxVelocity=speed,
                                    force=500,
                                    positionGain=0.7,
                                    velocityGain=0.3)



        while not finished:
            for j in range(len(joint_indices)):
                state.append(p.getJointState(robot_id,joint_indices[j])[0])
            simdiff = rad2deg(array(ik_solution)) - rad2deg(array(state))
            # print('SimNICO, Step: {}, Error: {}'.format(step, ['{:.2f}'.format(diff) for diff in simdiff],end='\r'))
            if linalg.norm(simdiff) <= ACCURACY:
                finished = True

            spin_simulation(1)
            step += 1
            state = []

            

        finished = False

        
        #Calculate IK solution error

        (x,y,z), (a,b,c,d),_,_,_,_ = p.getLinkState(robot_id, end_effector_index) 

        
        if arg_dict["real_robot"]:
            
            targetdeg = []

            for i,realjoint in enumerate(actuated_joints):
                degrees = rad2deg(ik_solution[i])
                speed = speed_control(actual_position[i], degrees, movement_duration)
                if realjoint == 'r_wrist_z':
                    degrees += ANGLE_SHIFT_WRIST_Z
                elif realjoint == 'r_wrist_x':
                    degrees += ANGLE_SHIFT_WRIST_X  
                robot.setAngle(realjoint, degrees,speed)
                targetdeg.append(degrees)
            #time.sleep(movement_duration)
            time_ex = check_execution(robot, actuated_joints, targetdeg)     
            #time_ex= (movement_duration)
            #time.sleep(2)
            final_pos = get_real_joints(robot,actuated_joints)
            difference = array(targetdeg) - array(final_pos)
            print('RealNICO goal: {:.2f}s, Error: {}'.format(time_ex, ['{:.2f}'.format(diff) for diff in difference])) 
            JointGstat.append(difference)
            TimeGstat.append(time_ex)

    filename = 'sim_time_errors/sim_time_error_with_duration_' + str(movement_duration) + '.txt'
    with open(filename, 'w') as f:
        for sim_time_error in sim_time_errors:
            f.write("%s\n" % sim_time_error)


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
