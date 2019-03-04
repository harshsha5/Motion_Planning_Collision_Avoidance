import os
import sys
import time
import numpy as np
#import ipdb
import transforms3d
import vrep_utils as vu
from random import random
from random import seed
import locobot_bounding_box as loc_bb
import collision_checker
from collections import defaultdict 
from urdf_parser_py.urdf import URDF
import locobot_joint_ctrl as ctrl
import pdb

class Graph: 
    #NOTE: Reference stated in the documentation of the program
    # Constructor 
    def __init__(self): 
  
        # default dictionary to store graph 
        self.graph = defaultdict(list) 
  
    # function to add an edge to graph 
    def addEdge(self,u,v): 
        self.graph[u].append(v)  

    def addVertex(self,u):
        self.graph[u] = []

class State:
    # Constructor 
    def __init__(self): 
  
        # default dictionary to store state in the graph
        self.joint_1 = None
        self.joint_2 = None
        self.joint_3 = None
        self.joint_4 = None
        self.joint_5 = None
        self.joint_6 = None
        self.joint_7 = None
  
    # function to add an edge to graph 
    def create_state(self,state): 
        '''
        state is a numpy array of 1X7. For this particular use case joint_6 and joint_7 are always fixed which are the fingers of the robot gripper.
        '''
        self.joint_1 = state[0]
        self.joint_2 = state[1]
        self.joint_3 = state[2]
        self.joint_4 = state[3]
        self.joint_5 = state[4]
        self.joint_6 = state[5]
        self.joint_7 = state[6]   

#Storing Model: Assume obj to have 3 numpy arrays- Position,Rotation and Axis.
#Position is 3X7. Axis is 3X7. ANd rotation is 3X7

class Model:
    def __init__(self): 
  
        self.position = None
        self.rotation = None
        self.axis = None

    def initialize_model_using_np(self,initial_joint_pos,initial_joint_orientation,initial_rotation):
        self.position = initial_joint_pos
        self.rotation = initial_joint_orientation
        self.axis = initial_rotation


def get_forward_kinematics(joint_angles,robot_model):
    homo_matrix_list = []
    present_net_homo = np.zeros((4,4))
    for i,elt in enumerate(joint_angles.shape[0]):
        rpy = robot_model.rotation[i]                               #Assume angles provided by model is in Euler format
        net_rotation = rpy + joint_angles[i]*robot_model.axis[i]
        R = transforms3d.euler.euler2mat(net_rotation[0], net_rotation[1], net_rotation[2], 'sxyz') #SOURCE OF ERROR- see xyz correct or not
        p = robot_model.position[i] 
        homo_temp = np.vstack(np.hstack(R,p),[[0,0,0,1]])
        present_net_homo = np.matmul(present_net_homo,homo_temp)
        homo_matrix_list.append(present_net_homo)
    return homo_matrix_list

# def get_bb_transforms(clientID):
#     homo_cuboid_list =[]
#     cuboid_pos,cuboid_orientation,cuboid_bb = loc_bb.collision_cuboid_configurations(clientID)
#     for i,elt in enumerate():

def get_random_joint_angles():      #Edit this so that it returns a 1 X 7 numpy array
    # random.randrange(-90, 90.1, 0.5)
    # return 360*np.random.rand(5)
    return np.random.uniform(-90,90,5)          #THE JOINT LIMITS ARE HARD CODED AS OF NOW. CHANGE WRT THE URDF

def check_if_state_is_collision_free(clientID,random_joint_angles,robot_model,robot_cuboid_dimensions,tf_bw_joint_cuboid_centroid):
    '''
    INPUT: random_joint_angles- (7,) Numpy vector
           robot_model a Model object
           robot_cuboid_dimensions- A 7X3 Matrix. That is 7 joints and x,y,z dimensions
           tf_bw_joint_cuboid_centroid- A 4X4X7 Matrix. ie. Homogeneous matrix for transforms b/w joint to cuboid for all the 7 joints
    OUTPUT: Returns True if state is collision free
    '''
    joint_cuboid_list =[]
    homo_matrix_list = get_forward_kinematics(random_joint_angles,robot_model)
    for i,elt in enumerate(homo_matrix_list):
        H_base_to_cc = np.matmul(elt,tf_bw_joint_cuboid_centroid[:,:,i])
        al, be, ga = mat2euler(H_base_to_cc, 'sxyz')
        pos = H_base_to_cc[0:3,3]
        pos = pos.tolist()
        dim = robot_cuboid_dimensions[i,:].tolist()
        cuboid_temp = collision_checker.cuboid(pos,[al,be,ga],dim)
        joint_cuboid_list.append(cuboid_temp)


def add_state_to_graph_as_vertex(g,joint_angle_array):
    S = State()
    S.create_state(joint_angle_array)
    g.addVertex(S)
    #see if return by value or by reference

def initialize_graph(clientID,g):
    initial_joint_angles = vu.get_arm_joint_positions(clientID)
    add_state_to_graph_as_vertex(g,initial_joint_angles)

def make_graph_PRM(clientID,g,robot_model,number_of_points_to_sample = 100):
    '''
    The graph phase of the PRM
    '''
    arm_handles = vu.get_arm_joint_handles(clientID)
    initialize_graph(clientID,g)
    robot_cuboid_dimensions = get_robot_cuboid_dimensions(clientID)
    #tf_bw_joint_cuboid_centroid = np.load("tf_bw_joint_cuboid_centroid.npy")
    for i in range(number_of_points_to_sample):
        random_joint_angles = get_random_joint_angles()
        finger_angle = np.array([0,0])                              #Initial angle for fingers. MUST CHANGE!
        random_joint_angles = np.hstack((random_joint_angles,finger_angle))
        # joint_positions =vu.get_arm_joint_positions(clientID)
        #previous_joint_positions = joint_positions
        if(check_if_state_is_collision_free(clientID,random_joint_angles,robot_model,robot_cuboid_dimensions,tf_bw_joint_cuboid_centroid)):   #THIS IS CORRECT
            print("hi")
    pass

def get_robot_cuboid_dimensions(clientID):
    '''
    OUTPUT: Returns a 7X3 matrix. Where 7 is the number of joints and 3 is dimension along x,y,z
    '''
    collision_cuboid_handles = vu.get_collision_cuboid_handles(clientID)
    result = loc_bb.get_collision_cuboid_bounding_boxes(collision_cuboid_handles,clientID)
    result = result[:,1,:] - result[:,0,:]
    return result

def get_model_from_URDF(urdf_xml):
    return URDF.from_xml_file(urdf_xml)

def initialize_model(initial_state_file1,initial_state_file2,robot_model):
    '''
    Used to generate the initial model of the robot with all joint angles as zero
    '''
    initial_joint_pos = np.load(initial_state_file1)
    initial_joint_orientation = np.load(initial_state_file2)
    joint_axis = np.array([[0,0,1],[0,1,0],[0,1,0],[0,1,0],[-1,0,0],[0,1,0],[0,1,0]])
    robot_model.initialize_model_using_np(initial_joint_pos,initial_joint_orientation,joint_axis)

def control_locobot(joint_targets,clientID):
    # Reset simulation in case something was running
    vu.reset_sim(clientID)
    
    # Initial control inputs are zero
    vu.set_arm_joint_target_velocities(clientID, np.zeros(vu.N_ARM_JOINTS))

    # Despite the name, this sets the maximum allowable joint force
    vu.set_arm_joint_forces(clientID, 50.*np.ones(vu.N_ARM_JOINTS))

    # One step to process the above settings
    vu.step_sim(clientID)

    deg_to_rad = np.pi/180

    # Instantiate controller
    controller = ctrl.ArmController()

    # Set previous integral error
    controller.set_previous_integral_error()

    # Iterate through target joint positions
    for target_number,target in enumerate(joint_targets):

        # Set new target position
        controller.set_target_joint_positions(target)

        steady_state_reached = False

        count = 1
        while not steady_state_reached:

            timestamp = vu.get_sim_time_seconds(clientID)
            print('Simulation time: {} sec'.format(timestamp))

            # Get current joint positions
            sensed_joint_positions = vu.get_arm_joint_positions(clientID)

            # Calculate commands
            commands = controller.calculate_commands_from_feedback(timestamp, sensed_joint_positions)

            # Send commands to V-REP
            vu.set_arm_joint_target_velocities(clientID, commands)

            # Print current joint positions (comment out if you'd like)
            # print(sensed_joint_positions)
            vu.step_sim(clientID, 1)

            # Determine if we've met the condition to move on to the next point
            steady_state_reached = controller.has_stably_converged_to_target()

            # print("\n", "Target number is ",target_number+1,"\n"," Iteration number is ",count,"\n")
            count+=1

    print("Exiting controller")
    # initial_joint_pos,initial_joint_orientation = np.asarray(loc_bb.arm_configurations(clientID))
    # print(initial_joint_pos.shape)
    # print(initial_joint_pos)
    # print(initial_joint_orientation.shape)
    # print(initial_joint_orientation)
    # np.save("initial_joint_pos",initial_joint_pos)
    # np.save("initial_joint_orientation",initial_joint_orientation)
    get_tf_bw_joint_and_bb(clientID)
    vu.stop_sim(clientID)

def get_tf_bw_joint_and_bb(clientID):
    '''
    OUTPUT: Outputs the transform between the various joints and their corresponding collision cuboid. Returns a 4X4X7 np matrix
    '''
    all_collision_cuboid_positions,all_collision_cuboid_orientations = np.asarray(loc_bb.collision_cuboid_configurations(clientID))
    all_joint_pos,all_joint_orientation = np.asarray(loc_bb.arm_configurations(clientID))
    tf_bw_joint_and_bb_list = np.zeros((4,4,all_joint_pos.shape[0]))
    for i,elt in enumerate(all_joint_pos.shape[0]):
        R = transforms3d.euler.euler2mat(all_joint_orientation[i,0], all_joint_orientation[i,1], all_joint_orientation[i,2], 'sxyz') #SOURCE OF ERROR- see xyz correct or not
        p = np.reshape(all_joint_pos[i,:],(3,1))
        H_JtoO = np.vstack(np.hstack(R,p),[[0,0,0,1]])
        R2 = transforms3d.euler.euler2mat(all_collision_cuboid_orientations[i,0], all_collision_cuboid_orientations[i,1], all_collision_cuboid_orientations[i,2], 'sxyz') #SOURCE OF ERROR- see xyz correct or not
        p2 = np.reshape(all_collision_cuboid_positions[i,:],(3,1))
        H_JtoC = np.vstack(np.hstack(R2,p2),[[0,0,0,1]])
        tf_bw_joint_and_bb_list[:,:,i] = np.matmul(H_JtoO,np.linalg.inv(H_JtoC))
    np.save("tf_bw_joint_cuboid_centroid.npy",tf_bw_joint_and_bb_list)
    pdb.set_trace()

if __name__ == "__main__":

    print ('Connecting to V-REP...')
    clientID = vu.connect_to_vrep()
    print ('Connected.')

    # Reset simulation in case something was running
    vu.reset_sim(clientID)

    #CHANGE ABSOLUTE PATH TO URDF FILE
    # urdf_xml = "/Users/harsh/Desktop/CMU_Sem_2/Robot_Autonomy/Assignments/hw1_release/locobot_description_v3.urdf"
    # robot_model = get_model_from_URDF(urdf_xml)
    initial_state_file1 = "/Users/harsh/Desktop/CMU_Sem_2/Robot_Autonomy/Assignments/hw2_release/code/utilities/initial_joint_pos.npy"
    initial_state_file2 = "/Users/harsh/Desktop/CMU_Sem_2/Robot_Autonomy/Assignments/hw2_release/code/utilities/initial_joint_orientation.npy"
    g = Graph() 
    number_of_points_to_sample = 1
    robot_model = Model()
    initialize_model(initial_state_file1,initial_state_file2,robot_model)
    #make_graph_PRM(clientID,g,robot_model,number_of_points_to_sample)
    target_positions = [[0,0,0,0,0,0,0]]
    control_locobot(target_positions,clientID)




    
