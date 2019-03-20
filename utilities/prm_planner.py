import os
import sys
import time
import numpy as np
#import ipdb
import transforms3d
import vrep_utils as vu
from random import random
#from random import seed
import locobot_bounding_box as loc_bb
import collision_checker
from collections import defaultdict 
from urdf_parser_py.urdf import URDF
import locobot_joint_ctrl as ctrl
import pdb
import pickle
import dijkastra
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

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

    def show_graph(self):
        print(self.graph.items())

    def edit_graph_for_not_connected_vertices(self,vertex_state_list):
        new_graph = defaultdict(list) 
        new_state_list = []
        vertices_removed = []
        count = 0
        for key,value in self.graph.items():
            if(not len(value)==0):
                new_graph[count] = value
                new_state_list.append(vertex_state_list[key])
                count+=1
            else:
                vertices_removed.append(key)
        print("Removed vertices are: \n")
        print(vertices_removed)
        self.graph = new_graph
        return new_state_list

class Model:
    def __init__(self): 
  
        self.position = None
        self.rotation = None
        self.axis = None

    def initialize_model_using_np(self,initial_joint_pos,initial_joint_orientation,initial_rotation):
        self.position = initial_joint_pos
        self.rotation = initial_joint_orientation
        self.axis = initial_rotation


def get_forward_kinematics(joint_angles,robot_model,tf_bw_joint_and_next_joint):
    '''
    Returns a list of H_OtoJ
    '''
    homo_matrix_list = []
    present_net_homo = np.identity(4)
    running_homo = np.identity(4)
    for i in range(joint_angles.shape[0]):        #Note change of -2 (Since the fingers don't rotate)
        # rpy = robot_model.rotation[i,:]                               #Assume angles provided by model is in Euler format
        # net_rotation = rpy + joint_angles[i]*robot_model.axis[i,:]
        if(i<=4):
            net_rotation = joint_angles[i]*robot_model.axis[i,:]
        else:
            net_rotation = np.array([0,0,0])
        R = transforms3d.euler.euler2mat(net_rotation[0], net_rotation[1], net_rotation[2], 'szyx') #SOURCE OF ERROR- see xyz correct or not
        #p = np.reshape(robot_model.position[i,:],(3,1)) 
        p = np.array([[0],[0],[0]])
        # pdb.set_trace()
        homo_temp = np.vstack((np.hstack((R,p)),np.array([0,0,0,1])))
        running_homo = np.dot(running_homo,tf_bw_joint_and_next_joint[:,:,i])
        present_net_homo = np.dot(running_homo,homo_temp)
        #present_net_homo = np.dot(present_net_homo,homo_temp)
        homo_matrix_list.append(present_net_homo)
    return homo_matrix_list

#Find H0toJn use HJtoCn to find H0toCn
#Do we neglect the last two joints (the fingers)

#I want this to return HJtoO, so that I can then use my already computed HJtoC to get HCtoO, which can then be sent to check for collisions

# def get_bb_transforms(clientID):
#     homo_cuboid_list =[]
#     cuboid_pos,cuboid_orientation,cuboid_bb = loc_bb.collision_cuboid_configurations(clientID)
#     for i,elt in enumerate():

def get_random_joint_angles():      #Edit this so that it returns a 1 X 7 numpy array
    # random.randrange(-90, 90.1, 0.5)
    return np.random.uniform(-np.pi/2,np.pi/2,5)          #THE JOINT LIMITS ARE HARD CODED AS OF NOW. CHANGE WRT THE URDF

def add_state_to_graph_as_vertex(g,vertex_number):
    '''
    This function simply adds the vertex to the graph (with no edge)
    '''
    g.addVertex(vertex_number)
    #see if return by value or by reference

def initialize_graph(clientID,g,vertex_state_list):
    initial_joint_angles = vu.get_arm_joint_positions(clientID)
    vertex_state_list.append(np.asarray(initial_joint_angles))
    add_state_to_graph_as_vertex(g,0)

# def get_nearest_neighbours(vertex_list,S):
#     dist = map(lambda x,y:x+y, a,b)
# dist = numpy.linalg.norm(a-b)

def check_if_state_is_collision_free(clientID,random_joint_angles,robot_model,robot_cuboid_dimensions,tf_bw_joint_cuboid_centroid,static_cuboid_list,tf_bw_joint_and_next_joint):
    '''
    INPUT: random_joint_angles- (7,) Numpy vector
           robot_model a Model object
           robot_cuboid_dimensions- A 7X3 Matrix. That is 7 joints and x,y,z dimensions
           tf_bw_joint_cuboid_centroid- A 4X4X7 Matrix. ie. Homogeneous matrix for transforms b/w joint to cuboid for all the 7 joints
    OUTPUT: Returns True if state is collision free
    '''
    joint_cuboid_list =[]
    homo_matrix_list = get_forward_kinematics(random_joint_angles,robot_model,tf_bw_joint_and_next_joint)
    for i,elt in enumerate(homo_matrix_list):
        # H_base_to_cc = np.dot(np.linalg.inv(tf_bw_joint_cuboid_centroid[:,:,i]),elt)
        H_base_to_cc = np.dot(elt,tf_bw_joint_cuboid_centroid[:,:,i])
        al, be, ga = transforms3d.euler.mat2euler(H_base_to_cc, 'sxyz')
        pos = H_base_to_cc[0:3,3]
        pos = pos.tolist()
        dim = robot_cuboid_dimensions[i,:].tolist()
        cuboid_temp = collision_checker.cuboid(pos,[al,be,ga],dim)
        joint_cuboid_list.append(cuboid_temp)
    # pdb.set_trace()
    for i,elt in enumerate(joint_cuboid_list):
        for j,elo in enumerate(static_cuboid_list):
            # print(i,j)
            # pdb.set_trace()
            if(collision_checker.check_for_collision_between_cuboids(elt,elo)):
                return False

    '''CHECK FOR SELF COLLISION'''
    # for i in range(len(joint_cuboid_list)-1):
    #     for j in range(1+len(joint_cuboid_list)):
    #         if(collision_checker.check_for_collision_between_cuboids(joint_cuboid_list[i],joint_cuboid_list[j])):
    #             return False

    return True

def make_graph_PRM(clientID,g,robot_model,number_of_points_to_sample = 100):
    '''
    The graph phase of the PRM
    OUTPUT: Will return the graph g and the vertex_state_list which is a numpy list of angles of the various vertices
    Note: The number_of_points_to_sample indicate that these many vertices should be accepted. It does NOT mean that many vertices
            will be sampled. So count only increases if a vertex is accepted. 
    '''
    #Tunable Parameters
    nearest_neighbours_to_take = 3  #This specifies the number of nearest neighbors to consider

    #Initializations
    vertex_state_list = []
    count = 0

    #Get information
    arm_handles = vu.get_arm_joint_handles(clientID)
    robot_cuboid_dimensions = get_robot_cuboid_dimensions(clientID)
    static_cuboid_list = get_static_cuboids(clientID)
    tf_bw_joint_cuboid_centroid = np.load("tf_bw_joint_cuboid_centroid.npy")
    tf_bw_joint_and_next_joint = np.load("tf_bw_joint_and_next_joint.npy")
    # pdb.set_trace()

    #initialize_graph(clientID,g,vertex_state_list) #Remove this line

    while count< number_of_points_to_sample:
        random_joint_angles = get_random_joint_angles()
        finger_pos = np.array([-0.03,0.03])                              #Initial angle for fingers. MUST CHANGE as required!
        random_joint_angles = np.hstack((random_joint_angles,finger_pos))
        # random_joint_angles = np.array([0,1.0472,-1.309,-1.309,0,-0.03,0.03])
        # random_joint_angles = np.array([-1.3955,0,0,0,0,-0.03,0.03])
        # joint_positions =vu.get_arm_joint_positions(clientID)
        #previous_joint_positions = joint_positions
        # print("Vertex sampled is ",random_joint_angles,"\n")
        if(check_if_state_is_collision_free(clientID,random_joint_angles,robot_model,robot_cuboid_dimensions,tf_bw_joint_cuboid_centroid,static_cuboid_list,tf_bw_joint_and_next_joint)):   
            print("Vertex ",count," added to graph","\n")
            add_state_to_graph_as_vertex(g,count)
            vertex_state_list.append(random_joint_angles)
            count+=1
            if(len(vertex_state_list)==1):
                continue
            nearest_neighbours = get_nearest_neighbours(vertex_state_list,random_joint_angles,nearest_neighbours_to_take)
            # print("Vertex added to graph")

            for neighbor in nearest_neighbours:
                if(connect(neighbor,vertex_state_list,clientID,random_joint_angles,robot_model,robot_cuboid_dimensions,tf_bw_joint_cuboid_centroid,static_cuboid_list,tf_bw_joint_and_next_joint)):
                    cost = np.linalg.norm(vertex_state_list[neighbor] - random_joint_angles)
                    g.addEdge(neighbor,[len(vertex_state_list)-1,cost])
                    g.addEdge(len(vertex_state_list)-1,[neighbor,cost])
        else:
            print("Vertex rejected")

    print("Done")
    # g.show_graph()
    # print("\n")
    # print(vertex_state_list)
    #vertex_state_list = g.edit_graph_for_not_connected_vertices(vertex_state_list)
    g.show_graph()
    # print("\n")
    # print(vertex_state_list)
    # pdb.set_trace()
    return g,vertex_state_list

def connect(neighbor,vertex_state_list,clientID,random_joint_angles,robot_model,robot_cuboid_dimensions,tf_bw_joint_cuboid_centroid,static_cuboid_list,tf_bw_joint_and_next_joint):
    '''
    INPUT: int neighbor (Index of the neighbour in vertex_state_list) 
           vertex_state_list (The entire list of states)
    OUTPUT: Returns true if path between the two positions is collision free and thus, the two points can be connected via edges

    You have two positions in the Cfree space, we take the difference of these and discretize the path in between these two positions in discretization_steps
    number of steps and check for collision of each.
    '''
    discretization_steps = 20
    step_size_for_each_joint = (vertex_state_list[neighbor] - vertex_state_list[-1])/discretization_steps
    for i in range(discretization_steps):
        new_position = vertex_state_list[-1] + (i+1)*step_size_for_each_joint 
        if(not (check_if_state_is_collision_free(clientID,new_position,robot_model,robot_cuboid_dimensions,tf_bw_joint_cuboid_centroid,static_cuboid_list,tf_bw_joint_and_next_joint))):
            # print("Connect test failed for ",i+1, " iteration ","\n")
            return False
    print("Vertex ",neighbor," and ",len(vertex_state_list)-1," connected \n")
    return True

def connect_start_and_goal_with_graph(g,vertex_state_list,START_ROBOT_POSITION,GOAL_ROBOT_POSITION,clientID,robot_model):
    #Tunable Parameters
    nearest_neighbours_to_take = len(vertex_state_list)  #This specifies the number of nearest neighbors to consider

    #Get information
    arm_handles = vu.get_arm_joint_handles(clientID)
    robot_cuboid_dimensions = get_robot_cuboid_dimensions(clientID)
    static_cuboid_list = get_static_cuboids(clientID)
    tf_bw_joint_cuboid_centroid = np.load("tf_bw_joint_cuboid_centroid.npy")
    tf_bw_joint_and_next_joint = np.load("tf_bw_joint_and_next_joint.npy")

    if(not check_if_state_is_collision_free(clientID,START_ROBOT_POSITION,robot_model,robot_cuboid_dimensions,tf_bw_joint_cuboid_centroid,static_cuboid_list,tf_bw_joint_and_next_joint)): 
        print("Start position in collision")
        return g,vertex_state_list,-1

    if(not check_if_state_is_collision_free(clientID,GOAL_ROBOT_POSITION,robot_model,robot_cuboid_dimensions,tf_bw_joint_cuboid_centroid,static_cuboid_list,tf_bw_joint_and_next_joint)): 
        print("Goal position in collision")
        return  g,vertex_state_list,-1

    add_state_to_graph_as_vertex(g,len(vertex_state_list))  
    vertex_state_list.append(START_ROBOT_POSITION)      #Adding the start robot position to the vertex_state_list. This will ultimately be the second last element in the list, goal being the last.
    if(len(vertex_state_list)==1):
            pass
    else:
        nearest_neighbours = get_nearest_neighbours(vertex_state_list,START_ROBOT_POSITION,nearest_neighbours_to_take)

        for neighbor in nearest_neighbours:
            if(connect(neighbor,vertex_state_list,clientID,START_ROBOT_POSITION,robot_model,robot_cuboid_dimensions,tf_bw_joint_cuboid_centroid,static_cuboid_list,tf_bw_joint_and_next_joint)):
                cost = np.linalg.norm(vertex_state_list[neighbor] - START_ROBOT_POSITION)
                g.addEdge(neighbor,[len(vertex_state_list)-1,cost])
                g.addEdge(len(vertex_state_list)-1,[neighbor,cost])
                break

    add_state_to_graph_as_vertex(g,len(vertex_state_list))  
    vertex_state_list.append(GOAL_ROBOT_POSITION)      #Adding the goal robot position to the vertex_state_list as the last index. 

    nearest_neighbours = get_nearest_neighbours(vertex_state_list,GOAL_ROBOT_POSITION,nearest_neighbours_to_take)

    for neighbor in nearest_neighbours:
        if(connect(neighbor,vertex_state_list,clientID,GOAL_ROBOT_POSITION,robot_model,robot_cuboid_dimensions,tf_bw_joint_cuboid_centroid,static_cuboid_list,tf_bw_joint_and_next_joint)):
            cost = np.linalg.norm(vertex_state_list[neighbor] - GOAL_ROBOT_POSITION)
            g.addEdge(neighbor,[len(vertex_state_list)-1,cost])
            g.addEdge(len(vertex_state_list)-1,[neighbor,cost])
            break
    print("Connect start and end with graph executed \n")
    print("\n")
    g.show_graph()
    print("\n")
    # print(vertex_state_list)
    return g,vertex_state_list,1

def get_nearest_neighbours(vertex_state_list,sampled_vertex,nearest_neighbours_to_take = 5):
    '''
    Input: The list of all the vertices and the newly sampled vertex
    Output: The indexes of the 5 closest vertices in the form of a numpy array
    '''

    vertex_state_list = np.asarray(vertex_state_list[:-1])
    dist = np.linalg.norm(vertex_state_list-sampled_vertex,axis=1) #dist is expected to be a column numpy array
    vertex_list_by_distance = np.argsort(dist)
    if(vertex_list_by_distance.shape[0]>nearest_neighbours_to_take-1):     
        return vertex_list_by_distance[0:nearest_neighbours_to_take]
    else:
        return vertex_list_by_distance

def get_nearest_neighbours_for_start_and_goal(vertex_state_list,sampled_vertex,nearest_neighbours_to_take = 5):
    '''
    Input: The list of all the vertices and the newly sampled vertex
    Output: The indexes of the 5 closest vertices in the form of a numpy array
    '''

    vertex_state_list = np.asarray(vertex_state_list[:-1])
    dist = np.linalg.norm(vertex_state_list-sampled_vertex,axis=1) #dist is expected to be a column numpy array
    vertex_list_by_distance = np.argsort(dist)
    if(vertex_list_by_distance.shape[0]>nearest_neighbours_to_take-1):     
        return vertex_list_by_distance[0:nearest_neighbours_to_take]
    else:
        return vertex_list_by_distance


def get_static_cuboids(clientID):            
    '''
    OUTPUT: Returns the list of static cuboids as objects of the class cuboid (in collision checker)
    '''
    all_static_cuboid_positions,all_static_cuboid_orientations,static_cuboid_bounding_boxes_dimensions = loc_bb.static_cuboid_configurations(clientID)
    '''
    all_static_cuboid_positions: a 6X3 Numpy array, for 6 static cuboids and x,y,z ie. 3
    all_static_cuboid_orientations: a 6X3 Numpy array, for 6 static cuboids and x,y,z ie. 3
    static_cuboid_bounding_boxes_dimensions: a 6X3 Numpy array, Dimension of x,y,z for 6 cuboids
    '''
    static_cuboid_list = []
    for i in range(all_static_cuboid_positions.shape[0]):
        static_cuboid_list.append(collision_checker.cuboid(all_static_cuboid_positions[i,:].tolist(),all_static_cuboid_orientations[i,:].tolist(),static_cuboid_bounding_boxes_dimensions[i,:].tolist()))   
    return static_cuboid_list

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
    #joint_axis = np.array([[0,0,1],[0,0,1],[0,0,1],[0,0,1],[0,0,1],[0,0,1],[0,0,1]])
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

    controller.plot_joints()
    print("Exiting controller")
    # get_tf_bw_joint_and_bb(clientID)
    # initial_joint_pos,initial_joint_orientation = np.asarray(loc_bb.arm_configurations(clientID))
    # print(initial_joint_pos.shape)
    # print(initial_joint_pos)
    # print(initial_joint_orientation.shape)
    # print(initial_joint_orientation)
    # np.save("initial_joint_pos",initial_joint_pos)
    # np.save("initial_joint_orientation",initial_joint_orientation)
    vu.stop_sim(clientID)

def get_tf_bw_joint_and_bb(clientID):       
    '''
    OUTPUT: Outputs the transform between the various joints and their corresponding collision cuboid. 
    The function saves two 4X4X7 np matrix. One stores the tf b/w joint and collision cuboid centroid. The other stores tf b/w joint and joint+1
    '''

    #all_collision_cuboid_positions,all_collision_cuboid_orientations = np.asarray(loc_bb.collision_cuboid_configurations(clientID))
    #all_joint_pos,all_joint_orientation = np.asarray(loc_bb.arm_configurations(clientID))
    all_joint_pos = np.array([[-0.0896, 0.0003875, 0.159], [0,0,0.04125], [0.05, 0, 0.2413], [0.2492, 0,0.2413], [0.063, 0.0001, 0.2413], [0.386, -0.02399, 0.2463], [0.386, 0.02601, 0.2463]])

    #np.load("/Users/harsh/Desktop/CMU_Sem_2/Robot_Autonomy/Assignments/hw2_release/code/utilities/cuboid_initial_cuboid_dim.npy")
    all_collision_cuboid_positions = np.load("/Users/harsh/Desktop/CMU_Sem_2/Robot_Autonomy/Assignments/hw2_release/code/utilities/cuboid_initial_cuboid_origin.npy")
    all_collision_cuboid_orientations = np.load("/Users/harsh/Desktop/CMU_Sem_2/Robot_Autonomy/Assignments/hw2_release/code/utilities/cuboid_initial_cuboid_rpy.npy")

    tf_bw_joint_and_bb_list = np.zeros((4,4,all_joint_pos.shape[0]))
    tf_bw_joint_and_next_joint = np.zeros((4,4,all_joint_pos.shape[0]))
    running_homo = np.identity(4)

    for i in range(all_joint_pos.shape[0]):
        R = transforms3d.euler.euler2mat(0,0,0, 'sxyz') #SOURCE OF ERROR- see xyz correct or not
        if(not i==0):
            p = np.reshape(all_joint_pos[i,:],(3,1)) + np.reshape(all_joint_pos[0,:],(3,1))
            # print(p)
        else:
            p = np.reshape(all_joint_pos[i,:],(3,1))
        H_OtoJ = np.vstack((np.hstack((R,p)),np.array([0,0,0,1])))
        # pdb.set_trace()

        R2 = transforms3d.euler.euler2mat(all_collision_cuboid_orientations[i,0], all_collision_cuboid_orientations[i,1], all_collision_cuboid_orientations[i,2], 'sxyz') #SOURCE OF ERROR- see xyz correct or not
        p2 = np.reshape(all_collision_cuboid_positions[i,:],(3,1))
        H_OtoC = np.vstack((np.hstack((R2,p2)),np.array([0,0,0,1])))
        # H_JtoC = np.vstack((np.hstack((R2,p2)),np.array([0,0,0,1])))
        # tf_bw_joint_and_bb_list[:,:,i] = np.dot(H_JtoO,np.linalg.inv(H_CtoO))
        tf_bw_joint_and_bb_list[:,:,i] = np.dot(np.linalg.inv(H_OtoJ),H_OtoC)

        if(i==0):
            tf_bw_joint_and_next_joint[:,:,i] = H_OtoJ
        else:
            tf_bw_joint_and_next_joint[:,:,i] = np.dot(np.linalg.inv(running_homo),H_OtoJ)
            # tf_bw_joint_and_next_joint[:,:,i] = np.dot(np.linalg.inv(tf_bw_joint_and_next_joint[:,:,i-1]),H_OtoJ)

        # pdb.set_trace()
        running_homo = np.dot(running_homo,tf_bw_joint_and_next_joint[:,:,i])
        # pdb.set_trace()

    np.save("tf_bw_joint_cuboid_centroid.npy",tf_bw_joint_and_bb_list)
    np.save("tf_bw_joint_and_next_joint.npy",tf_bw_joint_and_next_joint)
    print("tf captured and saved")

#H0toJ

def get_target_positions(path,vertex_state_list):
    target_positions = []
    for index in path:
        target_positions.append(vertex_state_list[index].tolist())

    return target_positions


if __name__ == "__main__":

    print ('Connecting to V-REP...')
    clientID = vu.connect_to_vrep()
    print ('Connected.')

    # Reset simulation in case something was running
    vu.reset_sim(clientID)

    #CHANGE ABSOLUTE PATH TO URDF FILE
    # urdf_xml = "/Users/harsh/Desktop/CMU_Sem_2/Robot_Autonomy/Assignments/hw1_release/locobot_description_v3.urdf"
    # robot_model = get_model_from_URDF(urdf_xml)

    #CHANGE THIS PATH TO ABSOLUTE!
    initial_state_file1 = "initial_joint_pos.npy"
    initial_state_file2 = "initial_joint_orientation.npy"
    deg_to_rad = np.pi/180

    #Initializations
    g = Graph() 
    number_of_points_to_sample = 30
    robot_model = Model()
    initialize_model(initial_state_file1,initial_state_file2,robot_model)

    #gazzab = np.load("/Users/harsh/Desktop/CMU_Sem_2/Robot_Autonomy/Assignments/hw2_release/code/utilities/cuboid_initial_cuboid_dim.npy")
    ## np.load("/Users/harsh/Desktop/CMU_Sem_2/Robot_Autonomy/Assignments/hw2_release/code/utilities/cuboid_initial_cuboid_origin.npy")
    ## np.load("/Users/harsh/Desktop/CMU_Sem_2/Robot_Autonomy/Assignments/hw2_release/code/utilities/cuboid_initial_cuboid_rpy.npy")

    #Preprocessing Phase: Make the PRM graph
    g,vertex_state_list = make_graph_PRM(clientID,g,robot_model,number_of_points_to_sample)

    #Enter start and end position of the robot in degrees for revolute joints and in meters for prismatic joints
    START_ROBOT_POSITION = np.array([-80,0,0,0,0,-0.03,0.03])   #Note: The last two joints are prismatic
    # START_ROBOT_POSITION = np.array([0,55,-75,-75,0,-0.03,0.03])  #Note: The last two joints are prismatic
    GOAL_ROBOT_POSITION = np.array([0,60,-75,-75,0,-0.03,0.03]) #Note: The last two joints are prismatic

    #degree to radians conversions for angles
    START_ROBOT_POSITION[0:5] = np.radians(START_ROBOT_POSITION[0:5])
    GOAL_ROBOT_POSITION[0:5] = np.radians(GOAL_ROBOT_POSITION[0:5])

    print("length of vertex state list is \n",len(vertex_state_list))

    #Query Phase: Connect start and goal positions to the graph
    g,vertex_state_list,flag = connect_start_and_goal_with_graph(g,vertex_state_list,START_ROBOT_POSITION,GOAL_ROBOT_POSITION,clientID,robot_model)
    print("length of vertex state list is \n",len(vertex_state_list))
    if(flag==1):
        source = len(vertex_state_list)-2
        goal = len(vertex_state_list)-1
        path = dijkastra.dijkstra(g,source, goal)
        print(path)

        target_positions = get_target_positions(path,vertex_state_list)
    else:
        print("Incorrect Query given")

    #Controls Phase (Provide angles in radians)
    # target_positions = [[0,0,0,0,0,-0.03,0.03]]
    # target_positions = [[-90*deg_to_rad,0,0,0,0,-0.03,0.03]]
    print("Path found ")
    pdb.set_trace()
    control_locobot(target_positions,clientID)





    
