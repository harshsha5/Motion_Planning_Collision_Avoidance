import os
import sys
import time
import numpy as np
#import ipdb
#import transforms3d
import vrep_utils as vu


def get_collision_cuboid_orientations(collision_cuboid_handles,clientID):
	all_collision_cuboid_orientations = []
	for elt in collision_cuboid_handles:
		all_collision_cuboid_orientations.append(vu.get_object_orientation(clientID, elt, -1))

	all_collision_cuboid_orientations = np.asarray(all_collision_cuboid_orientations)

	return all_collision_cuboid_orientations

def get_collision_cuboid_positions(collision_cuboid_handles,clientID):
	all_collision_cuboid_positions = []
	for elt in collision_cuboid_handles:
		all_collision_cuboid_positions.append(vu.get_object_position(clientID, elt, -1))

	all_collision_cuboid_positions = np.asarray(all_collision_cuboid_positions)
	return all_collision_cuboid_positions

def get_collision_cuboid_bounding_boxes(collision_cuboid_handles,clientID):
	'''
	OUTPUT: Returns a 7X2X3 numpy array. Where 7 is the number of joints, 2 is the max and min. 3 is the x,y,z
	'''
	all_collision_cuboid_bounding_boxes = []
	for elt in collision_cuboid_handles:
		all_collision_cuboid_bounding_boxes.append(vu.get_object_bounding_box(clientID, elt))

	all_collision_cuboid_bounding_boxes = np.asarray(all_collision_cuboid_bounding_boxes)
	return all_collision_cuboid_bounding_boxes

def collision_cuboid_configurations(clientID):
    collision_cuboid_handles = vu.get_collision_cuboid_handles(clientID)
    all_collision_cuboid_positions = get_collision_cuboid_positions(collision_cuboid_handles,clientID)
    print(all_collision_cuboid_positions.shape)
    all_collision_cuboid_orientations = get_collision_cuboid_orientations(collision_cuboid_handles,clientID)
    all_collision_cuboid_bounding_boxes = get_collision_cuboid_bounding_boxes(collision_cuboid_handles,clientID)
    print(all_collision_cuboid_bounding_boxes.shape)

# def get_arm_orientations(arm_handles,clientID):
# 	all_arm_orientations = []
# 	for elt in arm_handles:
# 		all_arm_orientations.append(vu.get_object_orientation(clientID, elt, -1))

# 	all_arm_orientations = np.asarray(all_arm_orientations)

# 	return all_arm_orientations

# def get_arm_bounding_boxes(arm_handles,clientID):
# 	'''
# 	OUTPUT: Returns a 7X2X3 numpy array. Where 7 is the number of joints, 2 is the max and min. 3 is the x,y,z
# 	'''
# 	all_arm_bounding_boxes = []
# 	for elt in arm_handles:
# 		all_arm_bounding_boxes.append(vu.get_object_bounding_box(clientID, elt))

# 	all_arm_bounding_boxes = np.asarray(all_arm_bounding_boxes)
# 	return all_arm_bounding_boxes

# def arm_configurations(clientID):
#     arm_handles = vu.get_arm_joint_handles(clientID)
#     all_arm_positions = get_arm_positions(arm_handles,clientID)
#     print(all_arm_positions.shape)
#     all_arm_orientations = get_arm_orientations(arm_handles,clientID)
#     all_arm_bounding_boxes = get_arm_bounding_boxes(arm_handles,clientID)

if __name__ == "__main__":
    # Connect to V-REP
    print ('Connecting to V-REP...')
    clientID = vu.connect_to_vrep()
    print ('Connected.')

    # Reset simulation in case something was running
    vu.reset_sim(clientID)

    #Get the joint bounding box configurations
    collision_cuboid_configurations(clientID)

    # arm_handles = vu.get_arm_joint_handles(clientID)
    # all_arm_positions = get_arm_positions(arm_handles,clientID)
    # all_arm_orientations = get_arm_orientations(arm_handles,clientID)
    # all_arm_bounding_boxes = get_arm_bounding_boxes(arm_handles,clientID)
