import os
import sys
import time
import numpy as np
#import ipdb
#import transforms3d
import vrep_utils as vu

def get_arm_orientations(arm_handles):
	all_arm_orientations = []
	for elt in arm_handles:
		all_arm_orientations.append(vu.get_object_orientation(clientID, elt, -1))

	return all_arm_orientations

def get_arm_positions(arm_handles):
	all_arm_positions = []
	for elt in arm_handles:
		all_arm_positions.append(vu.get_object_position(clientID, elt, -1))

	return all_arm_positions

def get_arm_bounding_boxes(arm_handles):
	all_arm_bounding_boxes = []
	for elt in arm_handles:
		all_arm_bounding_boxes.append(vu.get_object_bounding_box(clientID, elt))

	return all_arm_bounding_boxes

if __name__ == "__main__":
    # Connect to V-REP
    print ('Connecting to V-REP...')
    clientID = vu.connect_to_vrep()
    print ('Connected.')

    # Reset simulation in case something was running
    vu.reset_sim(clientID)

    #Get the arm joint handles
    arm_handles = vu.get_arm_joint_handles(clientID)

    # all_arm_positions = get_arm_positions(arm_handles)
    # print(all_arm_positions)

    # all_arm_orientations = get_arm_orientations(arm_handles)
    # print(all_arm_orientations)

    all_arm_bounding_boxes = get_arm_bounding_boxes(arm_handles)
    print(all_arm_bounding_boxes)
    print(len(all_arm_bounding_boxes),len(all_arm_bounding_boxes[0]))