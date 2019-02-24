# Import system libraries
import os
import sys
import time
import numpy as np
import ipdb
from scipy.spatial.transform import Rotation as R
import transforms3d

class cuboid:
  def __init__(self, origin, orientation, dimensions):
    self.origin = origin
    self.orientation = orientation
    self.dimensions = dimensions

  def get_initial_corners(self):
    '''
    INPUT: -
    OUTPUT: Returns the initial corners of cuboid as numpy array with rpy = 0,0,0 
    '''
    c_x = self.origin[0]
    c_y = self.origin[1]
    c_z = self.origin[2]

    d_x_half = self.dimensions[0]/2
    d_y_half = self.dimensions[1]/2
    d_z_half = self.dimensions[2]/2

    d_x = [self.origin[0] + d_x_half,self.origin[0] - d_x_half]
    d_y = [self.origin[1] + d_y_half,self.origin[1] - d_y_half]
    d_z = [self.origin[2] + d_z_half,self.origin[2] - d_z_half]

    initial_corners = []

    for elt_x in d_x:
        for elt_y in d_y:
            for elt_z in d_z:
                initial_corners.append([elt_x,elt_y,elt_z])

    initial_corners = np.transpose(np.asarray(initial_corners))
    print("Old corner points are: \n",initial_corners,"\n")
    return initial_corners

  def get_rotation_matrix(self):
    '''
    INPUT: -
    OUTPUT: Returns the Rotation matrix as a numpy array
    '''
    #rot_mat = R.from_rotvec([self.orientation[0], self.orientation[1], self.orientation[2]])
    rot_mat = transforms3d.euler.euler2mat(self.orientation[0], self.orientation[1], self.orientation[2], 'sxyz')
    return rot_mat

  def get_new_corners(self):
    '''
    INPUT: -
    OUTPUT: Returns the new corners as 3X8 numpy array taking into account the rotation matrix
    '''
    initial_corners = self.get_initial_corners()
    rot_mat = self.get_rotation_matrix()
    new_corners = np.dot(rot_mat,initial_corners)
    print("New corner points are: \n",new_corners,"\n")
    return new_corners

def collision_check_along_cuboid_normals(new_corners,points_2):    #EDIT CODE FOR OPTIMISING IT. CHECK ONE NORMAL AND THEN IF SEPERATION EXISTS COMMENT NP COLLISION
    '''
    INPUT: Takes in the new corners of the two cuboids. 
    OUTPUT: Returns if there is a collision with the normals of one of the cuboids face. True implies collision along all 3 faces
    Finds the normal creating points of one of the cuboids and then checks for collision about those normals
    '''
    normal_1 = new_corners[:,0:3]
    if(not points_to_collision(new_corners,points_2,normal_1)):
        return False

    normal_2 = np.hstack((np.vstack(new_corners[:,0:2]),np.reshape(new_corners[:,5],(3,1))))    #Why is vertical stack working in the interior bracket?
    if(not points_to_collision(new_corners,points_2,normal_2)):
        return False   

    normal_3 = np.transpose(np.reshape(np.hstack((np.hstack((new_corners[:,0],new_corners[:,2])),new_corners[:,6])),(3,3)))
    if(not points_to_collision(new_corners,points_2,normal_3)):
        return False   

    return True

def get_projections_from_normal(normal,points):
    '''
    INPUT: Takes in the normal generating points and the corner points of a cuboid
    OUTPUT: Returns the max and min projection along the normal
    This function converts the normal points to a normal. Makes a unit vector of the normal and then finds the max and min projection of the new
    corners along this normal
    '''
    normal = np.cross((normal[:,2] - normal[:,0]),(normal[:,1] - normal[:,0]))
    try:
        normal = normal/np.linalg.norm(normal)
        normal = np.reshape(normal,(1,3))
        print("Unit normal is \n",normal,"\n")
        projections = np.dot(normal,points)
        return np.amax(projections),np.amin(projections)
    except:
        print("Incorrect normal vector. Error in get_projection_from_normal")
        return None,None

def collision_check_along_normal(max1,min1,max2,min2):
    '''
    INPUT: Takes the max and the min projections of two cuboids
    OUTPUT: Returns True if there is a collision and False if a seperation exists
    '''
    if((min1<=min2 and min2<=max1) or (min1<=max2 and max2<=max1) or (min2<=min1 and min1<=max2) or(min2<=max1 and max1<=max2)):
        return True
    else:
        return False

def points_to_collision(points_1,points_2,normal):
    '''
    INPUT: Takes in the normal generating points and the corners of two cuboids
    OUTPUT: Returns if there is a collision with the normal of one of the cuboid's face. 
    '''
    max1,min1 = get_projections_from_normal(normal,points_1)
    max2,min2 = get_projections_from_normal(normal,points_2)
    #print("\n Normal is \n",normal,"\n")
    print(max1,"\t",min1,"\t",max2,"\t",min2,"\n")
    #ipdb.set_trace()
    return collision_check_along_normal(max1,min1,max2,min2)

def check_for_collision(cuboid1,cuboid2):
    points1 = cuboid1.get_new_corners()
    points2 = cuboid2.get_new_corners()
    if(collision_check_along_cuboid_normals(points1,points2) and collision_check_along_cuboid_normals(points2,points1)):
        return True
    else:
        return False

#MAIN OF THE PROGRAM
if __name__ == "__main__":
    reference_cuboid = cuboid([0,0,0],[0,0,0],[3,1,2])

    #test_1 = cuboid([0,1,0],[0,0,0],[0.8,0.8,0.8])
    # test_2 = cuboid([1.5,-1.5,0],[1,0,1.5],[1,3,3])
    test_3 = cuboid([0,0,-1],[0,0,0],[2,3,1])
    # test_4 = cuboid([3,0,0],[0,0,0],[3,1,1])
    # test_5 = cuboid([-1,0,-2],[-5,0,0.4],[2,0.7,2])
    # check = cuboid([10,10,10],[0,0,0],[3,1,2])
    #c = cuboid([0,0,0],[0,0,0],[3,1,2])

    if(check_for_collision(reference_cuboid,test_3)):
        print("In collision \n")
    else:
        print("Not colliding \n")

