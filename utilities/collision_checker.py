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
    c_x = self.origin[0]
    c_y = self.origin[1]
    c_z = self.origin[2]

    d_x_half = self.dimensions[0]/2
    d_y_half = self.dimensions[1]/2
    d_z_half = self.dimensions[2]/2

    d_x = [d_x_half,-1*d_x_half]
    d_y = [d_y_half,-1*d_y_half]
    d_z = [d_z_half,-1*d_z_half]

    initial_corners = []

    for elt_x in d_x:
        for elt_y in d_y:
            for elt_z in d_z:
                initial_corners.append([elt_x,elt_y,elt_z])

    initial_corners = np.transpose(np.asarray(initial_corners))
    print("Old corner points are: \n",initial_corners,"\n")
    return initial_corners

  def get_rotation_matrix(self):
    #rot_mat = R.from_rotvec([self.orientation[0], self.orientation[1], self.orientation[2]])
    rot_mat = transforms3d.euler.euler2mat(self.orientation[0], self.orientation[1], self.orientation[2], 'sxyz')
    return rot_mat

  def get_new_corners(self):
    initial_corners = self.get_initial_corners()
    rot_mat = self.get_rotation_matrix()
    new_corners = np.dot(rot_mat,initial_corners)
    print("New corner points are: \n",new_corners,"\n")
    return new_corners

def collision_check_along_cuboid_normals(new_corners,points_2):    #EDIT CODE FOR OPTIMISING IT. CHECK ONE NORMAL AND THEN IF SEPERATION EXISTS COMMENT NP COLLISION
    normal_1 = new_corners[:,0:3]
    if(!points_to_collision(new_corners,points_2,normal_1)):
        return False

    normal_2 = np.hstack((np.vstack(new_corners[:,0:2]),np.reshape(new_corners[:,5],(3,1))))    #Why is vertical stack working in the interior bracket?
    if(!points_to_collision(new_corners,points_2,normal_2)):
        return False   

    normal_3 = np.transpose(np.reshape(np.hstack((np.hstack((new_corners[:,0],new_corners[:,2])),new_corners[:,6])),(3,3)))
    if(!points_to_collision(new_corners,points_2,normal_3)):
        return False   

    return True

def get_projections_from_normal(normal,points):
    print("Actual normal is \n",normal,"\n")
    normal = np.cross((normal[:,2] - normal[:,0]),(normal[:,1] - normal[:,0]))
    try:
        normal = normal/np.linalg.norm(normal)
        normal = np.reshape(normal,(1,3))
        projections = np.dot(normal,points)
        return np.amax(projections),np.amin(projections)
    except:
        print("Incorrect normal vector. Error in get_projection_from_normal")
        return None,None

def collision_check_along_normal(max1,min1,max2,min2):
    if((min1<=min2 and min2<=max1) or (min1<=max2 and max2<=max1) or (min2<=min1 and min1<=max2) or(min2<=max1 and max1<=max2)):
        return True
    else:
        return False

def points_to_collision(points_1,points_2,normal):
    max1,min1 = get_projections_from_normal(normal,points_1)
    max2,min2 = get_projections_from_normal(normal,points_2)
    ipdb.set_trace()
    return collision_check_along_normal(max1,min1,max2,min2)

def check_for_collision(cuboid1,cuboid2):
    

    pass

#MAIN OF THE PROGRAM
if __name__ == "__main__":
    reference_cuboid = cuboid([0,0,0],[0,0,0],[3,1,2])

    test_1 = cuboid([0,1,0],[0,0,0],[0.8,0.8,0.8])
    #c = cuboid([0,0,0],[0,0,0],[3,1,2])

    reference_new_corners = reference_cuboid.get_new_corners()
    list1 = get_points_for_normals(reference_new_corners)

    #test1_new_corners = test_1.get_new_corners()

