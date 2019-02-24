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
    return initial_corners

  def get_rotation_matrix(self):
    #rot_mat = R.from_rotvec([self.orientation[0], self.orientation[1], self.orientation[2]])
    rot_mat = transforms3d.euler.euler2mat(self.orientation[0], self.orientation[1], self.orientation[2], 'sxyz')
    return rot_mat

  def get_new_corners(self,rot_mat,initial_corners):
    new_corners = np.dot(rot_mat,initial_corners)
    ipdb.set_trace()
    return new_corners



#MAIN OF THE PROGRAM
if __name__ == "__main__":
    c = cuboid([0,0,0],[0,0,0],[3,1,2])
    initial_corners = c.get_initial_corners()
    rot_mat = c.get_rotation_matrix()
    new_corners = c.get_new_corners(rot_mat,initial_corners)

