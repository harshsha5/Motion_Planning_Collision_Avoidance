# Import system libraries
import os
import sys
import time
import numpy as np

class cuboid:
  def __init__(self, origin, orientation, dimensions):
    self.origin = origin
    self.orientation = orientation
    self.dimensions = dimensions

  def get_initial_corners(self):
    c_x = self.origin[0]
    c_y = self.origin[1]
    c_z = self.origin[2]


