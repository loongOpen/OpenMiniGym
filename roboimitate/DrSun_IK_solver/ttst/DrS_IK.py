import time
import mujoco
import mujoco.viewer
import numpy as np
from ctypes import *


def DrS_IK_solver(real_arrayleft,real_arrayright):
    lib_l = CDLL('/home/tinymal/下载/imitate_robohome/software/DrSun_IK_solver/ttst/ik_7dof_fourier/test_l.so')
    lib_l.ik_left.restype = POINTER(c_double)
    
    left_ptr = lib_l.ik_left(real_arrayleft)
    right_ptr = lib_l.ik_left(real_arrayright)
    theta_left = [left_ptr[i] for i in range(7)]
    theta_right = [right_ptr[i] for i in range(7)]
    theta_left[1] = theta_left[1] + 2.007
    theta_left[2] = theta_left[2] + 1.5708
    theta_right[1] = -theta_right[1] - 2.007
    theta_right[2] = theta_right[2] + 1.5708
    theta_right[3] = -theta_right[3]
    theta_right[5] = -theta_right[5]
    theta_right[6] = -theta_right[6]
    lib_l.free_memory(left_ptr)
    lib_l.free_memory(right_ptr)
    return np.concatenate([theta_left,theta_right])
    
    
real_arrayleft = (c_double * 7)(95.4/180*3.1415, -17.4/180*3.1415, -0.6/180*3.1415, -231.7, 307.8, 158.1, 0.52333)
real_arrayright = (c_double * 7)(100.2/180*3.1415, -12.3/180*3.1415, 1.5/180*3.1415, -243.1,285.3,148.8 ,0.52333)
q = DrS_IK_solver(real_arrayleft,real_arrayright)
a=1# a=1