from shapely.geometry import Polygon
import math
import numpy as np

def convert_nparray_to_polygon(poly_ndarray):
        array_tmp = poly_ndarray.squeeze()
        x = array_tmp[0:4]
        y = array_tmp[4:8]
        polygon = Polygon([(x[i], y[i]) for i in range(0, 4)])
        return polygon


def trans(pos, yaw, l, w, rel_pos=np.array([0,0])):        
        outline = np.array([[-l/2, l/2, l/2, -l/2, -l/2],
                        [-w/2, -w/2, w/2, w/2, -w/2]])
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                        [-math.sin(yaw), math.cos(yaw)]])
        outline = (outline.T.dot(Rot1)).T
        rel_pos_rot = rel_pos.dot(Rot1)
        #print(rel_pos)
        #print(rel_pos_rot)
        outline[0, :] += pos[0] + rel_pos_rot[0]
        outline[1, :] += pos[1] + rel_pos_rot[1]
        box = convert_nparray_to_polygon(np.hstack((outline[0, :][:-1], outline[1, :][:-1])))  
        return box
