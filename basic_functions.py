from shapely.geometry import Polygon, Point, LineString
import math
import numpy as np
import pyproj
from shapely.ops import transform
from shapely import affinity
import matplotlib
import matplotlib.pyplot as plt

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
        #print(box)
        return box

def abs_to_rel(robot, obj_pos, l, w):
        rel_pos = obj_pos - robot.pos
        yaw = -0.5*math.pi
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                        [-math.sin(yaw), math.cos(yaw)]])

        rel_pos_rot = rel_pos.dot(Rot1)
        rel_yaw = 0.5*math.pi-robot.yaw
        
        rel_box = trans(np.array([0,0]), 0.5*math.pi+rel_yaw, l, w, rel_pos_rot)

        rel_object = {'polygon': rel_box, 'abs_pos': obj_pos}
        return rel_object

def coordinate_transform(robot, abs_pos):
        rel_pos = abs_pos - robot.pos
        yaw = 0.5*math.pi-robot.yaw
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                        [-math.sin(yaw), math.cos(yaw)]])
        rel_pos_rot = rel_pos.dot(Rot1)
        return rel_pos_rot

def coordinate_transform_abs_to_rel(robot, geom):
        yaw = robot.yaw-0.5*math.pi
        pos = robot.pos
        rot = affinity.rotate(geom, -yaw, (pos[0], pos[1]), use_radians=True)
        trans = affinity.translate(rot, -pos[0], -pos[1])
        return trans

def coordinate_transform_rel_to_abs(robot, geom):
        yaw = robot.yaw-0.5*math.pi
        pos = robot.pos
        trans = affinity.translate(geom, pos[0], pos[1])
        rot = affinity.rotate(trans, yaw, (pos[0], pos[1]), use_radians=True)
        return trans

def move_figure(f, x, y):
    """Move figure's upper left corner to pixel (x, y)"""
    backend = matplotlib.get_backend()
    if backend == 'TkAgg':
        f.canvas.manager.window.wm_geometry("+%d+%d" % (x, y))
    elif backend == 'WXAgg':
        f.canvas.manager.window.SetPosition((x, y))
    else:
        # This works for QT and GTK
        # You can also use window.setGeometry
        f.canvas.manager.window.move(x, y)

def shift_line(line, d):
        coords = line.coords[:]
        x1 = coords[0][0]
        x2 = coords[1][0]
        y1 = coords[0][1]
        y2 = coords[1][1]
        r = math.hypot(x2-x1, y2-y1)
        dx = d/r*(y1-y2)
        dy = d/r*(x2-x1)
        x3 = x1 + dx
        x4 = x2 + dx
        y3 = y1 + dy
        y4 = y2 + dy
        return LineString([(x3, y3),(x4, y4)])

def extend_line(line, phi, d):
        coords = line.coords[:]
        x1 = coords[0][0]
        x2 = coords[1][0]
        y1 = coords[0][1]
        y2 = coords[1][1]
        x3 = x2 - math.cos(phi)*d
        y3 = y2 - math.sin(phi)*d
        return LineString([(x1, y1),(x3, y3)])