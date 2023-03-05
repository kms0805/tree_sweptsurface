import numpy as np
from pyquaternion import Quaternion
def discretize_bezier( p0, p1, p2, p3, curve, resolution ):
    for j in range(resolution):
        t = 1.0/(resolution-1)*j
        b0 = (1.0-t)**3.0
        b1 = 3.0*t*(1.0-t)**2.0
        b2 = 3.0*t*t*(1.0-t)
        b3 = t*t*t
        b = np.array([b0, b1, b2, b3])
        p = np.array([p0, p1, p2, p3])
        point = b@p
        curve.append(point)
def CatmullRom(control_points, Bpoints, closed):
    num = len(control_points)
    if num<3:
        return
    # set Bpoints size
    if not closed:
        while len(Bpoints)<3*num-2:
            Bpoints.append([])
    else:
        while len(Bpoints)<3*num+1:
            Bpoints.append([])
    # set control points at keypoints
    for i in range(num):
        Bpoints[3*i] = control_points[i]
    if closed:
        Bpoints[3*num] = control_points[0]
    for i in range(num-1):
        if i>0:
            Bpoints[3*i+1] = control_points[i] + (control_points[i+1]-control_points[i-1])/6.0
        else:
            if closed:
                Bpoints[1] = control_points[0] + (control_points[1]-control_points[num-1])/6.0
            else:
                Bpoints[1] = control_points[0]
        if i<num-2:
            Bpoints[3*i+2] = control_points[i+1] - (control_points[i+2]-control_points[i])/6.0
        else:
            if closed:
                Bpoints[3*i+2] = control_points[num-1] + (control_points[num-2]-control_points[0])/6.0

                Bpoints[3*i+4] = control_points[num-1] - (control_points[num-2]-control_points[0])/6.0

                Bpoints[3*i+5] = control_points[0] - (control_points[1]-control_points[num-1])/6.0
            else:
                Bpoints[3*num-4] = control_points[num-1]
def CatmullRom_curve(control_points, curve, resolution,  closed):
    Bpoints= []
    CatmullRom(control_points, Bpoints, closed)
    num = len(Bpoints)
    if num<4:
        return
    for i in range(0,num-3,3):
        discretize_bezier(Bpoints[i], Bpoints[i+1], Bpoints[i+2], Bpoints[i+3], curve, resolution)

def discretize_bspline( p0, p1, p2, p3, curve, resolution):
    for j in range(resolution):
        t = 1.0/(resolution-1)*j
        b0 = (1.0-t)**3.0/6.0
        b1 = (3.0*t*t*t - 6.0*t*t + 4.0)/6.0
        b2 = (-3.0*t*t*t + 3*t*t + 3*t + 1)/6.0
        b3 = t**3/6.0
        b = np.array([b0, b1, b2, b3])
        p = np.array([p0, p1, p2, p3])
        point = b @ p
        curve.append(point)

def cubicBspline_curve(points, curve, resolution, closed ):
    num = len(points)
    if ( num<4 ):
        return
    if closed:
        for i in range(num-3):
            discretize_bspline( points[i], points[i+1], points[i+2], points[i+3], curve, resolution )
        discretize_bspline( points[num-3], points[num-2], points[num-1], points[0], curve, resolution)
        discretize_bspline( points[num-2], points[num-1], points[0], points[1], curve, resolution)
        discretize_bspline( points[num-1], points[0], points[1], points[2], curve, resolution)

    else:
        for i in range(num-3):
            discretize_bspline( points[i], points[i+1], points[i+2], points[i+3], curve, resolution )

def convert_rotations_to_quats(rotations):
    quats = []
    for rotation in rotations:
        try:
            quat = Quaternion(axis = rotation[1:4], angle = rotation[0])
        except(ZeroDivisionError):
            quat = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        quats.append(quat)
    return quats
def convert_quats_to_matrices(quats):
    matrices = []
    for quat in quats:
        matrix = quat.transformation_matrix
        matrices.append(matrix)
    return matrices
def CatmullRom_quat(control_quats, Bquats, closed):
    num = len(control_quats)
    if num < 3:
        return
    # set Bpoints size
    if not closed:
        while len(Bquats) < 3 * num - 2:
            Bquats.append([])
    else:
        while len(Bquats) < 3 * num + 1:
            Bquats.append([])
    # set control points at keypoints
    for i in range(num):
        Bquats[3 * i] = control_quats[i]
    if closed:
        Bquats[3 * num] = control_quats[0]
    for i in range(num - 1):
        if i > 0:
            Bquats[3 * i + 1] = control_quats[i] * \
                                Quaternion.exp(1/6 * Quaternion.log(control_quats[i - 1].inverse * control_quats[i + 1]))
        else:
            if closed:
                Bquats[1] = control_quats[0] * \
                                Quaternion.exp(1/6 * Quaternion.log(control_quats[num - 1].inverse * control_quats[1]))
            else:
                Bquats[1] = control_quats[0]
        if i < num - 2:
            Bquats[3 * i + 2] = control_quats[i + 1] * \
                                Quaternion.exp(1/6 * Quaternion.log(control_quats[i].inverse * control_quats[i + 2]))
        else:
            if closed:
                Bquats[3 * i + 2] = control_quats[num - 1] * \
                                Quaternion.exp(1/6 * Quaternion.log(control_quats[0].inverse * control_quats[num - 2]))
                Bquats[3 * i + 4] = control_quats[num - 1] * \
                                Quaternion.exp(1/6 * Quaternion.log(control_quats[0].inverse * control_quats[num - 2])).inverse
                Bquats[3 * i + 5] = control_quats[0] * \
                                Quaternion.exp(1/6 * Quaternion.log(control_quats[num - 1].inverse * control_quats[1])).inverse
            else:
                Bquats[3 * num - 4] = control_quats[num - 1]
def dicretize_quat(q0, q1, q2, q3, curve, resolution):
    for j in range(resolution):
        t = 1.0 / (resolution - 1) * j
        q01 = Quaternion.slerp(q0, q1, amount=t)
        q12 = Quaternion.slerp(q1, q2, amount=t)
        q23 = Quaternion.slerp(q2, q3, amount=t)
        q0112 = Quaternion.slerp(q01, q12, amount=t)
        q1223 = Quaternion.slerp(q12, q23, amount=t)
        q = Quaternion.slerp(q0112, q1223, amount=t)
        curve.append(q)
def CatmullRom_curve_quat(control_quats, curve, resolution,  closed):
    Bquats = []
    CatmullRom_quat(control_quats, Bquats, closed)
    num = len(Bquats)
    if num<4:
        return
    for i in range(0,num-3,3):
        dicretize_quat(Bquats[i], Bquats[i+1], Bquats[i+2], Bquats[i+3], curve, resolution)
