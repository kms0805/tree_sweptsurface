from OpenGL.GLUT import *
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
import os
from read_for_swept_surface import *

width = 500
height = 500
mouseRotatePressed = False
mouseMovePressed   = False
mouseZoomPressed   = False
lastX = 0
lastY = 0
zoom = 45
lastZoom = 0
timeStep = 30
eye = np.array([ 0, 0, 200, 1])
ori = np.array([ 0, 0, 0, 1])
rot = np.array([ 0.0, 1.0, 0.0, 1])
rot_M = np.identity(4)
lastrot_m = np.identity(4)
M = np.identity(4)
lastM = np.identity(4)
ESCAPE = b'\x1b'
show_control_points = False
show_axis = False
show_model = True
def print_help():
    print("############# 3D Viewer ###############\n",
          "[Rotate   ]: left mouse motion\n",
          "[Translate]: 'shift' & left mouse motion\n",
          "[Zoom i/o ]: 'ctrl' & left mouse motion(only X direction)\n",
          "[Dolly i/o]: keyboard 'd' -> dolly in, keyboard 'f'- > dolly out\n",
          "[Show all ]: keyboard 'a'\n",
          "[Seek     ]: right mouse click on point\n"
          )
def trackballProject(x,y):
    r = 1
    x = (2*x - width) / width
    y = (height- 2*y) / height
    if x * x + y * y <= r * r / 2:
        z = np.sqrt(r * r - x * x - y * y)
    else:
        z = r * r / 2 / np.sqrt(x * x + y * y)
    ret = np.array([x, y, z])
    ret = ret/np.linalg.norm(ret)
    return ret
def rotation_to_quat(axis,theta):
    return np.concatenate(([np.cos(theta/2)],np.sin(theta/2) * axis))
def quaternion_rotation_matrix(Q):
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
    rot_matrix = np.array([[r00, r01, r02, 0],
                           [r10, r11, r12, 0],
                           [r20, r21, r22, 0],
                           [0, 0, 0, 1]])
    return rot_matrix
def glutMotion(x,y):
    global lastX,lastY, zoom, rot_M, M, lastZoom, lastrot_m, lastM

    if mouseRotatePressed:
        v1 = trackballProject(lastX, lastY)
        v2 = trackballProject(x, y)
        axis = np.cross(v1, v2)
        if np.linalg.norm(axis) > 0:
            axis = axis / np.linalg.norm(axis)
            cos_theta = np.dot(v1, v2)
            sin_theta = np.linalg.norm(np.cross(v1, v2))
            theta = -np.arctan2(sin_theta, cos_theta)
            q = rotation_to_quat(axis, theta)
            Rot_M = quaternion_rotation_matrix(q)
            rot_M = lastrot_M @ Rot_M
            M = lastM @ Rot_M
            reshape(width,height)
    elif mouseMovePressed:
        Trans_M = np.array([[1, 0, 0, -(x - lastX)*0.1],
                          [0, 1, 0, -(lastY - y)*0.1],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])
        M = lastM @ Trans_M
        reshape(width, height)
    elif mouseZoomPressed:
        zoom = lastZoom + (x- lastX)*0.1
        if zoom > 80:
            zoom = 80
        elif zoom < 10:
            zoom = 10
        reshape(width, height)
    glutPostRedisplay()
def glutMouse(button, state, x, y):
    global mouseMovePressed,mouseRotatePressed,mouseZoomPressed,\
        lastX, lastY, lastM, lastZoom, lastM, lastrot_M, M, rot_M
    if state == GLUT_UP:
        mouseMovePressed = False
        mouseRotatePressed = False
        mouseZoomPressed = False
    else:
        if button == GLUT_LEFT_BUTTON:
            if GLUT_ACTIVE_SHIFT == glutGetModifiers():
                lastX = x
                lastY = y
                lastM = M
                mouseMovePressed = True
                mouseRotatePressed = False
                mouseZoomPressed = False
            elif GLUT_ACTIVE_CTRL == glutGetModifiers():
                lastX = x
                lastZoom = zoom
                mouseMovePressed = False
                mouseRotatePressed = False
                mouseZoomPressed = True
            else:
                lastX = x
                lastY = y
                lastM = M
                lastrot_M = rot_M
                mouseMovePressed = False
                mouseRotatePressed = True
                mouseZoomPressed = False
        if button == GLUT_RIGHT_BUTTON:
            winx = x
            winy =  height - y
            z = glReadPixels(winx,winy,1,1,GL_DEPTH_COMPONENT,GL_FLOAT)
            if z[0] < 1:
                new_point = gluUnProject(winx,winy,z[0])
                how_translate = M @ np.array([0, 0, 0, 1])
                M = np.array([[1, 0, 0, -how_translate[0] + new_point[0]],
                              [0, 1, 0, -how_translate[1] + new_point[1]],
                              [0, 0, 1, -how_translate[2] + new_point[2]],
                              [0, 0, 0, 1]]) @ M
                reshape(width,height)
            else:
                print('wrong point!')
    glutPostRedisplay()
def reshape(w, h):
    global width
    global height
    global zoom
    global eye, ori, rot
    width = glutGet(GLUT_WINDOW_WIDTH)
    height = glutGet(GLUT_WINDOW_HEIGHT)
    glViewport(0, 0, w, h)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    aspectRatio = w / h
    new_eye = M @ eye
    new_rot = rot_M @ rot
    new_ori = M @ ori
    gluPerspective(zoom,aspectRatio, 0.1, 1000.0)
    gluLookAt(new_eye[0], new_eye[1], new_eye[2],
              new_ori[0], new_ori[1], new_ori[2],
              new_rot[0], new_rot[1], new_rot[2])
    glMatrixMode(GL_MODELVIEW)
    glutPostRedisplay()
def glutKeyboard(key, x, y):
    global eye, zoom, M, show_control_points, show_axis, show_model

    if key == b'd' and eye[2]>30:
        eye[2] -= 5
        reshape(width, height)
    if key == b'f':
        eye[2] += 5
        reshape(width, height)
    if key == b'a':
        how_translate = M @ np.array([0, 0, 0, 1])
        M =	np.array([[1, 0, 0, -how_translate[0]],
                          [0, 1, 0, -how_translate[1]],
                          [0, 0, 1, -how_translate[2]],
                          [0, 0, 0, 1]]) @ M
        if eye[2] < 90:
            eye[2] = 200
        if zoom < 45:
            zoom = 45
        reshape(width,height)

    if key == b'c':
        show_control_points = not show_control_points
    if key == b'x':
        show_axis = not show_axis
    if key == b'm':
        show_model = not show_model
    if key == ESCAPE:
        os._exit(0)
    glutPostRedisplay()

def print_help_show():
    print("############# SWEPT SURFACE ###############\n",
          "show Axis           : keyboard 'x' toggle\n",
          "show Control Points : keyboard 'c' toggle\n",
          "show Model          : keyboard 'm' toggle\n",
          )
def draw_model(vertex_list, normal_list):
    glEnable(GL_COLOR_MATERIAL)
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE)
    for i in range(len(normal_list)):
        glColor3f(1, 1, 1)
        glBegin(GL_QUADS)
        glNormal3f(*normal_list[i])
        glVertex3f(*vertex_list[4 * i])
        glVertex3f(*vertex_list[4 * i + 1])
        glVertex3f(*vertex_list[4 * i + 2])
        glVertex3f(*vertex_list[4 * i + 3])
        glEnd()
def draw_raw():
    glEnable(GL_COLOR_MATERIAL)
    glLineWidth(2.5)
    for s in data.sections:
        glColor3f(1, 1, 1)
        glPushMatrix()
        glTranslatef(*s.position)
        glRotatef(-1 * s.rotation[0] * 180 /np.pi, *s.rotation[1:])
        glScalef(s.scale,s.scale,s.scale)
        glPointSize(5)
        glBegin(GL_LINE_LOOP)
        for c in s.control_points:
            glVertex3f(c[0], 0, c[1])
        glEnd()
        glPopMatrix()
    glDisable(GL_COLOR_MATERIAL)
def display():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glEnable(GL_DEPTH_TEST)
    glClearColor(0.1, 0.1, 0.1, 1.0)
    glLoadIdentity()
    if show_model:
        draw_model(vertex_list, normal_list)
    if show_control_points:
        draw_raw()
    if show_axis:
        displayAxis()
    glFlush()
    glutSwapBuffers()
def Timer(unused):
	glutPostRedisplay()
	glutTimerFunc(30, Timer, 1)
def displayAxis():
    glEnable(GL_COLOR_MATERIAL)
    glLineWidth(2.5)
    glColor3f(1.0, 0.0, 0.0)
    glBegin(GL_LINES)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(100, 0, 0)
    glEnd()
    # // x
    glColor3f(0.0, 1.0, 0.0)
    glBegin(GL_LINES)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0, 100, 0)
    glEnd()
    # // y
    glColor3f(0.0, 0.0, 1.0)
    glBegin(GL_LINES)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0, 0, 100)
    glEnd()
    # // z
    glDisable(GL_COLOR_MATERIAL)

if __name__ == "__main__":
    print_help()
    print_help_show()
    data = SurfaceData('christmas_tree.txt', 10, 10)
    vertex_list, normal_list = data.make_vertex_set_to_draw()
    glutInit()
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
    glutInitWindowSize(width, height)
    glutInitWindowPosition( 100, 0 )
    glutCreateWindow("swept_surface")
    glShadeModel(GL_SMOOTH)
    glEnable(GL_LIGHTING)

    glEnable(GL_LIGHT0)
    ambientLight = [0.05, 0.0, 0.0, 1.0]
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight)
    diffuseLight = [0, 0.5, 0, 1.0]
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight)
    position = [100, -50, 100.0, 0.0]
    glLightfv(GL_LIGHT0, GL_POSITION, position)

    glEnable(GL_LIGHT1)
    ambientLight = [0, 0.05, 0.0, 1]
    glLightfv(GL_LIGHT1, GL_AMBIENT, ambientLight)
    diffuseLight = [0, 0.3, 0.2, 1]
    glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuseLight)
    position = [-100, -50, -100.0, 0.0]
    glLightfv(GL_LIGHT1, GL_POSITION, position)

    glEnable(GL_LIGHT2)
    ambientLight = [0.01, 0, 0.0, 1]
    glLightfv(GL_LIGHT2, GL_AMBIENT, ambientLight)
    diffuseLight = [0.3, 0.3, 0, 0.1]
    glLightfv(GL_LIGHT2, GL_DIFFUSE, diffuseLight)
    position = [0, 100, 100, 0.0]
    glLightfv(GL_LIGHT2, GL_POSITION, position)

    glutReshapeFunc(reshape)
    glutDisplayFunc(display)
    glutKeyboardFunc(glutKeyboard)
    glutMouseFunc(glutMouse)
    glutMotionFunc(glutMotion)
    glutMainLoop()

