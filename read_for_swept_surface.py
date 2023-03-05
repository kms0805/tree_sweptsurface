import numpy as np
import spline

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
       return v
    return v / norm

class Section:
    def __init__(self, control_point = [], scale = 0, rotation = [0,0,0,0], position = [0,0,0]):
        self.control_points = control_point
        self.inter_points = []
        self.scale = scale
        self.rotation = rotation
        self.position = position

class SurfaceData:
    def __init__(self, file, resolution1, resolution2):
        self.curve_type = []
        self.resolution1 = resolution1
        self.resolution2 = resolution2
        self.num_cross_sections = 0
        self.num_control_points =0
        self.sections = []
        self.inter_sections = []
        self.read(file)
        self.interpolate()
    def read(self,file):
        with open(file) as f:
            lines = f.readlines()
        index = 0
        while lines[index].startswith('#') or lines[index].startswith('\n'):
            index += 1
        line = lines[index].replace('#', ' ')
        words = line.split()
        self.curve_type = words[0]
        index += 1
        line = lines[index].replace('#', ' ')
        words = line.split()
        self.num_cross_sections = int(words[0])
        index += 1
        line = lines[index].replace('#', ' ')
        words = line.split()
        self.num_control_points = int(words[0])
        index += 1
        for i in range(self.num_cross_sections):
            while lines[index].startswith('#') or lines[index].startswith('\n'):
                index += 1
            section = Section()
            control_points = []
            for j in range(self.num_control_points):
                line = lines[index].replace('#', ' ')
                words = line.split()
                control_point = np.array(words[0:2], dtype=float)
                control_points.append(control_point)
                index += 1
            section.control_points = control_points
            line = lines[index].replace('#', ' ')
            words = line.split()
            section.scale = np.array(words[0], dtype=float)
            index += 1
            line = lines[index].replace('#', ' ')
            words = line.split()
            section.rotation = np.array(words[0:4], dtype=float)
            index += 1
            line = lines[index].replace('#', ' ')
            words = line.split()
            section.position = np.array(words[0:3], dtype=float)
            index += 1
            self.sections.append(section)
    def interpolate(self):
        self.catmull_rom_section(False)
        if self.curve_type == 'CATMULL_ROM':
            self.catmull_rom_face()
        elif self.curve_type == 'BSPLINE':
            self.bspline_face()
    def catmull_rom_face(self):
        for section in self.inter_sections:
            spline.CatmullRom_curve(section.control_points, section.inter_points, self.resolution1, True)
    def bspline_face(self):
        for section in self.inter_sections:
            spline.cubicBspline_curve(section.control_points, section.inter_points, self.resolution1, True)
    def catmull_rom_section(self, closed):
        control_points_list = []
        inter_c = []
        for i in range(self.num_control_points):
            control_points_list.append([])
            inter_c.append([])
        position_list = []
        inter_p = []
        rotation_list = []
        inter_r = []
        scale_list = []
        inter_s = []
        for section in self.sections:
            for i in range(self.num_control_points):
                control_points_list[i].append(section.control_points[i])
            position_list.append(section.position)
            rotation_list.append(section.rotation)
            scale_list.append(section.scale)
        #interpolation
        for i in range(self.num_control_points):
            spline.CatmullRom_curve(control_points_list[i], inter_c[i], self.resolution2, closed)
        spline.CatmullRom_curve(position_list,inter_p,self.resolution2,closed)
        spline.CatmullRom_curve(scale_list, inter_s, self.resolution2, closed)
        #orientation interpolation
        quat_list = spline.convert_rotations_to_quats(rotation_list)
        inter_q = []
        spline.CatmullRom_curve_quat(quat_list,inter_q, self.resolution2, closed)
        inter_r = spline.convert_quats_to_matrices(inter_q)
        #new section
        for j in range(len(inter_p)):
            control_point = []
            for i in range(self.num_control_points):
                control_point.append(inter_c[i][j])
            self.inter_sections.append(Section(control_point, inter_s[j], inter_r[j], inter_p[j]))
    def make_vertex_set_to_draw(self):
        vertex_set = []
        normal_list = []
        vertex_list = []
        for section in self.inter_sections:
            rotation_M = section.rotation.T
            translation_M = np.array([[1, 0, 0, section.position[0]],
                                      [0, 1, 0, section.position[1]],
                                      [0, 0, 1, section.position[2]],
                                      [0, 0, 0, 1]])
            scale_M = np.array([[section.scale, 0, 0, 0],
                                [0, section.scale, 0, 0],
                                [0, 0, section.scale, 0],
                                [0, 0, 0, 1]])

            points = np.ones((4,len(section.inter_points)))
            for i,p  in enumerate(section.inter_points):
                points[0:3,i] = np.array([p[0], 0, p[1]])
            points = translation_M @ rotation_M @ scale_M @ points
            vertex_set.append(points[0:3])
        (r, c) = vertex_set[0].shape
        for i in range(len(vertex_set) - 1):
            v1 = vertex_set[i]
            v2 = vertex_set[i + 1]
            for j in range(c - 1):
                normal = -np.cross(v2[:, j] - v1[:, j], v1[:, j + 1] - v1[:, j])
                if np.linalg.norm(normal) == 0:
                    normal = np.cross(v2[:, j+1] - v2[:, j], v2[:, j] - v1[:, j])
                normal = normalize(normal)
                normal_list.append(normal)
                vertex_list.append(v1[:, j])
                vertex_list.append(v1[:, j + 1])
                vertex_list.append(v2[:, j + 1])
                vertex_list.append(v2[:, j])
            normal = -np.cross(v2[:, -1] - v1[:, -1], v1[:, 0] - v1[:, -1])
            if np.linalg.norm(normal) == 0:
                normal = np.cross(v2[:, 0] - v2[:, -1], v2[:, -1] - v1[:, -1])
            normal = normalize(normal)
            normal_list.append(normal)
            vertex_list.append(v1[:, -1])
            vertex_list.append(v1[:, 0])
            vertex_list.append(v2[:, 0])
            vertex_list.append(v2[:, -1])
        return vertex_list, normal_list

if __name__ == '__main__':
    a = SurfaceData('trombone.txt')
    b1 = a.sections[0]
    b2 = a.sections[1]

    print(a.num_cross_sections)
    print(a.num_control_points)
    print(a.curve_type)
    print(b1.control_points)
    print(b1.rotation)
    print(b1.scale)