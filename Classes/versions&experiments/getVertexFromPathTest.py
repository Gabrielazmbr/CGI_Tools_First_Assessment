import maya.cmds as cmds

"""
    For this test I was trying to get just the vertex involved in the path of my ball.
    Although this approach later changed, I could also have used vertex instead of
    faces centers.
"""


class Mobius_stair:
    def __init__(self, depth=0.5, width=5, height=100, s_depth=1, s_width=2, s_height=40):
        self.depth = depth
        self.width = width
        self.height = height
        self.s_depth = s_depth
        self.s_width = s_width
        self.s_height = s_height

    def make_strip(self, name="Stairs"):
        self.stairs = cmds.polyCube(
            n=name,
            w=self.width,
            h=self.height,
            d=self.depth,
            sw=self.s_width,
            sh=self.s_height,
            sd=self.s_depth,
        )[0]

        all_faces = cmds.polyEvaluate(self.stairs, face=True)
        top_faces = []
        bottom_faces = []

        for f in range(all_faces):
            if f < self.s_height * 2:
                top_faces.append(f)
            elif self.s_height * 2 + 1 < f < self.s_height * 4 + 2:
                bottom_faces.append(f)

        face_group = 4
        top_face_groups = [top_faces[i : i + face_group] for i in range(0, len(top_faces), face_group)]
        bottom_face_groups = [bottom_faces[i : i + face_group] for i in range(0, len(bottom_faces), face_group)]

        l_scale_z = 0
        step = 0.8
        mid_stairs = len(top_face_groups) / 2
        for i, face_range in enumerate(top_face_groups):
            start = face_range[0]
            end = face_range[-1]
            cmds.polyExtrudeFacet(f"{self.stairs}.f[{start}:{end}]", kft=True, ltz=l_scale_z)

            print(i, l_scale_z)

            if i < mid_stairs:
                l_scale_z = round(l_scale_z + step, 2)
            else:
                l_scale_z = round(l_scale_z - step, 2)
                if i == mid_stairs * 2:
                    break

        l_scale_z += 0.8
        for i, face_range in enumerate(bottom_face_groups):
            start = face_range[0]
            end = face_range[-1]
            cmds.polyExtrudeFacet(f"{self.stairs}.f[{start}:{end}]", kft=True, ltz=l_scale_z)
            print(i, l_scale_z)

            if i < mid_stairs:
                l_scale_z = round(l_scale_z + step, 2)
            else:
                l_scale_z = round(l_scale_z - step, 2)
                if i == mid_stairs * 2:
                    break

        cmds.nonLinear(self.stairs, type="twist", startAngle=180)
        cmds.nonLinear(self.stairs, type="bend", curvature=180)
        cmds.select(self.stairs)
        cmds.setAttr(f"{self.stairs}.translate", 0, 0, -0.4)
        cmds.select(self.stairs)
        cmds.CenterPivot(self.stairs)
        cmds.DeleteHistory(self.stairs)
        cmds.makeIdentity(self.stairs, apply=True, translate=True, rotate=True, scale=True)
        cmds.setAttr(f"{self.stairs}.rotate", 90, 0, 0)
        cmds.makeIdentity(self.stairs, apply=True, translate=True, rotate=True, scale=True)


def get_vertex(mesh, start_edge):
    # this works
    edge_loop = cmds.polySelect("Stairs", el=371)
    edges = [f"Stairs.e[{i}]" for i in edge_loop]
    vertices = []
    vertex = cmds.polyListComponentConversion(edges, fe=True, tv=True)
    cmds.select(vertex, r=True)
    vertices.append(vertex)
    print(vertices)


my_stair = Mobius_stair()
my_stair.make_strip()
mesh = "Stairs"
start_edge = 331
get_vertex(mesh, start_edge)
