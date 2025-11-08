import maya.cmds as cmds
import maya.api.OpenMaya as om


class Mobius_stair:
    def __init__(
        self, depth=0.5, width=5, height=100, s_depth=1, s_width=1, s_height=20, name = "Stairs"
    ):
        self.depth = depth
        self.width = width
        self.height = height
        self.s_depth = s_depth
        self.s_width = s_width
        self.s_height = s_height
        self.name = name

    def make_strip(self):
        self.stairs = cmds.polyCube(
            n=self.name,
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
            if f < self.s_height :
                top_faces.append(f)
            elif self.s_height < f < self.s_height * 2 + 1:
                bottom_faces.append(f)

        top_face_groups = [
            top_faces[i : i + 1] for i in range(0, len(top_faces), 1)
        ]
        bottom_face_groups = [
            bottom_faces[i : i + 1]
            for i in range(0, len(bottom_faces), 1)
        ]

        l_scale_z = 0
        step = 0.8
        mid_stairs = len(top_face_groups) / 2
        for i, face_range in enumerate(top_face_groups):
            start = face_range[0]
            end = face_range[-1]
            cmds.polyExtrudeFacet(
                f"{self.stairs}.f[{start}:{end}]", kft=True, ltz=l_scale_z
            )

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
            cmds.polyExtrudeFacet(
                f"{self.stairs}.f[{start}:{end}]", kft=True, ltz=l_scale_z
            )

            if i < mid_stairs:
                l_scale_z = round(l_scale_z + step, 2)
            else:
                l_scale_z = round(l_scale_z - step, 2)
                if i == mid_stairs * 2:
                    break

        cmds.nonLinear(self.stairs, type="twist", startAngle=180)
        cmds.nonLinear(self.stairs, type="bend", curvature=180)
        cmds.select(self.stairs)
        cmds.setAttr(f"{self.stairs}.translate",0,0,-0.4)
        cmds.select(self.stairs)
        cmds.CenterPivot(self.stairs)
        cmds.DeleteHistory(self.stairs)
        cmds.makeIdentity(
            self.stairs, apply=True, translate=True, rotate=True, scale=True
        )
        cmds.setAttr(f"{self.stairs}.rotate", 90, 0, 0)
        cmds.makeIdentity(
            self.stairs, apply=True, translate=True, rotate=True, scale=True
        )

    def get_face_center(self, face_index):
        # Get the DAG path for the mesh shape
        sel = om.MSelectionList()
        sel.add(self.name)
        dag = sel.getDagPath(0)

        # Make sure it's a mesh
        mesh_fn = om.MFnMesh(dag)

        # Get vertex indices for the face
        vert_ids = mesh_fn.getPolygonVertices(face_index)

        # Get world positions of the vertices
        points = [mesh_fn.getPoint(v, om.MSpace.kWorld) for v in vert_ids]

        # Compute average
        x = sum(p.x for p in points) / len(points)
        y = sum(p.y for p in points) / len(points)
        z = sum(p.z for p in points) / len(points)

        return (x, y, z)





my_stair = Mobius_stair()
my_stair.make_strip()
