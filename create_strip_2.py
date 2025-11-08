import maya.api.OpenMaya as om
import maya.cmds as cmds


class Mobius_stair:
    def __init__(
        self,
        depth=0.5,
        width=5,
        height=100,
        s_depth=1,
        s_width=1,
        s_height=20,
        name="Stairs",
    ):
        self.depth = depth
        self.width = width
        self.height = height
        self.s_depth = s_depth
        self.s_width = s_width
        self.s_height = s_height
        self.name = name

    def select_stairs_mesh(self):
        # Get the DAG path for the mesh shape
        sel = om.MSelectionList()
        sel.add(self.name)
        dag = sel.getDagPath(0)

        # Make sure it's a mesh
        mesh_fn = om.MFnMesh(dag)
        return mesh_fn

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
            if f < self.s_height:
                top_faces.append(f)
            elif self.s_height < f < self.s_height * 2 + 1:
                bottom_faces.append(f)

        top_face_groups = [top_faces[i : i + 1] for i in range(0, len(top_faces), 1)]
        bottom_face_groups = [
            bottom_faces[i : i + 1] for i in range(0, len(bottom_faces), 1)
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

        stairs_obj = self.select_stairs_mesh()
        num_faces = stairs_obj.numPolygons
        normal_direction_pos = (0, 0, 1)
        normal_direction_neg = (0, 0, -1)
        dir_vector_pos = om.MVector(*normal_direction_pos).normalize()
        dir_vector_neg = om.MVector(*normal_direction_neg).normalize()
        threshold = 0.5
        bounce_faces = []

        for face in range(num_faces):
            normal = stairs_obj.getPolygonNormal(face, om.MSpace.kWorld).normalize()
            dot_pos = normal * dir_vector_pos
            dot_neg = normal * dir_vector_neg
            if dot_pos >= threshold:
                bounce_faces.append(face)
            elif dot_neg >= threshold:
                bounce_faces.append(face)

        return bounce_faces

        cmds.nonLinear(self.stairs, type="twist", startAngle=180)
        cmds.nonLinear(self.stairs, type="bend", curvature=180)
        cmds.select(self.stairs)
        cmds.setAttr(f"{self.stairs}.translate", 0, 0, -0.4)
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
        stairs_obj = self.select_stairs_mesh()

        # Get vertex indices for the face
        vert_ids = stairs_obj.getPolygonVertices(face_index)

        # Get world positions of the vertices
        points = [stairs_obj.getPoint(v, om.MSpace.kWorld) for v in vert_ids]

        # Compute average
        x = sum(p.x for p in points) / len(points)
        y = sum(p.y for p in points) / len(points)
        z = sum(p.z for p in points) / len(points)

        face_center = [x, y, z]

        return face_center

    def get_all_face_center(self):
        # Get the DAG path for the mesh shape
        sel = om.MSelectionList()
        sel.add(self.name)
        dag = sel.getDagPath(0)

        # Make sure it's a mesh
        mesh_fn = om.MFnMesh(dag)

        num_faces = mesh_fn.numPolygons

        face_certer_list = []
        for face_index in range(num_faces):
            self.get_face_center(face_index)


class Ball:
    def __init__(self, radius, name):
        self.radius = radius
        self.name = name
        pX = 0
        pY = 1
        pZ = 0

        self.ball = cmds.polySphere(name=name, radius=radius)[0]
        cmds.move(pX, pY, pZ, self.ball)
        cmds.CenterPivot(self.stairs)
        cmds.DeleteHistory(self.stairs)
        cmds.makeIdentity(
            self.stairs, apply=True, translate=True, rotate=True, scale=True
        )

    def ball_rig(self):
        # Create the rotate and scale control curve
        self.rotate_ctrl = cmds.circle(
            name="rotate_ctrl", normal=(0, 0.5, 0), radius=1.5
        )[0]
        self.scale_ctrl = cmds.circle(
            name="scale_ctrl", normal=(0, 0.5, 0), radius=0.5
        )[0]

        # Match position of control to the ball
        cmds.delete(cmds.pointConstraint(self.ball, self.rotate_ctrl))

        # Parent the geometry under the control
        cmds.parent(self.ball, self.rotate_ctrl)
        cmds.parent(self.rotate_ctrl, self.scale_ctrl)

        # Create a group
        self.ctrl_grp = cmds.group(self.scale_ctrl, name="CTRL_GRP")

        # Freeze transforms
        cmds.FreezeTransformations(self.ctrl_grp, 0)

        return self.ctrl_grp

    def bounce(self):
        start_frame = 1
        # initial_height = 12
        # bounces = 5
        # dec = 0.6
        # speed = 0.8

        frame = start_frame
        height = initial_height
        time_gap = 10
        length = 0
        length_interval = 4

        squash = 0.8
        stretch = 1.2
        h_aprox = 1

        cmd.setKeyframe(self.ctrl_grp, attribute="translateY", t=frame, v=height)
        cmds.keyTangent(self.ctrl_grp, e=True, weightedTangents=True)
        cmds.keyTangent(self.ctrl_grp, e=True, a=True, t=(frame,), outWeight=3)
        cmd.setKeyframe(self.ctrl_grp, attribute="translateX", t=frame, v=length)
        cmd.setKeyframe(self.scale_ctrl, attribute="scaleY", t=frame, v=1)
        cmd.setKeyframe(self.scale_ctrl, attribute="scaleX", t=frame, v=1)
        cmd.setKeyframe(self.scale_ctrl, attribute="scaleZ", t=frame, v=1)

        for _ in range(bounces):
            frame = frame + time_gap / 2 - 1
            cmds.setKeyframe(self.ctrl_grp, attribute="translateY", t=frame, v=h_aprox)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleY", t=frame, v=stretch)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleX", t=frame, v=squash)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleZ", t=frame, v=squash)

            frame = frame + 1
            length += length_interval
            cmd.setKeyframe(self.ctrl_grp, attribute="translateY", t=frame, v=0)
            cmd.setKeyframe(self.ctrl_grp, attribute="translateX", t=frame, v=length)
            cmd.setKeyframe(self.scale_ctrl, attribute="scaleY", t=frame, v=squash)
            cmd.setKeyframe(self.scale_ctrl, attribute="scaleX", t=frame, v=stretch)
            cmd.setKeyframe(self.scale_ctrl, attribute="scaleZ", t=frame, v=stretch)
            cmd.keyTangent(
                self.ctrl_grp,
                attribute="translateY",
                t=(frame,),
                itt="linear",
                ott="linear",
            )

            frame = frame + 1
            cmds.setKeyframe(self.ctrl_grp, attribute="translateY", t=frame, v=h_aprox)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleY", t=frame, v=stretch)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleX", t=frame, v=squash)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleZ", t=frame, v=squash)

            length += length_interval
            height *= dec
            frame = frame + time_gap / 2 - 1
            cmd.setKeyframe(self.ctrl_grp, attribute="translateY", t=frame, v=height)
            cmds.keyTangent(self.ctrl_grp, e=True, weightedTangents=True)
            cmds.keyTangent(self.ctrl_grp, e=True, a=True, t=(frame,), outWeight=3)
            cmd.setKeyframe(self.scale_ctrl, attribute="scaleY", t=frame, v=1)
            cmd.setKeyframe(self.scale_ctrl, attribute="scaleX", t=frame, v=1)
            cmd.setKeyframe(self.scale_ctrl, attribute="scaleZ", t=frame, v=1)

            time_gap *= speed
            h_aprox = h_aprox - 0.25

        cmd.setKeyframe(self.ctrl_grp, attribute="translateY", t=frame, v=0)
        cmd.setKeyframe(self.ctrl_grp, attribute="translateX", t=frame, v=length)

        rolling_gap = 2
        rolling_speed = 1.4
        length_interval = 2
        rolling = 5

        for i in range(rolling):
            frame = int(frame + rolling_gap / 2)
            length += length_interval
            cmd.setKeyframe(self.ctrl_grp, attribute="translateX", t=frame, v=length)
            rolling_gap *= rolling_speed


my_stair = Mobius_stair()
my_stair.make_strip()
