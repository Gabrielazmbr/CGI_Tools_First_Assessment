import maya.api.OpenMaya as om
import maya.cmds as cmds
import numpy as np


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
        self.bounce_faces = []

        for face in range(num_faces):
            normal = stairs_obj.getPolygonNormal(face, om.MSpace.kWorld).normalize()
            dot_pos = normal * dir_vector_pos
            dot_neg = normal * dir_vector_neg
            if dot_pos >= threshold:
                self.bounce_faces.append(face)
            elif dot_neg >= threshold:
                self.bounce_faces.append(face)

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
        return self.bounce_faces

    def get_face_center(self):
        stairs_obj = self.select_stairs_mesh()
        self.centers = []

        # Get vertex indices for the face
        for face in self.bounce_faces:
            vert_ids = stairs_obj.getPolygonVertices(face)

            # Get world positions of the vertices
            points = [stairs_obj.getPoint(v, om.MSpace.kWorld) for v in vert_ids]

            # Compute average
            x = sum(p.x for p in points) / len(points)
            y = sum(p.y for p in points) / len(points)
            z = sum(p.z for p in points) / len(points)

            face_center = [x, y, z]
            self.centers.append(face_center)
        # Find midpoint
        midpoint = len(self.centers) // 2
        # reordered list
        self.centers_ordered = (
            self.centers[:midpoint] + self.centers[: midpoint - 1 : -1]
        )
        print(self.centers)
        print(self.centers_ordered)

    def get_face_normal(self):
        stairs_obj = self.select_stairs_mesh()
        self.faces_normals = []

        for face in self.bounce_faces:
            normal = stairs_obj.getPolygonNormal(face, om.MSpace.kWorld)
            normal_tuple = (normal.x, normal.y, normal.z)
            self.faces_normals.append(normal_tuple)
        # Find midpoint
        midpoint = len(self.faces_normals) // 2
        # reordered list
        self.faces_normals_ordered = (
            self.faces_normals[:midpoint] + self.faces_normals[: midpoint - 1 : -1]
        )
        print(self.faces_normals)
        print(len(self.faces_normals))
        print(self.faces_normals_ordered)


class Ball:
    def __init__(self, radius, name):
        self.radius = radius
        self.name = name
        pX = 0
        pY = 1
        pZ = 0

        self.ball = cmds.polySphere(name=name, radius=radius)[0]
        cmds.move(pX, pY, pZ, self.ball)
        cmds.CenterPivot(self.ball)
        cmds.DeleteHistory(self.ball)
        cmds.makeIdentity(
            self.ball, apply=True, translate=True, rotate=True, scale=True
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

    def unit(self, v, eps=1e-12):
        # Returns a unit normalized vector
        v = np.asarray(v, dtype=float)
        n = np.linalg.norm(v)
        if n < eps:
            raise ValueError("Zero-length vector")
        return v / n

    def apex_from_avg_vertex_normals(
        self, A, B, nA, nB, h, fallback_up=np.array([0.0, 0.0, 1.0]), eps=1e-9
    ):
        # Vec points
        A = np.asarray(A, dtype=float)
        B = np.asarray(B, dtype=float)
        # Face normals
        nA = np.asarray(nA, dtype=float)
        nB = np.asarray(nB, dtype=float)
        # Midpint from vectors
        M = 0.5 * (A + B)
        # Base vector
        b = B - A
        b_len = np.linalg.norm(b)
        if b_len < eps:
            raise ValueError("Base length too small.")
        # Base vector direction
        b_hat = b / b_len
        # average normal vector
        navg = 0.5 * (nA + nB)
        # Projection of normal vector to the base
        proj = navg - np.dot(navg, b_hat) * b_hat
        if np.linalg.norm(proj) < eps:
            proj = fallback_up - np.dot(fallback_up, b_hat) * b_hat
        # Direction perpendicular to b
        n_dir = self.unit(proj)
        if np.dot(n_dir, navg) < 0:
            n_dir = -n_dir

        apex = M + h * n_dir
        return apex.tolist()

    def bounce_path(self, mobius_stair):
        self.triangles = []
        verts = mobius_stair.centers_ordered
        normals = mobius_stair.faces_normals_ordered
        height = 10
        for cycle in range(2):
            for i in range(len(verts) - 1):
                A = verts[i]
                B = verts[i + 1]
                nA = normals[i]
                nB = normals[i + 1]
                apex = self.apex_from_avg_vertex_normals(A, B, nA, nB, height)
                self.triangles.append((A, B, apex))

        for i, tri in enumerate(self.triangles):
            print(f"Triangle {i}:")
            print("  A =", tri[0])
            print("  B =", tri[1])
            print("  apex =", tri[2])
            
    def align_pivot_to_face_normal(self, normal, up_axis="y"):

        # Create a rotation that aligns up axis with the face normal
        if up_axis.lower() == "x":
            up = om.MVector(1,0,0)
        elif up_axis.lower() == "y":
            up = om.MVector(0,1,0)
        else:
            up = om.MVector(0,0,1)
        
        # Compute rotation between vectors
        rot_axis = up ^ normal  # cross product
        rot_angle = up.angle(normal)
        
        # Convert to quaternion and then to Euler
        quat = om.MQuaternion(rot_angle, rot_axis.normal())
        euler = quat.asEulerRotation()
        
        # Apply rotation to the sphere
        cmds.xform(self.ctrl_grp, rotation=(om.MAngle(euler.x).asDegrees(),
                                     om.MAngle(euler.y).asDegrees(),
                                     om.MAngle(euler.z).asDegrees()),
                   worldSpace=True)

    def bounce(self, mobius_stair):
        start_frame = 1
        frame = start_frame
        time_gap = 10
        squash = 0.8
        stretch = 1.2

        for i,tri in enumerate(self.triangles):
            # Initial height
            cmds.setKeyframe(
                self.ctrl_grp, attribute="translateX", t=frame, v=tri[2][0]
            )
            cmds.setKeyframe(
                self.ctrl_grp, attribute="translateY", t=frame, v=tri[2][1]
            )
            cmds.setKeyframe(
                self.ctrl_grp, attribute="translateZ", t=frame, v=tri[2][2]
            )
            cmds.keyTangent(self.ctrl_grp, e=True, weightedTangents=True)
            cmds.keyTangent(self.ctrl_grp, e=True, a=True, t=(frame,), outWeight=8)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleY", t=frame, v=1)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleX", t=frame, v=1)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleZ", t=frame, v=1)

            # Approach to ground
            frame = int(frame + time_gap / 2 - 1)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleY", t=frame, v=stretch)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleX", t=frame, v=squash)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleZ", t=frame, v=squash)

            # Ground
            frame = int(frame + 1)
            cmds.setKeyframe(
                self.ctrl_grp, attribute="translateX", t=frame, v=tri[1][0]
            )
            cmds.setKeyframe(
                self.ctrl_grp, attribute="translateY", t=frame, v=tri[1][1]
            )
            cmds.setKeyframe(
                self.ctrl_grp, attribute="translateZ", t=frame, v=tri[1][2]
            )

            face_normal = mobius_stair.faces_normals_ordered[i % len(mobius_stair.faces_normals_ordered)]
            normal_vector = om.MVector(*face_normal)
            
            # Compute offset so bottom of ball touches stair
            offset_vector = normal_vector * self.radius
            ground_pos = om.MVector(*tri[1]) + offset_vector
            # Apply translation
            cmds.setKeyframe(self.ctrl_grp, attribute="translateX", t=frame, v=ground_pos.x)
            cmds.setKeyframe(self.ctrl_grp, attribute="translateY", t=frame, v=ground_pos.y)
            cmds.setKeyframe(self.ctrl_grp, attribute="translateZ", t=frame, v=ground_pos.z)
            
            # ALIGN PIVOT TO FACE NORMAL
            self.align_pivot_to_face_normal(normal_vector, up_axis="y")
            # Keyframe the rotation so the ball sticks
            cmds.setKeyframe(self.ctrl_grp, attribute="rotateX", t=frame)
            cmds.setKeyframe(self.ctrl_grp, attribute="rotateY", t=frame)
            cmds.setKeyframe(self.ctrl_grp, attribute="rotateZ", t=frame)
            
            #Squash in ground
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleY", t=frame, v=squash)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleX", t=frame, v=stretch)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleZ", t=frame, v=stretch)
            cmds.keyTangent(
                self.ctrl_grp,
                attribute="translateY",
                t=(frame,),
                itt="linear",
                ott="linear",
            )

            # Leaves ground
            frame = int(frame + 1)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleY", t=frame, v=stretch)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleX", t=frame, v=squash)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleZ", t=frame, v=squash)

            frame = int(frame + time_gap / 2)


my_stair = Mobius_stair()
my_stair.make_strip()
my_stair.get_face_center()
