import math

import maya.api.OpenMaya as om
import maya.cmds as cmds
import numpy as np


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
        cmds.makeIdentity(self.ball, apply=True, translate=True, rotate=True, scale=True)

    def ball_rig(self):
        # Create the rotate and scale control curve
        self.ctrl_grp = cmds.circle(name="ctrl_grp", normal=(0, 0.5, 0), radius=2)[0]

        self.squash_ctrl = cmds.circle(name="squash_ctrl", normal=(0, 0.5, 0), radius=1.5)[0]
        cmds.move(0, 1, 0, self.squash_ctrl)

        self.rotate_ctrl = cmds.circle(name="rotate_ctrl", normal=(0, 0, 0), radius=1.2)[0]
        cmds.move(0, 1, 0, self.rotate_ctrl)

        # Delete History circles
        cmds.DeleteHistory(self.ctrl_grp)
        cmds.DeleteHistory(self.squash_ctrl)
        cmds.DeleteHistory(self.rotate_ctrl)

        # Freeze transformations circles
        cmds.makeIdentity(self.ctrl_grp, apply=True, translate=True, rotate=True, scale=True)
        cmds.makeIdentity(self.squash_ctrl, apply=True, translate=True, rotate=True, scale=True)
        cmds.makeIdentity(self.rotate_ctrl, apply=True, translate=True, rotate=True, scale=True)

        # Squash handle
        cmds.select(self.ball)
        self.squashHandle = cmds.nonLinear(self.ball, type="squash")
        cmds.setAttr(f"{self.squashHandle[1]}.translateY", 0)
        cmds.setAttr(f"{self.squashHandle[0]}.lowBound", 0)
        cmds.setAttr(f"{self.squashHandle[0]}.highBound", 2)

        # Squash Controller attributes
        cmds.select(self.squash_ctrl)
        cmds.addAttr(longName="squashAttr", minValue=-0.5, maxValue=0.5, keyable=True)
        cmds.connectAttr(f"{self.squash_ctrl}.squashAttr", f"{self.squashHandle[0]}.factor")

        # Parent the geometry under the control
        cmds.parent(self.ball, self.rotate_ctrl)
        cmds.parent(self.squashHandle, self.squash_ctrl)
        cmds.parent(self.squash_ctrl, self.ctrl_grp)
        cmds.parent(self.rotate_ctrl, self.ctrl_grp)

        return self.ctrl_grp

    def apex_from_face_normals(self, A, B, nA, nB, height):
        A = np.array(A, dtype=float)
        B = np.array(B, dtype=float)
        nA = np.array(nA, dtype=float)
        nB = np.array(nB, dtype=float)
        # center between the two faces
        mid = (A + B) * 0.5
        mid_normal = (nA + nB) * 0.5
        # Normalice
        mid_normal /= np.linalg.norm(mid_normal)

        apex = mid + mid_normal * height
        return apex

    def bounce_path(self, mobius_stair):
        self.triangles = []
        verts = mobius_stair.centers_ordered
        normals = mobius_stair.faces_normals_ordered
        self.height = 8
        for cycle in range(5):
            for i in range(len(verts) - 1):
                j = (i + 1) % len(verts)
                A = verts[i]
                C = verts[j]
                nA = normals[i]
                nC = normals[j]
                B = self.apex_from_face_normals(A, C, nA, nC, self.height)
                self.triangles.append((A, B, C))

            # Add final triangle
            A = verts[-1]
            C = verts[0]
            nA = normals[-1]
            nC = normals[0]

            if not np.allclose(A, C):
                B = self.apex_from_face_normals(A, C, nA, nC, self.height)
                self.triangles.append((A, B, C))

    def line_on_path(self, curve_name="path"):
        flat_pts = []
        for tri in self.triangles:
            for p in tri:
                arr = np.array(p, dtype=float)
                pt = (float(arr[0]), float(arr[1]), float(arr[2]))
                flat_pts.append(pt)
        print(self.triangles)
        print(flat_pts)
        # create the curve
        curve = cmds.curve(name=curve_name, p=flat_pts[0:130])

        return curve

    def align_pivot_to_face_normal(self, normal, up_axis="y"):
        # Create a rotation that aligns up axis with the face normal
        if up_axis.lower() == "x":
            up = om.MVector(1, 0, 0)
        elif up_axis.lower() == "y":
            up = om.MVector(0, 1, 0)
        else:
            up = om.MVector(0, 0, 1)

        # Compute rotation between vectors
        rot_axis = up ^ normal  # cross product
        rot_angle = up.angle(normal)

        # Convert to quaternion and then to Euler
        quat = om.MQuaternion(rot_angle, rot_axis.normal())
        euler = quat.asEulerRotation()

        # Apply rotation to the sphere
        cmds.xform(
            self.ctrl_grp,
            rotation=(
                om.MAngle(euler.x).asDegrees(),
                om.MAngle(euler.y).asDegrees(),
                om.MAngle(euler.z).asDegrees(),
            ),
            worldSpace=True,
        )

    def bounce(self, mobius_stair):
        start_frame = 1
        frame = start_frame
        time_gap = 10
        squash = 0.8
        stretch = 1.2

        for i, tri in enumerate(self.triangles):
            # Initial height
            cmds.setKeyframe(self.ctrl_grp, attribute="translateX", t=frame, v=tri[1][0])
            cmds.setKeyframe(self.ctrl_grp, attribute="translateY", t=frame, v=tri[1][1])
            cmds.setKeyframe(self.ctrl_grp, attribute="translateZ", t=frame, v=tri[1][2])
            cmds.keyTangent(self.ctrl_grp, e=True, weightedTangents=True)
            cmds.keyTangent(self.ctrl_grp, e=True, a=True, t=(frame,), outWeight=8)
            cmds.setKeyframe(self.ctrl_grp, attribute="scaleY", t=frame, v=1)
            cmds.setKeyframe(self.ctrl_grp, attribute="scaleX", t=frame, v=1)
            cmds.setKeyframe(self.ctrl_grp, attribute="scaleZ", t=frame, v=1)

            # Approach to ground
            frame = int(frame + time_gap / 2 - 1)
            cmds.setKeyframe(self.ctrl_grp, attribute="scaleY", t=frame, v=stretch)
            cmds.setKeyframe(self.ctrl_grp, attribute="scaleX", t=frame, v=squash)
            cmds.setKeyframe(self.ctrl_grp, attribute="scaleZ", t=frame, v=squash)

            # Ground
            frame = int(frame + 1)
            cmds.setKeyframe(self.ctrl_grp, attribute="translateX", t=frame, v=tri[2][0])
            cmds.setKeyframe(self.ctrl_grp, attribute="translateY", t=frame, v=tri[2][1])
            cmds.setKeyframe(self.ctrl_grp, attribute="translateZ", t=frame, v=tri[2][2])

            # Align pivot to face normal
            face_normal = mobius_stair.faces_normals_ordered[i % len(mobius_stair.faces_normals_ordered)]
            normal_vector = om.MVector(*face_normal)
            self.align_pivot_to_face_normal(normal_vector, up_axis="y")
            # Keyframe the rotation
            cmds.setKeyframe(self.ctrl_grp, attribute="rotateX", t=frame)
            cmds.setKeyframe(self.ctrl_grp, attribute="rotateY", t=frame)
            cmds.setKeyframe(self.ctrl_grp, attribute="rotateZ", t=frame)

            # Squash in ground
            cmds.setKeyframe(self.ctrl_grp, attribute="scaleY", t=frame, v=squash)
            cmds.setKeyframe(self.ctrl_grp, attribute="scaleX", t=frame, v=stretch)
            cmds.setKeyframe(self.ctrl_grp, attribute="scaleZ", t=frame, v=stretch)
            cmds.keyTangent(
                self.ctrl_grp,
                attribute="translateY",
                t=(frame,),
                itt="linear",
                ott="linear",
            )

            # Leaves ground
            frame = int(frame + 1)
            cmds.setKeyframe(self.ctrl_grp, attribute="scaleY", t=frame, v=stretch)
            cmds.setKeyframe(self.ctrl_grp, attribute="scaleX", t=frame, v=squash)
            cmds.setKeyframe(self.ctrl_grp, attribute="scaleZ", t=frame, v=squash)

            frame = int(frame + time_gap / 2)
