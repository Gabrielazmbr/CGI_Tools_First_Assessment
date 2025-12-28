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
        cmds.makeIdentity(
            self.ball, apply=True, translate=True, rotate=True, scale=True
        )

    def ball_rig(self):
        # Create the rotate and squash control curve
        self.ctrl_grp = cmds.circle(name="ctrl_grp", normal=(0, 0.5, 0), radius=2)[0]

        self.scale_ctrl = cmds.circle(
            name="scale_ctrl", normal=(0, 0.5, 0), radius=1.5
        )[0]
        cmds.move(0, 0.8, 0, self.scale_ctrl)

        self.rotate_ctrl = cmds.circle(
            name="rotate_ctrl", normal=(0, 0.3, 0), radius=1.2
        )[0]
        cmds.move(0, 1.3, 0, self.rotate_ctrl)

        # Delete History circles
        cmds.DeleteHistory(self.ctrl_grp)
        cmds.DeleteHistory(self.scale_ctrl)
        cmds.DeleteHistory(self.rotate_ctrl)

        # Freeze transformations circles
        cmds.makeIdentity(
            self.ctrl_grp, apply=True, translate=True, rotate=True, scale=True
        )
        cmds.makeIdentity(
            self.scale_ctrl, apply=True, translate=True, rotate=True, scale=True
        )
        cmds.makeIdentity(
            self.rotate_ctrl, apply=True, translate=True, rotate=True, scale=True
        )

        # Parent the geometry under the control
        cmds.parent(self.ball, self.rotate_ctrl)
        cmds.parent(self.rotate_ctrl, self.scale_ctrl)
        cmds.parent(self.scale_ctrl, self.ctrl_grp)

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
        """
        Helper function.
        """
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
        # Create a rotation that aligns up axis with the face normal.
        if up_axis.lower() == "x":
            up = om.MVector(1, 0, 0)
        elif up_axis.lower() == "y":
            up = om.MVector(0, 1, 0)
        else:
            up = om.MVector(0, 0, 1)

        # Ensure normal is MVector and normalized
        if not isinstance(normal, om.MVector):
            normal = om.MVector(normal)
        normal = normal.normal()

        # Compute rotation axis/angle
        rot_axis = up ^ normal  # cross product
        rot_angle = up.angle(normal)

        # Handle parallel vectors
        eps = 1e-8
        if rot_axis.length() < eps:
            dot = up * normal
            if dot > 0.0:
                quat = om.MQuaternion()
            else:
                # Opposite direction , needs a 180 rotation
                test_axis = up ^ om.MVector(1, 0, 0)
                if test_axis.length() < eps:
                    test_axis = up ^ om.MVector(0, 0, 1)
                quat = om.MQuaternion(math.pi, test_axis.normal())
        else:
            quat = om.MQuaternion(rot_angle, rot_axis.normal())

        # Keep quaternion sign continuous relative to previous quaternion
        prev_q = getattr(self, "prev_quat", None)
        if prev_q is not None:
            dotq = (
                prev_q.x * quat.x
                + prev_q.y * quat.y
                + prev_q.z * quat.z
                + prev_q.w * quat.w
            )
            if dotq < 0.0:
                quat = om.MQuaternion(-quat.x, -quat.y, -quat.z, -quat.w)
        self.prev_quat = om.MQuaternion(quat.x, quat.y, quat.z, quat.w)

        # Convert to Euler rotation
        euler = quat.asEulerRotation()
        new_degs = [
            om.MAngle(euler.x).asDegrees(),
            om.MAngle(euler.y).asDegrees(),
            om.MAngle(euler.z).asDegrees(),
        ]

        # Get previous Euler to make angles continuous.
        prev_euler = getattr(self, "last_euler", None)
        if prev_euler is None:
            try:
                prev_euler = list(cmds.getAttr(f"{self.ctrl_grp}.rotate")[0])
            except Exception:
                prev_euler = [0.0, 0.0, 0.0]

        # Adjust 360 jumps
        adjusted = []
        for n, p in zip(new_degs, prev_euler):
            diff = n - p
            # normalize diff into [-180, 180]
            while diff > 180.0:
                n -= 360.0
                diff = n - p
            while diff < -180.0:
                n += 360.0
                diff = n - p
            adjusted.append(n)

        # Apply adjusted rotation to the control group
        cmds.setAttr(f"{self.ctrl_grp}.rotateX", adjusted[0])
        cmds.setAttr(f"{self.ctrl_grp}.rotateY", adjusted[1])
        cmds.setAttr(f"{self.ctrl_grp}.rotateZ", adjusted[2])

        # Store for next call
        self.last_euler = adjusted

        return tuple(adjusted)

    def bounce(self, mobius_stair):
        start_frame = 1
        frame = start_frame
        time_gap = 15
        squash = 0.8
        stretch = 1.2

        for i, tri in enumerate(self.triangles):
            # Initial height
            cmds.setKeyframe(
                self.ctrl_grp, attribute="translateX", t=frame, v=tri[1][0]
            )
            cmds.setKeyframe(
                self.ctrl_grp, attribute="translateY", t=frame, v=tri[1][1]
            )
            cmds.setKeyframe(
                self.ctrl_grp, attribute="translateZ", t=frame, v=tri[1][2]
            )
            cmds.keyTangent(self.ctrl_grp, e=True, weightedTangents=True)
            cmds.keyTangent(self.ctrl_grp, e=True, a=True, t=(frame,), outWeight=8)
            # Align pivot to face normal
            face_normal = mobius_stair.faces_normals_ordered[
                i % len(mobius_stair.faces_normals_ordered)
            ]
            normal_vector = om.MVector(*face_normal)
            adjusted = self.align_pivot_to_face_normal(normal_vector, up_axis="y")
            # Keyframe the rotation
            cmds.setKeyframe(self.ctrl_grp, attribute="rotateX", t=frame, v=adjusted[0])
            cmds.setKeyframe(self.ctrl_grp, attribute="rotateY", t=frame, v=adjusted[1])
            cmds.setKeyframe(self.ctrl_grp, attribute="rotateZ", t=frame, v=adjusted[2])
            # Initial scale
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
                self.ctrl_grp, attribute="translateX", t=frame, v=tri[2][0]
            )
            cmds.setKeyframe(
                self.ctrl_grp, attribute="translateY", t=frame, v=tri[2][1]
            )
            cmds.setKeyframe(
                self.ctrl_grp, attribute="translateZ", t=frame, v=tri[2][2]
            )
            cmds.keyTangent(
                self.ctrl_grp,
                attribute="translateY",
                t=(frame,),
                itt="linear",
                ott="linear",
            )
            # Align pivot to face normal
            face_normal = mobius_stair.faces_normals_ordered[
                i % len(mobius_stair.faces_normals_ordered)
            ]
            normal_vector = om.MVector(*face_normal)
            adjusted = self.align_pivot_to_face_normal(normal_vector, up_axis="y")
            # Keyframe the rotation using explicit continuous values (prevents Euler wrap jumps)
            cmds.setKeyframe(self.ctrl_grp, attribute="rotateX", t=frame, v=adjusted[0])
            cmds.setKeyframe(self.ctrl_grp, attribute="rotateY", t=frame, v=adjusted[1])
            cmds.setKeyframe(self.ctrl_grp, attribute="rotateZ", t=frame, v=adjusted[2])
            # Squash in ground
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleY", t=frame, v=squash)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleX", t=frame, v=stretch)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleZ", t=frame, v=stretch)

            # Leaves ground
            frame = int(frame + 1)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleY", t=frame, v=stretch)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleX", t=frame, v=squash)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleZ", t=frame, v=squash)

            frame = int(frame + time_gap / 2)
