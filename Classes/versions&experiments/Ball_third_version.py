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
        for cycle in range(2):
            for i in range(len(verts) - 1):
                A = verts[i]
                B = verts[i + 1]
                nA = normals[i]
                nB = normals[i + 1]
                apex = self.apex_from_face_normals(A, B, nA, nB, self.height)
                self.triangles.append((A, B, apex))
        np_triangles = np.array(self.triangles)
        np_travel_points = np_triangles.reshape(-1, 3)
        self.travel_points = np_travel_points.tolist()
        print(self.travel_points)

    def orient_ball_from_face_normal(self, A, B, normal):
        # Path
        forward = np.array(B) - np.array(A)
        forward = forward / np.linalg.norm(forward)

        # Surface Normal
        up = np.array(normal) / np.linalg.norm(normal)

        # Perpendicular vector
        right = np.cross(up, forward)
        right = right / np.linalg.norm(right)

        # Match forward orthogonality
        forward = np.cross(right, up)

        # 3x3 Matrix
        M3 = np.array([right, up, forward])
        return M3

    def matrix_to_quaternion(self, M3):
        # Convert to Euler angles to use in the ball rotation commands
        M = om.MMatrix([
            M3[0, 0],
            M3[0, 1],
            M3[0, 2],
            0,
            M3[1, 0],
            M3[1, 1],
            M3[1, 2],
            0,
            M3[2, 0],
            M3[2, 1],
            M3[2, 2],
            0,
            0,
            0,
            0,
            1,
        ])

        tm = om.MTransformationMatrix(M)
        euler = tm.rotation()
        quat = euler.asQuaternion()

        return quat

    def quaternion_to_euler_degrees(self, quat):
        e = quat.asEulerRotation()
        e.reorderIt(om.MEulerRotation.kXYZ)
        return [np.degrees(e.x), np.degrees(e.y), np.degrees(e.z)]

    def unwrap_angles(self, angles, prev_angles):
        angles = np.array(angles, dtype=float)
        prev = np.array(prev_angles, dtype=float)
        delta = angles - prev
        # Map delta to [-180,180]
        delta = (delta + 180.0) % 360.0 - 180.0
        return (prev + delta).tolist()

    def bounce(self, mobius_stair):
        start_frame = 1
        frame = start_frame
        time_gap = 10

        if not hasattr(self, "triangles") or len(self.triangles) == 0:
            self.bounce_path(mobius_stair)

        normals = mobius_stair.faces_normals_ordered
        prev_angles = None
        prev_quat = None
        SMOOTH_AT_SEAM_T = 0.4

        for i, tri in enumerate(self.triangles):
            # Align pivot to face normal
            A, B, apex = tri
            normal = normals[i % len(normals)]
            rotation_matrix = self.orient_ball_from_face_normal(A, B, normal)
            q = self.matrix_to_quaternion(rotation_matrix)

            # Ensure quaternion sign continuity
            if prev_quat is not None:
                dot = prev_quat.x * q.x + prev_quat.y * q.y + prev_quat.z * q.z + prev_quat.w * q.w
                if dot < 0.0:
                    q = om.MQuaternion(-q.x, -q.y, -q.z, -q.w)

            if prev_quat is not None and (i % len(normals)) == 0:
                if SMOOTH_AT_SEAM_T > 0.0:
                    p = np.array([prev_quat.x, prev_quat.y, prev_quat.z, prev_quat.w], dtype=float)
                    c = np.array([q.x, q.y, q.z, q.w], dtype=float)
                    # ensure shortest hemisphere again
                    if p.dot(c) < 0:
                        c = -c
                    blended = (1.0 - SMOOTH_AT_SEAM_T) * p + SMOOTH_AT_SEAM_T * c
                    # normalize
                    mag = np.linalg.norm(blended)
                    if mag > 1e-8:
                        blended /= mag
                    q = om.MQuaternion(blended[0], blended[1], blended[2], blended[3])


            euler_angles = self.quaternion_to_euler_degrees(q)


            if prev_angles is not None:
                euler_angles = self.unwrap_angles(euler_angles, prev_angles)

            prev_quat = q
            prev_angles = euler_angles

            print("Normals:", normal)

            # Initial height
            cmds.setKeyframe(self.ctrl_grp, attribute="translateX", t=frame, v=tri[2][0])
            cmds.setKeyframe(self.ctrl_grp, attribute="translateY", t=frame, v=tri[2][1])
            cmds.setKeyframe(self.ctrl_grp, attribute="translateZ", t=frame, v=tri[2][2])
            cmds.keyTangent(self.ctrl_grp, e=True, weightedTangents=True)
            cmds.keyTangent(self.ctrl_grp, e=True, a=True, t=(frame,), outWeight=8)
            cmds.setKeyframe(self.ctrl_grp, attribute="rotateX", t=frame, v=euler_angles[0])
            cmds.setKeyframe(self.ctrl_grp, attribute="rotateY", t=frame, v=euler_angles[1])
            cmds.setKeyframe(self.ctrl_grp, attribute="rotateZ", t=frame, v=euler_angles[2])
            cmds.setKeyframe(self.squash_ctrl, attribute="squashAttr", t=frame, v=0)

            # Approach to ground
            frame = int(frame + time_gap / 2 - 1)
            cmds.setKeyframe(self.squash_ctrl, attribute="squashAttr", t=frame, v=0.5)

            # Ground
            frame = int(frame + 1)
            cmds.setKeyframe(self.ctrl_grp, attribute="translateX", t=frame, v=tri[1][0])
            cmds.setKeyframe(self.ctrl_grp, attribute="translateY", t=frame, v=tri[1][1])
            cmds.setKeyframe(self.ctrl_grp, attribute="translateZ", t=frame, v=tri[1][2])

            # Keyframe the rotation so the ball sticks
            cmds.setKeyframe(self.ctrl_grp, attribute="rotateX", t=frame, v=euler_angles[0])
            cmds.setKeyframe(self.ctrl_grp, attribute="rotateY", t=frame, v=euler_angles[1])
            cmds.setKeyframe(self.ctrl_grp, attribute="rotateZ", t=frame, v=euler_angles[2])

            # Squash in ground
            cmds.setKeyframe(self.squash_ctrl, attribute="squashAttr", t=frame, v=-0.5)
            cmds.keyTangent(
                self.ctrl_grp,
                attribute="translateY",
                t=(frame,),
                itt="linear",
                ott="linear",
            )

            # Leaves ground
            frame = int(frame + 1)
            cmds.setKeyframe(self.squash_ctrl, attribute="squashAttr", t=frame, v=0.5)

            frame = int(frame + time_gap / 2)
