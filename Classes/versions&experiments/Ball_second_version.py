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

    def bounce(self, mobius_stair):
        start_frame = 1
        frame = start_frame
        time_gap = 10
        squash = 0.8
        stretch = 1.2
        if not hasattr(self, "travel_triangles") or len(self.triangles) == 0:
            self.bounce_path(mobius_stair)
        print(f"triangles: {self.triangles}")

        for i, tri in enumerate(self.triangles):
            # Initial height
            cmds.setKeyframe(self.ctrl_grp, attribute="translateX", t=frame, v=tri[2][0])
            cmds.setKeyframe(self.ctrl_grp, attribute="translateY", t=frame, v=tri[2][1])
            cmds.setKeyframe(self.ctrl_grp, attribute="translateZ", t=frame, v=tri[2][2])
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
            cmds.setKeyframe(self.ctrl_grp, attribute="translateX", t=frame, v=tri[1][0])
            cmds.setKeyframe(self.ctrl_grp, attribute="translateY", t=frame, v=tri[1][1])
            cmds.setKeyframe(self.ctrl_grp, attribute="translateZ", t=frame, v=tri[1][2])

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
            
            
