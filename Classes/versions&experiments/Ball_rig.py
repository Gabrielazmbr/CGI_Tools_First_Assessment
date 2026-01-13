import maya.api.OpenMaya as om
import maya.cmds as cmds
import numpy as np

"""
    First version of a working rig for the ball. It is important to note
    that this rig is much simpler than a standard ball rig. This is because
    I am using it inside my code only and it is not fully designed for an animator
    to use it in a later stage, although it still have some basic capabilities.
"""


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

        # Center ball rig
        cmds.move(0, -1, 0, self.ctrl_grp)
        cmds.DeleteHistory(self.ctrl_grp)
        cmds.makeIdentity(self.ctrl_grp, apply=True, translate=True, rotate=True, scale=True)

        return self.ctrl_grp


ball = Ball(1, "BouncingBall")
ball.ball_rig()
