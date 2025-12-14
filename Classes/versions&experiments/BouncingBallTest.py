import maya.cmds as cmd


class Ball:
    def __init__(self, radius, name):
        self.radius = radius
        self.name = name
        pX = 0
        pY = 1
        pZ = 0

        self.ball = cmd.polySphere(name=name, radius=radius)[0]
        cmd.move(pX, pY, pZ, self.ball)
        cmd.FreezeTransformations(self.ball, 0)

        mat_name = "the_mat"
        mat = cmd.shadingNode("lambert", asShader=True, name=mat_name)
        cmd.setAttr(mat + ".color", 0.8, 0.4, 0.1, type="double3")
        sg = cmd.sets(
            renderable=True, noSurfaceShader=True, empty=True, name=mat_name + "SG"
        )
        cmd.connectAttr(mat + ".outColor", sg + ".surfaceShader", f=True)
        cmd.sets(self.ball, e=True, forceElement=sg)

    def ball_rig(self):
        # Create the rotate and scale control curve
        self.rotate_ctrl = cmd.circle(
            name="rotate_ctrl", normal=(0, 0.5, 0), radius=1.5
        )[0]
        self.scale_ctrl = cmd.circle(name="scale_ctrl", normal=(0, 0.5, 0), radius=0.5)[
            0
        ]

        # Match position of control to the ball
        cmd.delete(cmd.pointConstraint(self.ball, self.rotate_ctrl))

        # Parent the geometry under the control
        cmd.parent(self.ball, self.rotate_ctrl)
        cmd.parent(self.rotate_ctrl, self.scale_ctrl)

        # Create a group
        self.ctrl_grp = cmd.group(self.scale_ctrl, name="CTRL_GRP")

        # Freeze transforms
        cmd.FreezeTransformations(self.ctrl_grp, 0)

        return self.ctrl_grp

    def bounce(self):
        start_frame = 1
        initial_height = 12
        bounces = 5
        dec = 0.6
        speed = 0.8

        frame = start_frame
        height = initial_height
        time_gap = 10
        length = 0
        length_interval = 4

        squash = 0.8
        stretch = 1.2
        h_aprox = 1

        cmd.setKeyframe(self.ctrl_grp, attribute="translateY", t=frame, v=height)
        cmd.keyTangent(self.ctrl_grp, e=True, weightedTangents=True)
        cmd.keyTangent(self.ctrl_grp, e=True, a=True, t=(frame,), outWeight=3)
        cmd.setKeyframe(self.ctrl_grp, attribute="translateX", t=frame, v=length)
        cmd.setKeyframe(self.scale_ctrl, attribute="scaleY", t=frame, v=1)
        cmd.setKeyframe(self.scale_ctrl, attribute="scaleX", t=frame, v=1)
        cmd.setKeyframe(self.scale_ctrl, attribute="scaleZ", t=frame, v=1)

        for _ in range(bounces):
            frame = frame + time_gap / 2 - 1
            cmd.setKeyframe(self.ctrl_grp, attribute="translateY", t=frame, v=h_aprox)
            cmd.setKeyframe(self.scale_ctrl, attribute="scaleY", t=frame, v=stretch)
            cmd.setKeyframe(self.scale_ctrl, attribute="scaleX", t=frame, v=squash)
            cmd.setKeyframe(self.scale_ctrl, attribute="scaleZ", t=frame, v=squash)

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
            cmd.setKeyframe(self.ctrl_grp, attribute="translateY", t=frame, v=h_aprox)
            cmd.setKeyframe(self.scale_ctrl, attribute="scaleY", t=frame, v=stretch)
            cmd.setKeyframe(self.scale_ctrl, attribute="scaleX", t=frame, v=squash)
            cmd.setKeyframe(self.scale_ctrl, attribute="scaleZ", t=frame, v=squash)

            length += length_interval
            height *= dec
            frame = frame + time_gap / 2 - 1
            cmd.setKeyframe(self.ctrl_grp, attribute="translateY", t=frame, v=height)
            cmd.keyTangent(self.ctrl_grp, e=True, weightedTangents=True)
            cmd.keyTangent(self.ctrl_grp, e=True, a=True, t=(frame,), outWeight=3)
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


def ground(name="ground", size=20):
    ground = cmd.polyPlane(w=200, h=200, n=name)[0]
    cmd.move(0, 0, 0, ground)
    cmd.color(ground, rgb=(0.5, 0.5, 0.5))
    return ground


theGround = ground()
bolita = Ball(1, "BouncingBall")
bolita.ball_rig()
bolita.bounce()
