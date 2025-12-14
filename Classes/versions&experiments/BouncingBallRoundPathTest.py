class BouncingBall:
    def __init__(self, radius=0.5, name='ball'):
        self.radius = radius
        self.name = name
        self.ball = cmds.polySphere(name= name, radius= radius)[0]
        cmds.move(0, 0.5, 0, self.ball)
        cmd.FreezeTransformations(self.ball, 0)
        self.plane = cmds.polyPlane(name='ground', width=6, height=40, subdivisionsX=2, subdivisionsY=10)[0]
        cmds.rotate(0, 90, 0, self.plane)
        cmds.nonLinear( self.plane, type='bend', curvature= 180)
        cmds.rotate(90, 90, 0, self.plane)
        cmds.delete(self.plane, ch=True)
        cmds.rotate(0, 0, 0, self.plane)
        cmds.FreezeTransformations(self.plane, 0)
        
        mat_name = "the_mat"
        mat = cmd.shadingNode("lambert", asShader=True, name=mat_name)
        cmd.setAttr(mat + ".color", 0.8, 0.4, 0.1, type="double3")
        sg = cmd.sets(renderable=True, noSurfaceShader=True, empty=True, name=mat_name + "SG")
        cmd.connectAttr(mat + ".outColor", sg + ".surfaceShader", f=True)
        cmd.sets(self.ball, e=True, forceElement=sg)
        
        
    def ball_rig(self):
        # Create the rotate and scale control curve
        self.rotate_ctrl = cmds.circle(name='rotate_ctrl', normal=(0, 0.5, 0), radius=1)[0]
        self.scale_ctrl = cmds.circle(name='scale_ctrl', normal=(0, 0.5, 0), radius=0.5)[0]
        
        # Match position of control to the ball
        cmds.delete(cmds.pointConstraint(self.ball, self.rotate_ctrl))
        
        # Parent the geometry under the control
        cmds.parent(self.ball, self.rotate_ctrl )
        cmds.parent(self.rotate_ctrl, self.scale_ctrl )
        
        # Create a group
        self.ctrl_grp = cmds.group(self.scale_ctrl, name='CTRL_GRP')
        
        # Freeze transforms
        cmds.FreezeTransformations(self.ctrl_grp, 0)
        
        return self.ctrl_grp
        
        
        
    def bounce (self):
        cmds.playbackOptions(min=1, max=200)
        frame = 1
        height = 12
        time_gap = 20

        squash = 0.8
        stretch = 1.2
        
        vertex_count = cmds.polyEvaluate(self.plane, vertex=True)
        vertex_indices = list(range(1, vertex_count, 3))  # your intentional skip
 

        
        for i in range (20):
            # Pick vertex index (wrap around if needed)
            vtx_idx = vertex_indices[i % len(vertex_indices)]
    
            # Sample vertex position dynamically
            vertex_pos = cmds.pointPosition(f"{self.plane}.vtx[{vtx_idx}]", world=True)
            pos_x = vertex_pos[0]
            pos_z = vertex_pos[2]
            
            # Initial height
            cmds.setKeyframe(self.ctrl_grp, attribute="translateY", t=frame, v=height)
            cmds.keyTangent(self.ctrl_grp, e=True, weightedTangents=True)
            cmds.keyTangent(self.ctrl_grp, e=True, a=True, t=(frame,), outWeight=8)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleY", t=frame, v=1)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleX", t=frame, v=1)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleZ", t=frame, v=1) 
            
            #Approach to ground
            frame = int(frame + time_gap / 2 -1)
            cmds.setKeyframe(self.ctrl_grp, attribute="translateY", t=frame, v=1)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleY", t=frame, v=stretch)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleX", t=frame, v=squash)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleZ", t=frame, v=squash)       
            
            #Ground
            frame = int(frame + 1)
            cmds.setKeyframe(self.ctrl_grp, attribute="translateY", t=frame, v=0)
            cmds.setKeyframe(self.ctrl_grp, attribute="translateX", t=frame, v=pos_x)
            cmds.setKeyframe(self.ctrl_grp, attribute="translateZ", t=frame, v=pos_z)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleY", t=frame, v=squash)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleX", t=frame, v=stretch)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleZ", t=frame, v=stretch)
            cmds.keyTangent(self.ctrl_grp, attribute="translateY", t=(frame,), itt="linear", ott="linear")

            #Leaves ground
            frame = int(frame + 1)
            cmds.setKeyframe(self.ctrl_grp, attribute="translateY", t=frame, v=1)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleY", t=frame, v=stretch)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleX", t=frame, v=squash)
            cmds.setKeyframe(self.scale_ctrl, attribute="scaleZ", t=frame, v=squash)

            frame = int(frame + time_gap / 2)
            
            
       
    


    
ball = BouncingBall()
ball.ball_rig()
ball.bounce()