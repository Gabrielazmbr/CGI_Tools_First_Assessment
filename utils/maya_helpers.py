import maya.cmds as cmds


def clear_scene():
    """Deletes everything in the scene"""
    all_objs = cmds.ls(dag=True, long=True)
    if all_objs:
        cmds.delete(all_objs)
