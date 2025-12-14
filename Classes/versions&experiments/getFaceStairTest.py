import maya.api.OpenMaya as om
import maya.cmds as cmds


name = "mySphere"
radius = 1
cmds.polySphere(name=name, radius=radius)


# Function to get face center
def get_face_center(face_index):
    sel = om.MSelectionList()
    sel.add("Stairs")  
    dag = sel.getDagPath(0)

    # Make sure it's a mesh
    mesh_fn = om.MFnMesh(dag)

    # Get vertex indices for the face
    vert_ids = mesh_fn.getPolygonVertices(face_index)

    # Get world positions of the vertices
    points = [mesh_fn.getPoint(v, om.MSpace.kWorld) for v in vert_ids]

    # Compute average
    x = sum(p.x for p in points) / len(points)
    y = sum(p.y for p in points) / len(points)
    z = sum(p.z for p in points) / len(points)

    return (x, y, z)


# Example usage
face_idx = 1
center = get_face_center(face_idx)
print(center)
cmds.move(center[0], center[1], center[2], "mySphere")
