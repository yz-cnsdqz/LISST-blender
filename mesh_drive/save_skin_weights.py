# import bpy
# import numpy as np

# # Get the active object
# obj = bpy.context.active_object

# for bone_name in obj.vertex_groups.keys():
#     # Get the vertex group for the bone you want to extract the skin weights for
#     # bone_name = "Bone"
#     vertex_group = obj.vertex_groups.get(bone_name)

#     # Get the skin weights for each vertex in the vertex group
#     skin_weights = {}
#     for v in obj.data.vertices:
#         # Get the skin weight for the current vertex in the vertex group
#         weight = vertex_group.weight(v.index)
#         skin_weights[v.index] = weight

# # Print the skin weights to the console
# for v_idx, weight in skin_weights.items():
#     print("Vertex {}: weight = {}".format(v_idx, weight))


import bpy

# Get the active object
obj = bpy.context.active_object

# Get the vertex group for the bone you want to extract the skin weights for
bone_name = "root"
vertex_group = obj.vertex_groups.get(bone_name)

# Get the skin weights for each vertex in the vertex group
skin_weights = {}
for v in obj.data.vertices:
    # Check if the vertex is in the vertex group
    if vertex_group is not None and vertex_group in v.groups.keys():
        # Get the skin weight for the current vertex in the vertex group
        weight = vertex_group.weight(v.index)
        skin_weights[v.index] = weight

# Print the skin weights to the console
for v_idx, weight in skin_weights.items():
    print("Vertex {}: weight = {}".format(v_idx, weight))