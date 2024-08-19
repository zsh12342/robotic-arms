import os
import meshio

def stl_to_obj(stl_filename, obj_filename):
    mesh = meshio.read(stl_filename)
    meshio.write(obj_filename, mesh)

def batch_convert_stl_to_obj(folder_path):
    stl_files = [f for f in os.listdir(folder_path) if f.endswith('.STL')]
    
    for stl_file in stl_files:
        stl_path = os.path.join(folder_path, stl_file)
        obj_file = os.path.splitext(stl_file)[0] + '.obj'
        obj_path = os.path.join(folder_path, obj_file)
        stl_to_obj(stl_path, obj_path)

# Replace 'folder_path' with the path to your folder containing STL files
folder_path = '/home/lemon/workspace/pbcwalk3_gen2/models/new_kuavo/meshes'
batch_convert_stl_to_obj(folder_path)
