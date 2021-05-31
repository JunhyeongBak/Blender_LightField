import bpy
import os
from math import radians
import numpy as np
import addon_utils
import random

addon_utils.enable("io_import_images_as_planes")

SRC_PATH = '/home/junhyeong/Projects/Room3D/input_data'
DST_PATH = '/home/junhyeong/Projects/Room3D/output_data/sai'
DST_DEP_PATH = '/home/junhyeong/Projects/Room3D/output_data/depth'

res = (512, 512) # Resolution of SAI (rows, cols)
n_sai = 25 # Number of SAI (only support square matrix)
foc_len = 50 # Focal length
i_center = 12 # Index of central SAI
pick_mode = '5x5' # SAI index picking mode
baseline = 0.0025 # Camera baseline
baserot = 0 # Camera rotation term (don't recommend to use this)
syn_start = 0 # Starting index of synthesis
syn_end = 2 # Range of synthesis

def shift_value_9x9(i, baseline):
    '''
    Return camera shift value according to SAI index
    '''
    if i>=0 and i<=8:
        tx = -4*baseline
    elif i>=9 and i<=17:
        tx = -3*baseline
    elif i>=18 and i<=26:
        tx = -2*baseline
    elif i>=27 and i<=35:
        tx = -1*baseline
    elif i>=36 and i<=44:
        tx = 0*baseline
    elif i>=45 and i<=53:
        tx = 1*baseline
    elif i>=54 and i<=62:
        tx = 2*baseline
    elif i>=63 and i<=71:
        tx = 3*baseline
    elif i>=72 and i<=80:
        tx = 4*baseline
    else:
        tx = 20*baseline
    if i==0 or (i%9==0 and i>8):
        ty = -4*baseline
    elif i == 1 or (i-1)%9==0:
        ty = -3*baseline
    elif i == 2 or (i-2)%9==0:
        ty = -2*baseline
    elif i == 3 or (i-3)%9==0:
        ty = -1*baseline
    elif i == 4 or (i-4)%9==0:
        ty = 0*baseline
    elif i == 5 or (i-5)%9==0:
        ty = 1*baseline
    elif i == 6 or (i-6)%9==0:
        ty = 2*baseline
    elif i == 7 or (i-7)%9==0:
        ty = 3*baseline
    elif i == 8 or (i-8)%9==0:
        ty = 4*baseline
    else:
        ty = 40*baseline
    return tx, -ty

def convert_index(i, pick_mode):
    '''
    Return converted index of SAI according to picking mode
    '''
    if pick_mode == '5x5':
        id_list = [20, 21, 22, 23, 24,
                    29, 30, 31, 32, 33,
                    38, 39, 40, 41, 42,
                    47, 48, 49, 50, 51,
                    56, 57, 58, 59, 60]
    else:
        id_list = list(range(81))
    return id_list[i]

def object_navigation(obj, rotate, location):
    '''
    Apply rotation and relocation on ojbect
    '''
    ang_x = rotate[0]
    theta_x = np.radians(ang_x)
    cos_x, sin_x = np.cos(theta_x), np.sin(theta_x)
    Rx = np.array(((1, 0, 0), (0, cos_x, sin_x), (0, -sin_x, cos_x)))
        
    ang_y = rotate[1]
    theta_y = np.radians(ang_y)
    cos_y, sin_y = np.cos(theta_y), np.sin(theta_y)
    Ry = np.array(((cos_y, 0, -sin_y), (0, 1, 0), (sin_y, 0, cos_y)))
        
    ang_z = rotate[2]
    theta_z = np.radians(ang_z)
    cos_z, sin_z = np.cos(theta_z), np.sin(theta_z)
    Rz = np.array(((cos_z, sin_z, 0), (-sin_z, cos_z, 0), (0, 0, 1)))
        
    obj.location = Rz @ Rx @ Ry @ obj.location
    obj.rotation_euler = obj.rotation_euler - np.array((radians(ang_x), radians(ang_y), radians(ang_z)))
    obj.location = np.array(obj.location) + location
    return obj
            
# < Initialize a scene >
# << Delete whole previous scene >>
for scn in bpy.data.scenes:
    if len(bpy.data.scenes) != 1: # Cannot delete a default scene   
        bpy.ops.scene.delete()

# << Genreate a scene >>                      
scene = bpy.context.scene

# << Configure scene >>
scene.render.resolution_x = res[1]
scene.render.resolution_y = res[0]
                
# << Configure scene compositor >>
scene.use_nodes = True
tree = scene.node_tree
links = tree.links

for node in tree.nodes:
    tree.nodes.remove(node)

rend_layers = tree.nodes.new('CompositorNodeRLayers')
normalize = tree.nodes.new(type="CompositorNodeNormalize")
links.new(rend_layers.outputs[2], normalize.inputs[0])
invert = tree.nodes.new(type="CompositorNodeInvert")
links.new(normalize.outputs[0], invert.inputs[1])
viewer = tree.nodes.new(type="CompositorNodeViewer")
links.new(invert.outputs[0], viewer.inputs[0])
links.new(rend_layers.outputs[1], viewer.inputs[1])
file_output = tree.nodes.new(type="CompositorNodeOutputFile")
links.new(invert.outputs[0], file_output.inputs[0])
scene.render.image_settings.color_depth = '16'

# << Print a scene pixel information >>
scale = scene.render.resolution_percentage / 100
pixels_in_u_per_mm = scene.render.resolution_x * scale / 32
pixels_in_v_per_mm = scene.render.resolution_y * scale * (32/24) / 24
pixel_size_in_u_direction = 1/pixels_in_u_per_mm
pixel_size_in_v_direction = 1/pixels_in_v_per_mm
print('scale: ', scale)
print('pixels_in_u_per_mm: ', pixels_in_u_per_mm)
print('pixels_in_v_per_mm: ', pixels_in_v_per_mm)
print('pixel_size_in_u_direction: ', pixel_size_in_u_direction)
print('pixel_size_in_v_direction: ', pixel_size_in_v_direction)

for i_syn in range(syn_start, syn_end):
    # < Reset seed according to synthesis index >
    np.random.seed(i_syn) # Using this, we can make same random condition on each synthesis
    
    # < Initialize cameras >
    # << Delete whole previous cameras >>
    cams = [obj for obj in bpy.data.objects if obj.name.startswith("Camera")]
    if len(cams) > 0:
        for cam in cams:
            bpy.ops.object.select_all(action='DESELECT')
            cam.select_set(True) # Have to use setter to select object
            bpy.ops.object.delete()

    # << Generate shifted multiple cameras >>
    for i_sai in range(n_sai):
        i_pick = convert_index(i_sai, pick_mode)
        tx, ty = shift_value_9x9(i_pick, baseline)
        rx, ry = shift_value_9x9(i_pick, baserot)
        bpy.ops.object.camera_add(location=(tx, -0.4, ty), 
                                    rotation=(radians(90-ry), radians(0), radians(0+rx)))
        #bpy.context.active_object.name = "Camera" + str(i)
        bpy.context.active_object.data.lens = foc_len
                 
    # << Set random lotation and location of cameras >>
    rand_rot = np.array((np.random.normal(0, 10), 
                        np.random.normal(0, 0), 
                        np.random.normal(0, 180)))
    if np.random.rand() > .5: # Room1 rocation case
        rand_loc = np.array((np.random.normal(0, 0.2), 
                            np.random.normal(0, 0.2),
                            np.random.normal(1.2, 0.05)))
    else: # Room2 rocation case
        rand_loc = np.array((np.random.normal(5.2, 0.2), 
                            np.random.normal(2.5, 0.2), 
                            np.random.normal(1.2, 0.05)))
    cams = [obj for obj in bpy.data.objects if obj.name.startswith("Camera")]
    for cam in cams:
        cam = object_navigation(cam, rand_rot, rand_loc)

    # < Initialize lamps >
    # << Point lamps >>
    point_lamps = [obj for obj in bpy.data.objects if obj.name.startswith("Point")]
    for lamp in point_lamps:
        lamp.data.energy = np.random.normal(32, 8)
        print(lamp.data.energy)

    # << Area lamps >>    
    area_lamps = [obj for obj in bpy.data.objects if obj.name.startswith("Area")]
    for lamp in area_lamps:
        lamp.data.energy = np.random.normal(200, 50)
        print(lamp.data.energy)
    
    # < Initialize a face object >
    # << Delete a previous face object >>
    faces = [obj for obj in bpy.data.objects if obj.name.startswith("Face")]
    if len(faces) > 0:
        for face in faces:
            bpy.ops.object.select_all(action='DESELECT')
            face.select_set(True)
            bpy.ops.object.delete()

    # << Load a new face object >>
    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.import_scene.obj(filepath=os.path.join(SRC_PATH, '{}_coarse.obj'.format(i_syn)))
    face = bpy.context.selected_objects[0]
    face.name = "Face"

    # << Merge mesh of face object >>
    if face.type == 'MESH':
        bpy.context.view_layer.objects.active = face
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.merge_normals()
        bpy.ops.object.mode_set(mode='OBJECT')

    # << Apply texture on face object >>    
    mat = bpy.data.materials.new(name='texture')
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    texImage = mat.node_tree.nodes.new('ShaderNodeTexImage')
    texImage.image = bpy.data.images.load(os.path.join(SRC_PATH, '{}texture.png'.format(i_syn)))
    mat.node_tree.links.new(bsdf.inputs['Base Color'], texImage.outputs['Color'])
    if face.data.materials:
        face.data.materials[0] = mat
    else:
        face.data.materials.append(mat)

    # << Set face object scale and location >>
    face.location = (np.random.normal(0, 0.005), 
                        np.random.normal(0, 0), 
                        np.random.normal(0, 0.005)-0.01)
    face.rotation_euler = (radians(90-np.random.normal(0, 2)), 
                            radians(np.random.normal(0, 2)), 
                            radians(np.random.normal(0, 2)))
    face.scale = (np.random.normal(0.005333, 0.0005), 
                    np.random.normal(0.004335, 0.0005), 
                    np.random.normal(0.0043350, 0))
    face = object_navigation(face, rand_rot, rand_loc)
    
    # < Rendering >
    cams = [obj for obj in bpy.data.objects if obj.name.startswith("Camera")]
    for i_sai in range(n_sai):        
        scene.camera = cams[i_sai]
        scene.render.filepath = os.path.join(DST_PATH, 'sai{}_{}.png'.format(i_syn, i_sai))
        file_output.base_path = os.path.join(DST_DEP_PATH, 'dep{}_{}'.format(i_syn, i_sai))
        bpy.ops.render.render(write_still = True)
        
