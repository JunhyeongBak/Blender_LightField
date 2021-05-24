import bpy
from math import radians
import numpy as np
import addon_utils
import random

addon_utils.enable("io_import_images_as_planes")

SRC_PATH = '/home/junhyeong/Projects/Room3D/input_data'
DST_PATH = '/home/junhyeong/Projects/Room3D/output_data/sai'
DST_DEPTH_PATH = '/home/junhyeong/Projects/Room3D/output_data/depth'

nSTART = 0
nEND = 5
nSAI = 25
nSAI_ORG = 81
PICK_MOD = '9x9'
CENTER_ID = 12
SHIFT_VAL = 0.0025
ROTATE_VAL = 0

def shift_value_9x9(i, shift_val):
    if i>=0 and i<=8:
        tx = -4*shift_val
    elif i>=9 and i<=17:
        tx = -3*shift_val
    elif i>=18 and i<=26:
        tx = -2*shift_val
    elif i>=27 and i<=35:
        tx = -1*shift_val
    elif i>=36 and i<=44:
        tx = 0*shift_val
    elif i>=45 and i<=53:
        tx = 1*shift_val
    elif i>=54 and i<=62:
        tx = 2*shift_val
    elif i>=63 and i<=71:
        tx = 3*shift_val
    elif i>=72 and i<=80:
        tx = 4*shift_val
    else:
        tx = 20*shift_val
    if i==0 or (i%9==0 and i>8):
        ty = -4*shift_val
    elif i == 1 or (i-1)%9==0:
        ty = -3*shift_val
    elif i == 2 or (i-2)%9==0:
        ty = -2*shift_val
    elif i == 3 or (i-3)%9==0:
        ty = -1*shift_val
    elif i == 4 or (i-4)%9==0:
        ty = 0*shift_val
    elif i == 5 or (i-5)%9==0:
        ty = 1*shift_val
    elif i == 6 or (i-6)%9==0:
        ty = 2*shift_val
    elif i == 7 or (i-7)%9==0:
        ty = 3*shift_val
    elif i == 8 or (i-8)%9==0:
        ty = 4*shift_val
    else:
        ty = 40*shift_val
    return tx, -ty

def index_picker_5x5(i, pick_mode='9x9'):
    if pick_mode == '9x9':
        id_list = [20, 21, 22, 23, 24,
                    29, 30, 31, 32, 33,
                    38, 39, 40, 41, 42,
                    47, 48, 49, 50, 51,
                    56, 57, 58, 59, 60]
    elif pick_mode == '8x8':
        id_list = [9, 10, 11, 12, 13,
                    17, 18, 19, 20, 21,
                    25, 26, 27, 28, 29,
                    33, 34, 35, 36, 37,
                    41, 42, 43, 44, 45]
    return id_list[i]

def my_navigation(obj, rotate, location):
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
            
# < Init scene >
for scn in bpy.data.scenes:
    if len(bpy.data.scenes) != 1:
        bpy.ops.scene.delete() # Cannot delete a default scene                
scene = bpy.context.scene

scene.render.resolution_x = 512
scene.render.resolution_y = 512

# < Init camera >
for obj_id in range(nSTART, nEND):
    # < Reset seed by id >
    np.random.seed(obj_id);
    
    random_rotation = np.array((np.random.normal(0, 10), np.random.normal(0, 0), np.random.normal(0, 180)))
    if np.random.rand() > .5:
        random_location = np.array((np.random.normal(0, 0.2), np.random.normal(0, 0.2), np.random.normal(1.2, 0.05)))
    else:
        random_location = np.array((np.random.normal(5.2, 0.2), np.random.normal(2.5, 0.2), np.random.normal(1.2, 0.05)))
    cams = [obj for obj in bpy.data.objects if obj.name.startswith("Camera")]
    if len(cams) > 0:
        for cam in cams:
            bpy.ops.object.select_all(action='DESELECT')
            cam.select_set(True) # Blocked to use '.select = True' use setter and getter
            bpy.ops.object.delete()

    for i in range(nSAI_ORG):
        tx, ty = shift_value_9x9(i, SHIFT_VAL)
        rx, ry = shift_value_9x9(i, ROTATE_VAL)
        bpy.ops.object.camera_add(location=(tx, -0.4, ty), rotation=(radians(90-ry), radians(0), radians(0+rx)))
        #bpy.context.active_object.name = "Camera" + str(i)
        bpy.context.active_object.data.lens = 50 #50
                 
    cams = [obj for obj in bpy.data.objects if obj.name.startswith("Camera")]
    for cam in cams:
        cam = my_navigation(cam, random_rotation, random_location)

    # < Lamp >
    points = [obj for obj in bpy.data.objects if obj.name.startswith("Point")]
    for point in points:
        point.data.energy = np.random.normal(32, 8)
        print(point.data.energy)
        
    areas = [obj for obj in bpy.data.objects if obj.name.startswith("Area")]
    for area in areas:
        area.data.energy = np.random.normal(200, 50)
        print(area.data.energy)
    
    # < Face >
    faces = [obj for obj in bpy.data.objects if obj.name.startswith("Face")]
    if len(faces) > 0:
        for face in faces:
            bpy.ops.object.select_all(action='DESELECT')
            face.select_set(True) # Blocked to use '.select = True' use setter and getter
            bpy.ops.object.delete()

    bpy.ops.object.select_all(action='DESELECT')
    PATH = SRC_PATH+'/'+str(obj_id)+"_coarse.obj"
    bpy.ops.import_scene.obj(filepath=PATH)
    face = bpy.context.selected_objects[0]
    face.name = "Face"

    if face.type == 'MESH':
        bpy.context.view_layer.objects.active = face
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.merge_normals()
        bpy.ops.object.mode_set(mode='OBJECT')
        
    mat = bpy.data.materials.new(name='texture')
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    texImage = mat.node_tree.nodes.new('ShaderNodeTexImage')
    PATH = SRC_PATH+'/'+str(obj_id)+"texture.png"
    texImage.image = bpy.data.images.load(PATH)
    mat.node_tree.links.new(bsdf.inputs['Base Color'], texImage.outputs['Color'])
    if face.data.materials:
        face.data.materials[0] = mat
    else:
        face.data.materials.append(mat)

    face.location = (np.random.normal(0, 0.005), np.random.normal(0, 0), np.random.normal(0, 0.005)-0.01)
    face.scale = (np.random.normal(0.005333, 0.0005), np.random.normal(0.004335, 0.0005), np.random.normal(0.0043350, 0))
    face.rotation_euler = (radians(90-np.random.normal(0, 2)), radians(np.random.normal(0, 2)), radians(np.random.normal(0, 2)))
    
    face = my_navigation(face, random_rotation, random_location)
    
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
    
    # < Rendering >
    cameras = [obj for obj in bpy.data.objects if obj.name.startswith("Camera")]
    for i in range(nSAI):        
        i_pick = index_picker_5x5(i, PICK_MOD)
        scene.camera = cameras[i_pick]
        PATH = DST_PATH+'/sai'+str(obj_id)+'_'+str(i_pick)+'.png'
        bpy.context.scene.render.filepath = PATH
        #scene.render.use_multiview = True
        bpy.ops.render.render(write_still = True)
