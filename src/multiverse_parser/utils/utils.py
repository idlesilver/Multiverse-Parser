#!/usr/bin/env python3

import subprocess

import numpy
from scipy.spatial.transform import Rotation

from multiverse_parser import logging
from pxr import Usd, UsdGeom, Gf, UsdPhysics
from .mesh_exporter import export_usd
from .mesh_importer import import_usd

xform_cache = UsdGeom.XformCache()


def get_transform(pos: numpy.ndarray, quat: numpy.ndarray, scale: numpy.ndarray):
    pos = numpy.asarray(pos)
    quat = numpy.asarray(quat)
    scale = numpy.asarray(scale)
    mat = Gf.Matrix4d()
    mat.SetTranslateOnly(Gf.Vec3d(*pos))
    mat.SetRotateOnly(Gf.Quatd(quat[3], Gf.Vec3d(*quat[:3])))
    mat_scale = Gf.Matrix4d()
    mat_scale.SetScale(Gf.Vec3d(*scale))
    return mat_scale * mat


def diagonalize_inertia(inertia_tensor) -> (numpy.ndarray, numpy.ndarray):
    diagonal_inertia, eigenvectors = numpy.linalg.eigh(inertia_tensor)

    if numpy.linalg.det(eigenvectors) < 0:
        eigenvectors[:, 0] = -eigenvectors[:, 0]

    rotation_quat = Rotation.from_matrix(eigenvectors).as_quat()

    return diagonal_inertia, rotation_quat


def modify_name(in_name: str, replacement: str = None) -> str:
    out_name = in_name
    for special_char in [" ", "-", "~", ".", "/"]:
        if special_char in in_name:
            logging.warning(f"Name {in_name} contains {special_char}, replacing with _")
            out_name = out_name.replace(special_char, "_")

    if out_name and out_name[0].isdigit():
        if replacement is None:
            raise ValueError(f"Name {in_name} starts with digit and replacement is None.")
        out_name = replacement + out_name

    if out_name == "":
        if replacement is None:
            raise ValueError(f"Name {in_name} is empty and replacement is None.")
        out_name = replacement
    return out_name


def calculate_tet3_inertia_moment(v1: numpy.ndarray, v2: numpy.ndarray, v3: numpy.ndarray, i: int) -> float:
    return v1[i] ** 2 + v2[i] ** 2 + v3[i] ** 2 + v1[i] * v2[i] + v2[i] * v3[i] + v3[i] * v1[i]


def calculate_tet3_inertia_product(v1: numpy.ndarray, v2: numpy.ndarray, v3: numpy.ndarray, i: int, j: int) -> float:
    return (2 * v1[i] * v1[j] + 2 * v2[i] * v2[j] + 2 * v3[i] * v3[j] +
            v1[i] * v2[j] + v2[i] * v1[j] + v1[i] * v3[j] + v3[i] * v1[j] + v2[i] * v3[j] + v3[i] * v2[j])


def calculate_mesh_inertial(vertices: numpy.ndarray, faces: numpy.ndarray, density: float) -> \
        (float, numpy.ndarray, numpy.ndarray):
    # Initialize the mass to zero
    mass = 0.0

    # Initialize the inertia tensor to zeros
    Ixx = Iyy = Izz = Ixy = Ixz = Iyz = 0.0

    # Initialize the center of mass to zeros
    center_of_mass = numpy.zeros((1, 3))

    for face in faces:
        # Extract the vertices of the triangle face
        v1, v2, v3 = vertices[face]

        det = numpy.dot(v1, numpy.cross(v2, v3))

        triangle_volume = det / 6.0
        triangle_mass = density * triangle_volume
        mass += triangle_mass

        triangle_center_of_mass = (v1 + v2 + v3) / 4.0
        center_of_mass += triangle_center_of_mass * triangle_mass

        v100 = calculate_tet3_inertia_moment(v1, v2, v3, 0)
        v010 = calculate_tet3_inertia_moment(v1, v2, v3, 1)
        v001 = calculate_tet3_inertia_moment(v1, v2, v3, 2)

        Ixx += det * (v010 + v001)
        Iyy += det * (v100 + v001)
        Izz += det * (v100 + v010)
        Ixy += det * calculate_tet3_inertia_product(v1, v2, v3, 0, 1)
        Ixz += det * calculate_tet3_inertia_product(v1, v2, v3, 0, 2)
        Iyz += det * calculate_tet3_inertia_product(v1, v2, v3, 1, 2)

    center_of_mass /= mass

    Ixx = density * Ixx / 60.0
    Iyy = density * Iyy / 60.0
    Izz = density * Izz / 60.0
    Ixy = density * Ixy / 120.0
    Ixz = density * Ixz / 120.0
    Iyz = density * Iyz / 120.0

    inertia_tensor = numpy.array([
        [Ixx, -Ixy, -Ixz],
        [-Ixy, Iyy, -Iyz],
        [-Ixz, -Iyz, Izz]
    ])

    return mass, inertia_tensor, center_of_mass


def shift_inertia_tensor(mass: float,
                         inertia_tensor: numpy.ndarray,
                         pos: numpy.ndarray = numpy.zeros((1, 3)),
                         quat: numpy.ndarray = numpy.array([0.0, 0.0, 0.0, 1.0])) -> numpy.ndarray:
    # https://phys.libretexts.org/Bookshelves/Classical_Mechanics/Variational_Principles_in_Classical_Mechanics_(Cline)/13%3A_Rigid-body_Rotation/13.08%3A_Parallel-Axis_Theorem
    inertia_tensor_parallel = mass * (
        numpy.array([[pos[0][1] ** 2 + pos[0][2] ** 2, - pos[0][0] * pos[0][1], - pos[0][0] * pos[0][2]],
                     [- pos[0][0] * pos[0][1], pos[0][2] ** 2 + pos[0][0] ** 2, - pos[0][1] * pos[0][2]],
                     [- pos[0][0] * pos[0][2], - pos[0][1] * pos[0][2], pos[0][0] ** 2 + pos[0][1] ** 2]]))

    rotation_matrix = Rotation.from_quat(quat).as_matrix()
    return rotation_matrix @ inertia_tensor @ rotation_matrix.T + inertia_tensor_parallel


def shift_center_of_mass(center_of_mass: numpy.ndarray,
                         pos: numpy.ndarray = numpy.zeros((1, 3)),
                         quat: numpy.ndarray = numpy.array([0.0, 0.0, 0.0, 1.0])) -> numpy.ndarray:
    rotation = Rotation.from_quat(quat)
    return rotation.apply(center_of_mass) + pos


def triangulate_mesh(in_usd_file: str, out_usd_file: str) -> None:
    cmd = import_usd(in_usds=[in_usd_file])
    cmd += export_usd(out_usd=out_usd_file)
    cmd = ["blender",
           "--background",
           "--python-expr",
           f"import bpy"
           f"{cmd}"]
    process = subprocess.Popen(cmd)
    process.wait()


def extract_transform(prim: Usd.Prim) -> (numpy.ndarray, numpy.ndarray, numpy.ndarray):
    prim_transform = xform_cache.GetLocalToWorldTransform(prim)
    prim_quat = prim_transform.RemoveScaleShear().ExtractRotationQuat()
    prim_scale = numpy.array([prim_transform.GetRow(i).GetLength() for i in range(3)])
    if prim_transform.GetDeterminant3() < 0:
        prim_scale = -prim_scale
    prim_scale = numpy.linalg.norm(numpy.array([prim_quat.Transform(
        Gf.Vec3d(*v)) for v in numpy.eye(3) * prim_scale]), axis=0)
    prim_quat = numpy.array([prim_quat.GetImaginary()[0], prim_quat.GetImaginary()[1],
                             prim_quat.GetImaginary()[2], prim_quat.GetReal()])
    prim_translate = numpy.array([*prim_transform.ExtractTranslation()]) * prim_scale
    return prim_translate, prim_quat, prim_scale


def get_relative_transform(from_prim: Usd.Prim, to_prim: Usd.Prim) -> Gf.Matrix4d:  # type: ignore
    from_prim_transform = xform_cache.GetLocalToWorldTransform(from_prim)
    from_prim_rotation = from_prim_transform.RemoveScaleShear().ExtractRotation()
    from_prim_scale = numpy.array([from_prim_transform.GetRow(i).GetLength() for i in range(3)])
    if from_prim_transform.GetDeterminant3() < 0:
        from_prim_scale = -from_prim_scale
    from_prim_scale = numpy.linalg.norm(numpy.array([from_prim_rotation.TransformDir(
        Gf.Vec3d(*v)) for v in numpy.eye(3) * from_prim_scale]), axis=0)
    to_prim_transform = xform_cache.GetLocalToWorldTransform(to_prim)
    relative_transform = (to_prim_transform * from_prim_transform.GetInverse()).RemoveScaleShear()
    relative_translate = numpy.array([*relative_transform.ExtractTranslation()]) * numpy.array([*from_prim_scale])
    relative_transform.SetTranslateOnly(Gf.Vec3d(*relative_translate))
    return relative_transform


def get_translate_and_rotvec_from_transform(transform: Gf.Matrix4d) -> (numpy.ndarray, numpy.ndarray):
    """
    Extract translation and rotation vector from a Gf.Matrix4d transform.
    :param transform: Gf.Matrix4d transform
    :return: translation (numpy.ndarray), rotation vector (numpy.ndarray)
    """
    translation = numpy.array(transform.ExtractTranslation())
    rotation_quat = transform.RemoveScaleShear().ExtractRotationQuat()
    rotation = Rotation.from_quat([rotation_quat.GetImaginary()[0], rotation_quat.GetImaginary()[1],
                                   rotation_quat.GetImaginary()[2], rotation_quat.GetReal()])
    rotvec = rotation.as_rotvec(degrees=True)
    return translation, rotvec


def validate_joint_prim(joint_prim: Usd.Prim) -> None:
    """
    Validate the joint prim by checking the transforms between the joint, parent, and child bodies.
    :param joint_prim: The joint prim to validate.
    :raises ValueError: If the joint has inconsistent transforms.
    """
    stage = joint_prim.GetStage()
    joint = UsdPhysics.Joint(joint_prim)
    child_prim_path = joint.GetBody1Rel().GetTargets()[0]
    child_prim = stage.GetPrimAtPath(child_prim_path)

    parent_prim_paths = joint.GetBody0Rel().GetTargets()
    parent_prim = stage.GetPrimAtPath(parent_prim_paths[0]) if len(
        parent_prim_paths) > 0 else joint_prim.GetParent()
    parent_to_child_transform = get_relative_transform(from_prim=parent_prim,
                                                       to_prim=child_prim)
    _, _, joint_scale = extract_transform(joint_prim)

    parent_to_joint_quat = Gf.Quatd(joint.GetLocalRot0Attr().Get())
    parent_to_joint_translate = Gf.Vec3d(*[joint.GetLocalPos0Attr().Get()[i] * joint_scale[i] for i in range(3)])
    parent_to_joint_transform = Gf.Matrix4d().SetTranslateOnly(parent_to_joint_translate).SetRotateOnly(
        parent_to_joint_quat)
    child_to_joint_quat = Gf.Quatd(joint.GetLocalRot1Attr().Get())
    child_to_joint_translate = Gf.Vec3d(*[joint.GetLocalPos1Attr().Get()[i] * joint_scale[i] for i in range(3)])
    child_to_joint_transform = Gf.Matrix4d().SetTranslateOnly(child_to_joint_translate).SetRotateOnly(
        child_to_joint_quat)
    joint_to_child_transform = child_to_joint_transform.GetInverse()
    parent_to_child_translate_1, parent_to_child_rotvec_1 = get_translate_and_rotvec_from_transform(
        parent_to_child_transform)
    parent_to_child_translate_2, parent_to_child_rotvec_2 = get_translate_and_rotvec_from_transform(
        joint_to_child_transform * parent_to_joint_transform)
    different_rot = (Rotation.from_rotvec(parent_to_child_rotvec_2, degrees=True) * Rotation.from_rotvec(
        parent_to_child_rotvec_1, degrees=True).inv()).as_rotvec(degrees=True)
    if not numpy.allclose(parent_to_child_translate_1, parent_to_child_translate_2, atol=1e-2) or \
            numpy.linalg.norm(different_rot) > 1.0:
        parent_to_joint_translate, parent_to_joint_rotvec = get_translate_and_rotvec_from_transform(
            parent_to_joint_transform)
        joint_to_child_translate, joint_to_child_rotvec = get_translate_and_rotvec_from_transform(
            joint_to_child_transform)
        logging.error(f"Joint {joint_prim.GetName()} has inconsistent transforms: ")
        logging.error(f"Parent to child transform 1: {parent_to_child_translate_1}, {parent_to_child_rotvec_1}")
        logging.error(f"Parent to joint transform: {parent_to_joint_translate}, {parent_to_joint_rotvec}")
        logging.error(f"Joint to child transform: {joint_to_child_translate}, {joint_to_child_rotvec}")
        logging.error(f"Parent to child transform 2: {parent_to_child_translate_2}, {parent_to_child_rotvec_2}")
        logging.error(f"Difference: {parent_to_child_translate_1 - parent_to_child_translate_2}, {different_rot}")
        raise ValueError(f"Joint {joint_prim.GetPath()} has inconsistent transforms.")


def merge_texture(output_texture_path: str) -> str:
    return f"""
import bpy

def create_bake_uv_and_select(obj, bake_uv):
    uvs = obj.data.uv_layers
    if not uvs:
        return False
    
    if bake_uv not in uvs:
        uvs.new(name=bake_uv)
    
    bake_uv_map = uvs[bake_uv]
    bake_uv_map.active = True
    uvs.active = bake_uv_map
    return True

def create_bake_texture_and_image(bake_texture_name, bake_image_name, width, height):
    if bake_image_name in bpy.data.images:
        bake_image = bpy.data.images[bake_image_name]
        bpy.data.images.remove(bake_image)

    bake_image = bpy.data.images.new(name=bake_image_name, width=width, height=height)

    if bake_texture_name in bpy.data.textures:
        bake_texture = bpy.data.textures[bake_texture_name]
    else:
        bake_texture = bpy.data.textures.new(name=bake_texture_name, type='IMAGE')

    bake_texture.image = bake_image
    return bake_image

def check_materials(obj):
    empty_slots = False
    if len(obj.data.materials) == 0:
        return
    for slot in obj.material_slots:
        if slot.material is None:
            empty_slots = True
            break

    return not empty_slots

def add_bake_image_texture_node_to_materials_and_select(obj, bake_texture_node_name, bake_image):
    for material in obj.data.materials:
        if not material.use_nodes:
            material.use_nodes = True

        for node in material.node_tree.nodes:
            node.select = False

        existing_node = None
        for node in material.node_tree.nodes:
            if node.type == 'TEX_IMAGE' and node.name == bake_texture_node_name:
                existing_node = node
                break

        if existing_node is None:
            texture_node = material.node_tree.nodes.new(type='ShaderNodeTexImage')
            texture_node.name = bake_texture_node_name
            texture_node.location = (-300, 300)
            existing_node = texture_node
        
        existing_node.select = True
        existing_node.image = bake_image
        material.node_tree.nodes.active = existing_node

def pack_uv_islands():
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.uv.select_all(action='SELECT')
    bpy.ops.uv.smart_project(angle_limit=66, island_margin=0.03)

def set_bake_settings():
    bpy.context.scene.render.engine = 'CYCLES'
    bpy.context.scene.cycles.device = 'GPU'
    bpy.context.scene.cycles.bake_type = 'DIFFUSE'
    bpy.context.scene.render.bake.use_pass_direct = False
    bpy.context.scene.render.bake.use_pass_indirect = False
    bpy.context.preferences.addons['cycles'].preferences.compute_device_type = 'CUDA'

def bake_and_export(output_texture_path):
    obj = next((obj for obj in bpy.data.objects if obj.type == 'MESH'), None)
    if obj is None:
        return

    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj

    if not check_materials(obj):
        return
    
    bake_uv = "bake"
    bake_image = "BakeImage"
    bake_texture = "BakeTexture"

    success = create_bake_uv_and_select(obj, bake_uv)
    if not success:
        return
    bake_image = create_bake_texture_and_image(bake_texture, bake_image, 2048, 2048)
    add_bake_image_texture_node_to_materials_and_select(obj, bake_texture, bake_image)
    pack_uv_islands()
    set_bake_settings()

    bpy.ops.object.bake(type='DIFFUSE')

    bake_image.filepath_raw = bpy.path.abspath(output_texture_path)
    bake_image.file_format = 'PNG'
    bake_image.save()

    baked_mat = bpy.data.materials.new(name="BakedMaterial")
    baked_mat.use_nodes = True
    nodes = baked_mat.node_tree.nodes
    links = baked_mat.node_tree.links

    nodes.clear()
    
    tex_node = nodes.new(type='ShaderNodeTexImage')
    tex_node.image = bake_image
    tex_node.interpolation = 'Smart'
    tex_node.extension = 'CLIP'
    tex_node.location = (-400, 0)

    bsdf_node = nodes.new(type='ShaderNodeBsdfPrincipled')
    bsdf_node.location = (-150, 0)

    output_node = nodes.new(type='ShaderNodeOutputMaterial')
    output_node.location = (200, 0)

    links.new(tex_node.outputs['Color'], bsdf_node.inputs['Base Color'])
    links.new(bsdf_node.outputs['BSDF'], output_node.inputs['Surface'])
    obj.data.materials.clear()
    obj.data.materials.append(baked_mat)

    obj.data.uv_layers.active = obj.data.uv_layers[bake_uv]
    obj.data.uv_layers[bake_uv].active_render = True

    bpy.ops.object.mode_set(mode='OBJECT')

bake_and_export(r'{output_texture_path}')
"""