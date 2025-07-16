#!/usr/bin/env python3

import os
from dataclasses import dataclass
from typing import Optional, Tuple
from xml.dom import minidom
from xml.etree import ElementTree as ET

import numpy
from numpy import radians

from multiverse_parser import logging
from pxr import UsdMujoco, UsdUrdf, Gf, UsdPhysics, UsdGeom, Usd, UsdShade
from ..factory import Factory
from ..factory import (JointBuilder, JointType,
                       GeomBuilder, GeomType,
                       PointsBuilder, MaterialProperty)
from ..utils import xform_cache, modify_name, merge_texture


def build_inertial(xform_prim: Usd.Prim, body: ET.Element) -> None:  # type: ignore
    mujoco_body_inertial_api = get_mujoco_inertial_api(xform_prim=xform_prim)

    inertial = ET.SubElement(body, "inertial")

    mass = mujoco_body_inertial_api.GetMassAttr().Get()
    pos = mujoco_body_inertial_api.GetPosAttr().Get()
    quat = mujoco_body_inertial_api.GetQuatAttr().Get()
    quat = numpy.array([quat.GetReal(), *quat.GetImaginary()])
    diaginertia = mujoco_body_inertial_api.GetDiaginertiaAttr().Get()

    inertial.set("mass", str(mass))
    inertial.set("pos", " ".join(map(str, pos)))
    inertial.set("quat", " ".join(map(str, quat)))
    inertial.set("diaginertia", " ".join(map(str, diaginertia)))


def get_mujoco_inertial_api(xform_prim: UsdGeom.Xform) -> UsdMujoco.MujocoBodyInertialAPI:  # type: ignore
    if not xform_prim.HasAPI(UsdPhysics.MassAPI):  # type: ignore
        return None

    physics_mass_api = UsdPhysics.MassAPI(xform_prim)  # type: ignore
    mass = physics_mass_api.GetMassAttr().Get()
    pos = physics_mass_api.GetCenterOfMassAttr().Get()
    quat = physics_mass_api.GetPrincipalAxesAttr().Get()
    diagonal_inertia = physics_mass_api.GetDiagonalInertiaAttr().Get()

    mujoco_inertial_api = UsdMujoco.MujocoBodyInertialAPI.Apply(xform_prim)  # type: ignore
    mujoco_inertial_api.CreateMassAttr(mass)
    mujoco_inertial_api.CreatePosAttr(pos)
    mujoco_inertial_api.CreateQuatAttr(quat)
    mujoco_inertial_api.CreateDiaginertiaAttr(diagonal_inertia)

    return mujoco_inertial_api


def get_mujoco_body_api(xform_prim: Usd.Prim,  # type: ignore
                        parent_xform_prim: Optional[Usd.Prim] = None) -> UsdMujoco.MujocoBodyAPI:  # type: ignore
    if parent_xform_prim is None:
        body_relative_transform = xform_cache.GetLocalToWorldTransform(xform_prim)
    else:
        parent_body_transform = xform_cache.GetLocalToWorldTransform(parent_xform_prim)
        body_transformation = xform_cache.GetLocalToWorldTransform(xform_prim)
        body_relative_transform = body_transformation * parent_body_transform.GetInverse()
    body_relative_transform = body_relative_transform.RemoveScaleShear()
    body_relative_pos = body_relative_transform.ExtractTranslation()
    body_relative_quat = body_relative_transform.ExtractRotationQuat()

    mujoco_body_api = UsdMujoco.MujocoBodyAPI.Apply(xform_prim)  # type: ignore
    mujoco_body_api.CreatePosAttr(body_relative_pos)
    mujoco_body_api.CreateQuatAttr(Gf.Quatf(body_relative_quat))  # type: ignore

    return mujoco_body_api


def get_mujoco_composite_api(points_builder: PointsBuilder) -> UsdMujoco.MujocoCompositeAPI:  # type: ignore
    points = points_builder.points
    points_prim = points.GetPrim()
    if points_prim.HasAPI(UsdMujoco.MujocoCompositeAPI):  # type: ignore
        return UsdMujoco.MujocoCompositeAPI(points_prim)  # type: ignore
    else:
        logging.warning(f"Composite {points_prim.GetName()} does not have MujocoCompositeAPI.")
        return None


def get_mujoco_joint_api(joint_builder: JointBuilder) -> UsdMujoco.MujocoJointAPI:  # type: ignore
    joint = joint_builder.joint
    joint_prim = joint.GetPrim()
    if joint_prim.HasAPI(UsdMujoco.MujocoJointAPI):  # type: ignore
        mujoco_joint_api = UsdMujoco.MujocoJointAPI(joint_prim)  # type: ignore
    else:
        mj_joint_type = "hinge" if joint_builder.type in [JointType.REVOLUTE, JointType.CONTINUOUS] \
            else "slide" if joint_builder.type == JointType.PRISMATIC \
            else "ball" if joint_builder.type == JointType.SPHERICAL \
            else None
        if mj_joint_type is None:
            raise NotImplementedError(f"Joint type {joint_builder.type} not supported.")

        mj_joint_pos = joint_builder.pos
        mj_joint_axis = joint_builder.quat.Transform(Gf.Vec3d([0.0, 0.0, 1.0]))  # type: ignore

        mujoco_joint_api = UsdMujoco.MujocoJointAPI.Apply(joint_prim)  # type: ignore
        mujoco_joint_api.CreateTypeAttr(mj_joint_type)
        mujoco_joint_api.CreatePosAttr(mj_joint_pos)
        mujoco_joint_api.CreateAxisAttr(Gf.Vec3f(*mj_joint_axis))  # type: ignore
        if joint_builder.type == JointType.PRISMATIC or joint_builder.type == JointType.REVOLUTE:
            if joint_builder.type == JointType.PRISMATIC:
                lower = joint.GetLowerLimitAttr().Get()
                upper = joint.GetUpperLimitAttr().Get()
            else:
                lower = radians(joint.GetLowerLimitAttr().Get())
                upper = radians(joint.GetUpperLimitAttr().Get())
            mujoco_joint_api.CreateRangeAttr(Gf.Vec2f(lower, upper))  # type: ignore

    return mujoco_joint_api


def get_urdf_joint_api(joint_builder: JointBuilder) -> UsdUrdf.UrdfJointAPI:  # type: ignore
    joint = joint_builder.joint
    joint_prim = joint.GetPrim()
    return UsdUrdf.UrdfJointAPI(joint_prim) if joint_prim.HasAPI(UsdUrdf.UrdfJointAPI) else None  # type: ignore


def get_mujoco_geom_api(geom_builder: GeomBuilder, merge_texture: bool) -> UsdMujoco.MujocoGeomAPI:  # type: ignore
    gprim = geom_builder.gprim
    gprim_prim = gprim.GetPrim()
    if gprim_prim.HasAPI(UsdMujoco.MujocoGeomAPI):  # type: ignore
        mujoco_geom_api = UsdMujoco.MujocoGeomAPI(gprim_prim)  # type: ignore
    else:
        geom_transformation = gprim.GetLocalTransformation().RemoveScaleShear()
        geom_pos = geom_transformation.ExtractTranslation()
        geom_quat = geom_transformation.ExtractRotationQuat()
        if geom_builder.type == GeomType.CUBE:
            if gprim_prim.HasAPI(UsdUrdf.UrdfGeometryBoxAPI):  # type: ignore
                urdf_geometry_box_api = UsdUrdf.UrdfGeometryBoxAPI(gprim_prim)  # type: ignore
                geom_size = urdf_geometry_box_api.GetSizeAttr().Get()
                geom_size = numpy.array([*geom_size]) / 2.0
            else:
                xform = UsdGeom.Xform(gprim_prim)  # type: ignore
                transformation = xform.GetLocalTransformation()
                geom_size = numpy.array([round(transformation.GetRow(i).GetLength(), 3) for i in range(3)])
                geom_size_mat = numpy.array([[transformation.GetRow(i)[j] for i in range(3)] for j in range(3)])
                det_geom_size = numpy.linalg.det(geom_size_mat)
                if det_geom_size < 0:
                    logging.warning(
                        f"Geom {gprim_prim.GetName()} has negative scale, flipping the sign from {geom_size} to {-geom_size}.")
                    geom_size = -geom_size
            geom_type = "box"
        elif geom_builder.type == GeomType.SPHERE:
            geom_size = numpy.array([gprim.GetRadiusAttr().Get(), 0.0, 0.0])
            geom_type = "sphere"
        elif geom_builder.type == GeomType.CYLINDER:
            geom_size = numpy.array([gprim.GetRadiusAttr().Get(),
                                     gprim.GetHeightAttr().Get() / 2, 0.0])
            geom_type = "cylinder"
        elif geom_builder.type == GeomType.CAPSULE:
            geom_size = numpy.array([gprim.GetRadiusAttr().Get(),
                                     gprim.GetHeightAttr().Get() / 2, 0.0])
            geom_type = "capsule"
        elif geom_builder.type == GeomType.MESH:
            if gprim_prim.HasAPI(UsdUrdf.UrdfGeometryMeshAPI):  # type: ignore
                urdf_geometry_mesh_api = UsdUrdf.UrdfGeometryMeshAPI(gprim_prim)  # type: ignore
                geom_size = urdf_geometry_mesh_api.GetScaleAttr().Get()
                geom_size = numpy.array([*geom_size]) if geom_size is not None else numpy.array([1.0, 1.0, 1.0])
            else:
                xform = UsdGeom.Xform(gprim_prim)  # type: ignore
                transformation = xform.GetLocalTransformation()
                geom_size = numpy.array([round(transformation.GetRow(i).GetLength(), 3) for i in range(3)])
                geom_size_mat = numpy.array([[transformation.GetRow(i)[j] for i in range(3)] for j in range(3)])
                det_geom_size = numpy.linalg.det(geom_size_mat)
                if det_geom_size < 0:
                    logging.warning(
                        f"Geom {gprim_prim.GetName()} has negative scale, flipping the sign from {geom_size} to {-geom_size}.")
                    geom_size = -geom_size
            geom_type = "mesh"
        else:
            raise NotImplementedError(f"Geom type {geom_builder.type} not implemented.")

        mujoco_geom_api = UsdMujoco.MujocoGeomAPI.Apply(gprim_prim)  # type: ignore
        mujoco_geom_api.CreatePosAttr(geom_pos)
        mujoco_geom_api.CreateQuatAttr(Gf.Quatf(geom_quat))  # type: ignore
        mujoco_geom_api.CreateSizeAttr(Gf.Vec3f(*geom_size))  # type: ignore
        mujoco_geom_api.CreateTypeAttr(geom_type)
        if geom_builder.type == GeomType.MESH:
            prepended_items = gprim_prim.GetPrimStack()[0].referenceList.prependedItems
            if len(prepended_items) != 1:
                raise NotImplementedError(f"Geom {gprim_prim.GetName()} has {len(prepended_items)} prepended items.")

            stage = gprim_prim.GetStage()
            mesh_file_path = prepended_items[0].assetPath
            mesh_name = os.path.splitext(os.path.basename(mesh_file_path))[0]
            mesh_name = add_scale_to_mesh_name(mesh_name=mesh_name, mesh_scale=geom_size)

            mujoco_asset_prim = stage.GetPrimAtPath("/mujoco/asset")
            mujoco_meshes_prim = stage.GetPrimAtPath("/mujoco/asset/meshes")
            mujoco_materials_prim = stage.GetPrimAtPath("/mujoco/asset/materials")
            mujoco_textures_prim = stage.GetPrimAtPath("/mujoco/asset/textures")
            mujoco_mesh_path = mujoco_meshes_prim.GetPath().AppendChild(mesh_name)
            if (not stage.GetPrimAtPath(mujoco_mesh_path).IsValid() or
                    not stage.GetPrimAtPath(mujoco_mesh_path).IsA(UsdMujoco.MujocoMesh)):  # type: ignore
                stage.GetRootLayer().Save()
                raise ValueError(f"Mesh {mujoco_mesh_path} does not exist in {stage.GetRootLayer().realPath}")
            mujoco_geom_api.CreateMeshRel().SetTargets([mujoco_mesh_path])

            if geom_builder.is_visible:
                if gprim_prim.HasAPI(UsdShade.MaterialBindingAPI):  # type: ignore
                    material_binding_api = UsdShade.MaterialBindingAPI(gprim_prim)  # type: ignore
                    material_path = material_binding_api.GetDirectBindingRel().GetTargets()[0]
                    material_name = material_path.name
                    mujoco_material_path = mujoco_materials_prim.GetPath().AppendChild(material_name)
                    if not stage.GetPrimAtPath(mujoco_material_path).IsA(UsdMujoco.MujocoMaterial):  # type: ignore
                        raise ValueError(f"Material {material_name} does not exist.")
                    mujoco_geom_api.CreateMaterialRel().SetTargets([mujoco_material_path])
                else:
                    texture_name = f"T_{gprim_prim.GetName().replace('SM_', '')}"
                    texture_file_path = os.path.join(os.path.dirname(gprim_prim.GetStage().GetRootLayer().realPath),
                                                     "tmp",
                                                     "textures",
                                                     f"{texture_name}.png")
                    if os.path.exists(os.path.normpath(texture_file_path)):
                        child_geom_subset_prims = [child_prim for child_prim in gprim_prim.GetChildren() if
                                                   child_prim.IsA(UsdGeom.Subset) and child_prim.HasAPI(  # type: ignore
                                                       UsdShade.MaterialBindingAPI)]  # type: ignore

                        if merge_texture:
                            logging.warning(
                                f"Geom {gprim_prim.GetName()} has no material, but has {len(child_geom_subset_prims)} subsets, will export the merged material.")
                            material_name = f"M_{gprim_prim.GetName()}"
                            mujoco_material_path = mujoco_materials_prim.GetPath().AppendChild(material_name)
                            mujoco_material = UsdMujoco.MujocoMaterial.Define(stage,  # type: ignore
                                                                              mujoco_material_path)

                            mujoco_texture_path = mujoco_textures_prim.GetPath().AppendChild(texture_name)
                            mujoco_material.CreateTextureRel().SetTargets([mujoco_texture_path])
                            mujoco_texture = UsdMujoco.MujocoTexture.Define(stage, mujoco_texture_path)  # type: ignore
                            mujoco_texture.CreateTypeAttr("2d")
                            mujoco_texture.CreateFileAttr(f"{texture_name}.png")

                            mujoco_geom_api.CreateMaterialRel().SetTargets([mujoco_material_path])
                        else:
                            logging.warning(
                                f"Geom {gprim_prim.GetName()} has no material, but has {len(child_geom_subset_prims)} subsets, will only export the first subset.")
                            for geom_subset_prim in child_geom_subset_prims:
                                material_binding_api = UsdShade.MaterialBindingAPI(geom_subset_prim)  # type: ignore
                                material_path = material_binding_api.GetDirectBindingRel().GetTargets()[0]
                                material_name = material_path.name
                                mujoco_material_path = mujoco_asset_prim.GetPath().AppendChild("materials").AppendChild(
                                    material_name)
                                if not stage.GetPrimAtPath(mujoco_material_path).IsA(UsdMujoco.MujocoMaterial):  # type: ignore
                                    raise ValueError(f"Material {material_name} does not exist.")
                                mujoco_geom_api.CreateMaterialRel().SetTargets([mujoco_material_path])
                                break

    return mujoco_geom_api


def add_scale_to_mesh_name(mesh_name: str, mesh_scale: numpy.ndarray) -> str:
    if not numpy.isclose(mesh_scale, numpy.array([1.0, 1.0, 1.0])).all():
        mesh_name += "_" + "_".join(map(str, mesh_scale))
    mesh_name = modify_name(mesh_name, "Mesh_")
    return mesh_name


@dataclass(frozen=True, eq=True)
class MeshFileProperty:
    scale: Tuple[float, float, float]
    has_texture_coordinate: bool


class MjcfExporter:
    def __init__(
            self,
            factory: Factory,
            file_path: str,
            merge_textures: bool = True,
    ) -> None:
        self._factory = factory
        self._file_path = file_path
        self._mesh_dir_abs_path = os.path.join(os.path.dirname(self.file_path), self.file_name)
        self._root = ET.Element("mujoco")
        self._body_dict = {}
        self._merge_textures = merge_textures

    def build(self) -> None:
        self._build_config()

        worldbody = ET.SubElement(self.root, "worldbody")
        self.body_dict["worldbody"] = worldbody

        world_builder = self.factory.world_builder
        first_body_builder = world_builder.body_builders[0]
        first_body_name = first_body_builder.xform.GetPrim().GetName()
        if first_body_name == "world":
            self.body_dict["world"] = worldbody
            for geom_builder in first_body_builder.geom_builders:
                self._build_geom(geom_builder=geom_builder, body=worldbody)
        else:
            self._build_body(body_name=first_body_name, parent_body_name="worldbody")

        body_builders = world_builder.body_builders
        reduces_body_builders = body_builders

        stop = False
        while not stop:
            stop = True
            for body_builder in body_builders:
                body_name = body_builder.xform.GetPrim().GetName()
                parent_body_name = body_builder.xform.GetPrim().GetParent().GetName()
                if (parent_body_name in self.body_dict and
                        body_name not in self.body_dict and
                        len(body_builder.joint_builders) == 0):
                    stop = False
                    self._build_body(body_name=body_name, parent_body_name=parent_body_name)
                    reduces_body_builders.remove(body_builder)
                for joint_builder in body_builder.joint_builders:
                    parent_body_name = joint_builder.parent_prim.GetName()
                    child_body_name = joint_builder.child_prim.GetName()
                    if parent_body_name in self.body_dict and child_body_name not in self.body_dict:
                        stop = False
                        self._build_body(body_name=child_body_name, parent_body_name=parent_body_name)
                        if self.factory.config.with_physics:
                            self._build_joint(joint_builder=joint_builder, body_name=child_body_name)
                        child_body_builder = world_builder.get_body_builder(child_body_name)
                        reduces_body_builders.remove(child_body_builder)
            body_builders = reduces_body_builders

        self._export_equality()

        self._build_asset()

        if self.factory.config.with_physics:
            self._move_free_bodies()

    def _build_config(self):
        self._import_mujoco()

        self._build_mujoco_asset_mesh_and_material_prims()

        model_name = UsdMujoco.Mujoco(self.mujoco_prim).GetModelAttr().Get()  # type: ignore
        self.root.set("model", model_name)

        compiler = ET.SubElement(self.root, "compiler")
        meshdir = os.path.join(self.file_name, "meshes")
        compiler.set("meshdir", meshdir)
        texturedir = os.path.join(self.file_name, "textures")
        compiler.set("texturedir", texturedir)
        compiler.set("angle", "radian")
        compiler.set("autolimits", "true")
        compiler.set("balanceinertia", "true")
        compiler.set("boundmass", "0.000001")
        compiler.set("boundinertia", "0.000001")

        default = ET.SubElement(self.root, "default")
        default_model = ET.SubElement(default, "default")
        default_model.set("class", f"{self.factory.config.model_name}")
        default_geom = ET.SubElement(default_model, "geom")
        default_geom.set(
            "group",
            "1"
        )

        default_visual = ET.SubElement(default, "default")
        default_visual.set("class", f"{self.factory.config.model_name}_visual")
        default_visual_geom = ET.SubElement(default_visual, "geom")
        default_visual_geom.set("contype", "0")
        default_visual_geom.set("conaffinity", "0")
        default_visual_geom.set(
            "group",
            "2"
        )

        default_collision = ET.SubElement(default, "default")
        default_collision.set("class", f"{self.factory.config.model_name}_collision")
        default_collision_geom = ET.SubElement(default_collision, "geom")
        default_collision_geom.set(
            "rgba",
            " ".join(
                map(
                    str, self.factory.config.default_rgba
                )
            ),
        )
        default_collision_geom.set(
            "group",
            "3"
        )

    def _build_asset(self) -> None:
        asset = ET.Element("asset")
        for i, elem in enumerate(self.root):
            if elem.tag == "default":
                self.root.insert(i + 1, asset)
                break
        else:
            asset = ET.SubElement(self.root, "asset")
        mujoco_meshes = [UsdMujoco.MujocoMesh(prim) for prim in self.mujoco_meshes_prim.GetChildren()  # type: ignore
                         if prim.IsA(UsdMujoco.MujocoMesh)]  # type: ignore
        for mujoco_mesh in mujoco_meshes:
            mesh = ET.SubElement(asset, "mesh")
            mesh.set("name", mujoco_mesh.GetPrim().GetName())
            tmp_mesh_path = mujoco_mesh.GetFileAttr().Get().path
            tmp_mesh_path = tmp_mesh_path.replace(".//", "/")
            tmp_mesh_relpath = os.path.relpath(tmp_mesh_path, self.factory.tmp_mesh_dir_path)

            mesh.set("file", tmp_mesh_relpath)
            scale = mujoco_mesh.GetScaleAttr().Get()
            mesh.set("scale", " ".join(map(str, scale)))

        stage = self.factory.world_builder.stage
        mujoco_materials = {}
        for gprim_prim in [prim for prim in stage.TraverseAll() if
                           prim.IsA(UsdGeom.Gprim) and prim.HasAPI(UsdMujoco.MujocoGeomAPI)]:  # type: ignore
            mujoco_geom_api = UsdMujoco.MujocoGeomAPI(gprim_prim)  # type: ignore
            for mujoco_material_path in mujoco_geom_api.GetMaterialRel().GetTargets():
                mujoco_material_prim = stage.GetPrimAtPath(mujoco_material_path)
                mujoco_material = UsdMujoco.MujocoMaterial(mujoco_material_prim)  # type: ignore
                mujoco_materials[mujoco_material_prim.GetName()] = mujoco_material
        mujoco_textures = {}
        for mujoco_material in mujoco_materials.values():
            for mujoco_texture_path in mujoco_material.GetTextureRel().GetTargets():
                mujoco_texture_prim = stage.GetPrimAtPath(mujoco_texture_path)
                mujoco_texture = UsdMujoco.MujocoTexture(mujoco_texture_prim)  # type: ignore
                mujoco_textures[mujoco_texture_prim.GetName()] = mujoco_texture

        for mujoco_material_name, mujoco_material in mujoco_materials.items():
            material = ET.SubElement(asset, "material")
            material.set("name", mujoco_material_name)
            if len(mujoco_material.GetTextureRel().GetTargets()) > 1:
                raise NotImplementedError(f"Material {mujoco_material_name} has "
                                          f"{len(mujoco_material.GetTextureRel().GetTargets())} textures.")
            for texture_path in mujoco_material.GetTextureRel().GetTargets():
                texture_prim = stage.GetPrimAtPath(texture_path)
                texture_name = texture_prim.GetName()
                material.set("texture", texture_name)

            if len(mujoco_material.GetTextureRel().GetTargets()) == 0:
                rgba = mujoco_material.GetRgbaAttr().Get()
                if rgba is not None:
                    material.set("rgba", " ".join(map(str, rgba)))
                emission = mujoco_material.GetEmissionAttr().Get()
                if emission is not None:
                    material.set("emission", str(emission))
                specular = mujoco_material.GetSpecularAttr().Get()
                if specular is not None:
                    material.set("specular", str(specular))

        for mujoco_texture_name, mujoco_texture in mujoco_textures.items():
            texture = ET.SubElement(asset, "texture")
            texture.set("name", mujoco_texture_name)
            texture_type = mujoco_texture.GetTypeAttr().Get()
            texture.set("type", texture_type)
            texture.set("file", mujoco_texture.GetFileAttr().Get().path)

    def _import_mujoco(self):
        stage = self.factory.world_builder.stage
        if not stage.GetPrimAtPath("/mujoco").IsValid():
            usd_mujoco = UsdMujoco.Mujoco.Define(stage, "/mujoco")  # type: ignore
            model_name = stage.GetDefaultPrim().GetName()
            usd_mujoco.CreateModelAttr(model_name)
            self._import_option()
            self._import_asset()
            self._import_equality()

    def _import_option(self):
        if not self.mujoco_prim.HasAPI(UsdMujoco.MujocoOptionAPI):  # type: ignore
            UsdMujoco.MujocoOptionAPI.Apply(self.mujoco_prim)  # type: ignore

    def _import_asset(self):
        stage = self.factory.world_builder.stage
        if not stage.GetPrimAtPath("/mujoco/asset").IsValid():
            UsdMujoco.MujocoAsset.Define(stage, "/mujoco/asset")  # type: ignore
            UsdMujoco.MujocoMesh.Define(stage, "/mujoco/asset/meshes")  # type: ignore
            UsdMujoco.MujocoMaterial.Define(stage, "/mujoco/asset/materials")  # type: ignore
            UsdMujoco.MujocoTexture.Define(stage, "/mujoco/asset/textures")  # type: ignore

    def _import_equality(self):
        stage = self.factory.world_builder.stage
        if not stage.GetPrimAtPath("/mujoco/equality").IsValid():
            UsdMujoco.MujocoEquality.Define(stage, "/mujoco/equality")  # type: ignore

    def _build_mujoco_asset_mesh_and_material_prims(self):
        stage = self.factory.world_builder.stage
        mesh_files = {}
        mesh_dir_name = os.path.dirname(stage.GetRootLayer().realPath)
        for prim in stage.TraverseAll():
            if not prim.IsA(UsdGeom.Mesh):  # type: ignore
                continue
            mujoco_mesh_api = UsdMujoco.MujocoGeomAPI(prim)  # type: ignore
            if len(mujoco_mesh_api.GetMaterialRel().GetTargets()) > 0:
                continue
            geom_is_visible = UsdGeom.Gprim(  # type: ignore
                prim).GetVisibilityAttr().Get() == UsdGeom.Tokens.inherited and UsdGeom.Gprim(  # type: ignore
                prim).GetPurposeAttr().Get() != UsdGeom.Tokens.guide  # type: ignore
            prepended_items = prim.GetPrimStack()[0].referenceList.prependedItems
            if len(prepended_items) > 0:
                for prepended_item in prepended_items:
                    mesh_file_path = prepended_item.assetPath
                    if not os.path.isabs(mesh_file_path):
                        mesh_file_path = os.path.join(mesh_dir_name, mesh_file_path)
                    if prim.HasAPI(UsdUrdf.UrdfGeometryMeshAPI):  # type: ignore
                        urdf_geometry_mesh_api = UsdUrdf.UrdfGeometryMeshAPI(prim)  # type: ignore
                        mesh_scale = urdf_geometry_mesh_api.GetScaleAttr().Get()
                        mesh_scale = tuple(mesh_scale) if mesh_scale is not None else (1.0, 1.0, 1.0)
                    else:
                        xform = UsdGeom.Xform(prim)  # type: ignore
                        transformation = xform.GetLocalTransformation()
                        mesh_scale = tuple(round(transformation.GetRow(i).GetLength(), 3) for i in range(3))
                        mesh_size_mat = numpy.array([[transformation.GetRow(i)[j] for i in range(3)] for j in range(3)])
                        det_geom_size = numpy.linalg.det(mesh_size_mat)
                        if det_geom_size < 0:
                            logging.warning(
                                f"Mesh {mesh_file_path} has negative scale, flipping the sign from {mesh_scale} to {tuple(-s for s in mesh_scale)}.")
                            mesh_scale = tuple(-s for s in mesh_scale)

                    texture_coordinate_attr = UsdGeom.PrimvarsAPI(prim).GetPrimvar("st")  # type: ignore
                    mesh_has_texture_coordinate = texture_coordinate_attr.HasValue() and texture_coordinate_attr.GetTypeName().cppTypeName == "VtArray<GfVec2f>"
                    mesh_file_property = MeshFileProperty(scale=mesh_scale,
                                                          has_texture_coordinate=mesh_has_texture_coordinate)
                    if geom_is_visible:
                        local_mesh_prim_path = prim.GetPath()
                        local_materials_prim = self.factory.world_builder.stage.GetPrimAtPath(
                            local_mesh_prim_path.AppendChild("Materials"))
                        if local_materials_prim.IsValid() and any(
                                [child_prim.IsA(UsdShade.Material) for child_prim in  # type: ignore
                                 local_materials_prim.GetChildren()]) \
                                or any(
                            [child_prim.IsA(UsdGeom.Subset) for child_prim in prim.GetChildren()]):  # type: ignore
                            mesh_stage = Usd.Stage.Open(mesh_file_path)  # type: ignore
                            mesh_prim = mesh_stage.GetDefaultPrim()
                            mesh_prim_path = mesh_prim.GetPath()
                            assert mesh_prim.IsValid(), f"Mesh prim is not valid in {mesh_file_path}."
                            mesh_materials_prim_path = None
                            if local_materials_prim.IsValid():
                                mesh_materials = UsdGeom.Scope.Define(mesh_stage,  # type: ignore
                                                                      mesh_prim_path.AppendChild("Materials"))
                                mesh_materials_prim = mesh_materials.GetPrim()
                                mesh_materials_prim_path = mesh_materials_prim.GetPath()
                                for local_material_prim in [child_prim for child_prim in
                                                            local_materials_prim.GetChildren() if
                                                            child_prim.IsA(UsdShade.Material)]:  # type: ignore
                                    material_name = local_material_prim.GetName()
                                    mesh_material = UsdShade.Material.Define(mesh_stage,  # type: ignore
                                                                             mesh_materials_prim_path.AppendChild(
                                                                                 material_name))
                                    mesh_material_prim = mesh_material.GetPrim()
                                    for material_prepended_item in local_material_prim.GetPrimStack()[
                                        0].referenceList.prependedItems:
                                        material_file_path = material_prepended_item.assetPath
                                        if not os.path.isabs(material_file_path):
                                            material_file_path = os.path.join(mesh_dir_name, material_file_path)
                                        material_rel_file_path = os.path.relpath(material_file_path,
                                                                                 os.path.dirname(mesh_file_path))
                                        material_prim_path = material_prepended_item.primPath
                                        mesh_material_prim.GetReferences().AddReference(f"./{material_rel_file_path}",
                                                                                        material_prim_path)

                            for local_geom_subset_prim in [child_prim for child_prim in prim.GetChildren() if
                                                           child_prim.IsA(UsdGeom.Subset)]:  # type: ignore
                                local_geom_subset = UsdGeom.Subset(local_geom_subset_prim)  # type: ignore
                                geom_subset_name = local_geom_subset_prim.GetName()
                                geom_subset = UsdGeom.Subset.Define(mesh_stage,  # type: ignore
                                                                    mesh_prim.GetPath().AppendChild(geom_subset_name))
                                geom_subset_prim = geom_subset.GetPrim()
                                for schema_api in local_geom_subset_prim.GetAppliedSchemas():
                                    if schema_api == "MaterialBindingAPI":
                                        assert mesh_materials_prim_path is not None, \
                                            f"Mesh {mesh_file_path} has geom subset {geom_subset_name} with MaterialBindingAPI, but no Materials prim found."
                                        local_material_binding_api = UsdShade.MaterialBindingAPI(  # type: ignore
                                            local_geom_subset_prim)
                                        material_paths = local_material_binding_api.GetDirectBindingRel().GetTargets()
                                        assert len(material_paths) == 1, \
                                            f"Geom subset {geom_subset_name} in {mesh_file_path} has {len(material_paths)} materials, expected 1."
                                        material_name = material_paths[0].name
                                        material_binding_api = UsdShade.MaterialBindingAPI.Apply(  # type: ignore
                                            geom_subset_prim)
                                        material_binding_api.GetDirectBindingRel().SetTargets(
                                            [mesh_materials_prim_path.AppendChild(material_name)])
                                geom_subset.CreateElementTypeAttr(UsdGeom.Tokens.face)  # type: ignore
                                geom_subset.CreateIndicesAttr(local_geom_subset.GetIndicesAttr().Get())
                                geom_subset.CreateFamilyNameAttr(local_geom_subset.GetFamilyNameAttr().Get())

                            mesh_stage.GetRootLayer().Save()

                            if self._merge_textures:
                                texture_name = f"T_{mesh_prim.GetName().replace('SM_', '')}.png"
                                texture_path = os.path.join(self.factory.tmp_texture_dir_path, texture_name)
                                mesh_file_dir = os.path.dirname(mesh_file_path)
                                mesh_file_name = os.path.splitext(os.path.basename(mesh_file_path))[0]
                                output_mesh_file_path = os.path.join(mesh_file_dir, f"{mesh_file_name}.obj")
                                logging.warning(
                                    f"Mesh {mesh_file_path} has materials, merging textures to {texture_path} and exporting to {output_mesh_file_path}.")
                                self.factory.export_mesh(in_mesh_file_path=mesh_file_path,
                                                         execute_cmd_between=merge_texture(
                                                             output_texture_path=texture_path),
                                                         out_mesh_file_path=output_mesh_file_path)
                                mesh_file_path = output_mesh_file_path

                    if mesh_file_path not in mesh_files:
                        mesh_files[mesh_file_path] = {mesh_file_property}
                    elif mesh_scale not in mesh_files[mesh_file_path]:
                        mesh_files[mesh_file_path].add(mesh_file_property)

        for mesh_file_path, mesh_file_properties in mesh_files.items():
            mesh_file_name = os.path.splitext(os.path.basename(mesh_file_path))[0]
            for mesh_file_property in mesh_file_properties:
                mesh_file_ext = "obj" if mesh_file_property.has_texture_coordinate else "stl"
                tmp_mesh_file_path = os.path.join(self.factory.tmp_mesh_dir_path,
                                                  mesh_file_ext,
                                                  f"{mesh_file_name}.{mesh_file_ext}")
                if not os.path.exists(tmp_mesh_file_path):
                    self.factory.export_mesh(in_mesh_file_path=mesh_file_path,
                                             out_mesh_file_path=tmp_mesh_file_path,
                                             execute_later=True)

                mesh_file_name_scaled = add_scale_to_mesh_name(mesh_name=mesh_file_name,
                                                               mesh_scale=numpy.array(mesh_file_property.scale))

                mujoco_mesh_path = self.mujoco_meshes_prim.GetPath().AppendChild(mesh_file_name_scaled)
                if stage.GetPrimAtPath(mujoco_mesh_path).IsValid():
                    continue
                mujoco_mesh = UsdMujoco.MujocoMesh.Define(stage, mujoco_mesh_path)  # type: ignore
                mujoco_mesh.CreateFileAttr(f"./{tmp_mesh_file_path}")
                mujoco_mesh.CreateScaleAttr(Gf.Vec3f(*mesh_file_property.scale))  # type: ignore

        self.factory.execute_cmds()

        materials = {}
        for material_prim in [child_prim for child_prim in stage.TraverseAll() if
                              child_prim.IsA(UsdShade.Material)]:  # type: ignore
            material_name = material_prim.GetName()
            if material_name in materials:
                continue
            materials[material_name] = MaterialProperty.from_prim(material_prim=material_prim)

        for material_name, material_property in materials.items():
            mujoco_material_path = self.mujoco_materials_prim.GetPath().AppendChild(material_name)
            if stage.GetPrimAtPath(mujoco_material_path).IsValid():
                continue

            mujoco_material = UsdMujoco.MujocoMaterial.Define(stage, mujoco_material_path)  # type: ignore
            if material_property.diffuse_color is not None and material_property.opacity is not None:
                if isinstance(material_property.diffuse_color, numpy.ndarray):
                    rgba = Gf.Vec4f(*material_property.diffuse_color.tolist(),
                                    material_property.opacity)  # type: ignore
                    mujoco_material.CreateRgbaAttr(rgba)

                    if material_property.emissive_color is not None and all(
                            [c != 0.0 for c in material_property.diffuse_color]):
                        emissions = [material_property.emissive_color[i] / material_property.diffuse_color[i]
                                     for i in range(3)]
                        if emissions[0] == emissions[1] == emissions[2]:
                            mujoco_material.CreateEmissionAttr(float(emissions[0]))

                elif isinstance(material_property.diffuse_color, str):
                    texture_name = os.path.splitext(os.path.basename(material_property.diffuse_color))[0]
                    mujoco_texture_path = self.mujoco_textures_prim.GetPath().AppendChild(texture_name)
                    mujoco_material.CreateTextureRel().SetTargets([mujoco_texture_path])

                    mujoco_texture = UsdMujoco.MujocoTexture.Define(stage, mujoco_texture_path)  # type: ignore
                    mujoco_texture.CreateTypeAttr("2d")
                    mujoco_texture.CreateFileAttr(f"{texture_name}.png")
                else:
                    raise NotImplementedError(f"Material {material_name} does not have a proper diffuse color.")

            specular_color = material_property.specular_color
            if (isinstance(specular_color, numpy.ndarray) and
                    specular_color[0] == specular_color[1] == specular_color[2]):
                specular = float(specular_color[0])
                mujoco_material.CreateSpecularAttr(specular)

    def _build_body(self, body_name: str, parent_body_name: str) -> None:
        parent_body = self.body_dict[parent_body_name]
        body = ET.SubElement(parent_body, "body")
        self.body_dict[body_name] = body

        body.set("name", body_name)

        world_builder = self.factory.world_builder
        body_builder = world_builder.get_body_builder(body_name)

        xform_prim = body_builder.xform.GetPrim()
        if self.factory.config.with_physics and xform_prim.HasAPI(UsdPhysics.MassAPI):  # type: ignore
            build_inertial(xform_prim=xform_prim, body=body)

        if xform_prim.HasAPI(UsdMujoco.MujocoBodyAPI):  # type: ignore
            mujoco_body_api = UsdMujoco.MujocoBodyAPI(xform_prim)  # type: ignore
            if parent_body_name == "world" or parent_body_name == "worldbody":
                if self.factory.config.with_physics and self.factory.config.fixed_base is False:
                    ET.SubElement(body, "freejoint")
        else:
            if parent_body_name == "world" or parent_body_name == "worldbody":
                mujoco_body_api = get_mujoco_body_api(xform_prim=xform_prim)
                if self.factory.config.with_physics and self.factory.config.fixed_base is False:
                    ET.SubElement(body, "freejoint")
            else:
                parent_body_builder = world_builder.get_body_builder(parent_body_name)
                parent_xform_prim = parent_body_builder.xform.GetPrim()
                mujoco_body_api = get_mujoco_body_api(xform_prim=xform_prim, parent_xform_prim=parent_xform_prim)

        pos = mujoco_body_api.GetPosAttr().Get()
        quat = mujoco_body_api.GetQuatAttr().Get()
        quat = numpy.array([quat.GetReal(), *quat.GetImaginary()])

        body.set("pos", " ".join(map(str, pos)))
        body.set("quat", " ".join(map(str, quat)))

        for geom_builder in body_builder.geom_builders:
            self._build_geom(geom_builder=geom_builder, body=body)

        for points_builder in body_builder.points_builders:
            self._build_composite(points_builder=points_builder, body=body)

    def _build_geom(self, geom_builder: GeomBuilder, body: ET.Element) -> None:
        mujoco_geom_api = get_mujoco_geom_api(geom_builder=geom_builder, merge_texture=self._merge_textures)

        gprim_prim = geom_builder.gprim.GetPrim()
        geom_name = gprim_prim.GetName()
        geom = ET.SubElement(body, "geom")
        geom.set("name", geom_name)
        geom_type = mujoco_geom_api.GetTypeAttr().Get()
        geom.set("type", geom_type)
        geom_pos = mujoco_geom_api.GetPosAttr().Get()
        geom.set("pos", " ".join(map(str, geom_pos)))
        geom_quat = mujoco_geom_api.GetQuatAttr().Get()
        geom_quat = numpy.array([geom_quat.GetReal(), *geom_quat.GetImaginary()])
        geom.set("quat", " ".join(map(str, geom_quat)))
        if geom_type != "mesh":
            geom_size = mujoco_geom_api.GetSizeAttr().Get()
            geom.set("size", " ".join(map(str, geom_size)))
        else:
            mesh_rel_path = mujoco_geom_api.GetMeshRel().GetTargets()[0]
            mesh_name = mesh_rel_path.name
            geom.set("mesh", mesh_name)

        if len(mujoco_geom_api.GetMaterialRel().GetTargets()) > 0:
            material_rel_path = mujoco_geom_api.GetMaterialRel().GetTargets()[0]
            material_name = material_rel_path.name
            geom.set("material", material_name)
        elif not gprim_prim.GetPrim().HasAPI(UsdPhysics.CollisionAPI):  # type: ignore
            rgba = geom_builder.rgba
            if rgba is not None:
                geom.set("rgba", " ".join(map(str, rgba)))

        geom_is_collidable = gprim_prim.GetPrim().HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(gprim_prim).GetCollisionEnabledAttr().Get()  # type: ignore
        geom_is_visible = UsdGeom.Gprim(gprim_prim).GetVisibilityAttr().Get() == UsdGeom.Tokens.inherited and UsdGeom.Gprim(gprim_prim).GetPurposeAttr().Get() != UsdGeom.Tokens.guide  # type: ignore
        assert geom_is_collidable or geom_is_visible, \
            f"Geom {geom_name} is neither collidable nor visible. Please check the USD prim {gprim_prim.GetPath()}."

        if geom_is_collidable and geom_is_visible:
            geom.set("class", f"{self.factory.config.model_name}")
        else:
            geom.set("class", f"{self.factory.config.model_name}_{'visual' if geom_is_visible else 'collision'}")

    def _build_composite(self, points_builder: PointsBuilder, body: ET.Element) -> None:
        mujoco_composite_api = get_mujoco_composite_api(points_builder=points_builder)
        if mujoco_composite_api is None:
            return

        composite = ET.SubElement(body, "composite")
        composite_type = mujoco_composite_api.GetTypeAttr().Get()
        composite.set("type", composite_type)
        composite_count = mujoco_composite_api.GetCountAttr().Get()
        composite_count = [int(count) for count in composite_count]
        composite.set("count", " ".join(map(str, composite_count)))
        composite_spacing = mujoco_composite_api.GetSpacingAttr().Get()
        composite.set("spacing", str(composite_spacing))
        composite_offset = mujoco_composite_api.GetOffsetAttr().Get()
        composite.set("offset", " ".join(map(str, composite_offset)))
        composite_prefix = mujoco_composite_api.GetPrefixAttr().Get()
        composite.set("prefix", composite_prefix)

        points = points_builder.points

        widths_attr = points.GetWidthsAttr()
        geom_width = None
        if widths_attr is not None:
            for width in widths_attr.Get():
                if geom_width is None:
                    geom_width = width
                elif geom_width != width:
                    raise ValueError("Composite widths are not the same.")

            geom = ET.SubElement(composite, "geom")
            geom.set("size", f"{geom_width / 2}")

            points_display_color_attr = points.GetDisplayColorAttr()
            points_display_opacity_attr = points.GetDisplayOpacityAttr()
            if points_display_color_attr is not None and points_display_opacity_attr is not None:
                points_display_color = points_display_color_attr.Get()[0]
                points_display_opacity = points_display_opacity_attr.Get()[0]
                geom.set("rgba", " ".join(map(str, points_display_color)) + f" {points_display_opacity}")

    def _build_joint(self, joint_builder: JointBuilder, body_name: str) -> None:
        mujoco_joint_api = get_mujoco_joint_api(joint_builder=joint_builder)
        if joint_builder.type == JointType.FIXED:
            return

        joint_prim = joint_builder.joint.GetPrim()
        joint_name = joint_prim.GetName()

        body = self.body_dict[body_name]
        joint = ET.SubElement(body, "joint")
        joint.set("name", joint_name)

        joint_type = mujoco_joint_api.GetTypeAttr().Get()
        joint_pos = mujoco_joint_api.GetPosAttr().Get()
        joint.set("type", joint_type)
        joint.set("pos", " ".join(map(str, joint_pos)))

        if joint_builder.type == JointType.PRISMATIC or joint_builder.type == JointType.REVOLUTE:
            joint_range = mujoco_joint_api.GetRangeAttr().Get()
            joint.set("range", " ".join(map(str, joint_range)))

        if joint_builder.type != JointType.SPHERICAL:
            joint_axis = mujoco_joint_api.GetAxisAttr().Get()
            joint.set("axis", " ".join(map(str, joint_axis)))

        urdf_joint_api = get_urdf_joint_api(joint_builder=joint_builder)
        if urdf_joint_api is not None and len(urdf_joint_api.GetJointRel().GetTargets()) > 0:
            stage = self.factory.world_builder.stage
            equality_prim = stage.GetPrimAtPath("/mujoco/equality")
            joint1_path = joint_prim.GetPath()
            joint2_path = urdf_joint_api.GetJointRel().GetTargets()[0]
            equality_joint_path = equality_prim.GetPath().AppendChild(f"{joint1_path.name}_{joint2_path.name}")
            equality_joint_prim = UsdMujoco.MujocoEqualityJoint.Define(stage, equality_joint_path)  # type: ignore
            equality_joint_prim.CreateJoint1Rel().SetTargets([joint1_path])
            equality_joint_prim.CreateJoint2Rel().SetTargets([joint2_path])
            a0 = urdf_joint_api.GetOffsetAttr().Get()
            a1 = urdf_joint_api.GetMultiplierAttr().Get()
            equality_joint_prim.CreatePolycoefAttr([a0, a1, 0.0, 0.0, 0.0])

    def _export_equality(self):
        stage = self.factory.world_builder.stage

        equality_prim = stage.GetPrimAtPath("/mujoco/equality")
        equality = ET.SubElement(self.root, "equality")
        for child_prim in equality_prim.GetChildren():
            if child_prim.IsA(UsdMujoco.MujocoEqualityJoint):  # type: ignore
                equality_joint_prim = UsdMujoco.MujocoEqualityJoint(child_prim)  # type: ignore
                joint1_path = equality_joint_prim.GetJoint1Rel().GetTargets()[0]
                joint2_path = equality_joint_prim.GetJoint2Rel().GetTargets()[0]
                if (not stage.GetPrimAtPath(joint1_path).IsA(UsdPhysics.Joint)  # type: ignore
                        or not stage.GetPrimAtPath(joint2_path).IsA(UsdPhysics.Joint)):  # type: ignore
                    raise ValueError(f"Equality joint {child_prim.GetName()} does not exist.")
                joint1_name = joint1_path.name
                joint2_name = joint2_path.name
                poly_coef = equality_joint_prim.GetPolycoefAttr().Get()

                equality_joint = ET.SubElement(equality, "joint")
                equality_joint.set("name", child_prim.GetName())
                equality_joint.set("joint1", joint1_name)
                equality_joint.set("joint2", joint2_name)
                equality_joint.set("polycoef", " ".join(map(str, poly_coef)))

    def _move_free_bodies(self) -> None:
        non_free_body_names = set()
        stage = self.factory.world_builder.stage
        for joint in [UsdPhysics.Joint(joint_prim) for joint_prim in stage.TraverseAll() if  # type: ignore
                      joint_prim.IsA(UsdPhysics.Joint)]:  # type: ignore
            for child_body_path in joint.GetBody1Rel().GetTargets():
                child_body_prim = stage.GetPrimAtPath(child_body_path)
                non_free_body_names.add(child_body_prim.GetName())
                for each_child_body_prim in child_body_prim.GetAllChildren():
                    if each_child_body_prim.IsA(UsdGeom.Xform):  # type: ignore
                        non_free_body_names.add(each_child_body_prim.GetName())

        free_body_names = set()
        for xform_prim in [prim for prim in stage.TraverseAll() if
                           prim.IsA(UsdGeom.Xform) and
                           prim.HasAPI(UsdMujoco.MujocoBodyInertialAPI)]:  # type: ignore
            if xform_prim.GetName() in non_free_body_names:
                continue
            sibling_body_names = [sibling_prim.GetName() for sibling_prim in xform_prim.GetParent().GetChildren()]
            if any([sibling_body_name in non_free_body_names for sibling_body_name in sibling_body_names]):
                continue
            free_body_names.add(xform_prim.GetName())

        worldbody = self.root.find("worldbody")
        to_move = []
        for top_level in list(worldbody):
            for child in top_level.iter():
                for sub in list(child):
                    if sub.tag == "body":
                        body_name = sub.attrib.get("name")
                        if body_name in free_body_names:
                            to_move.append((sub, child))

        for body, parent in to_move:
            parent.remove(body)
            worldbody.append(body)
            freejoint = ET.Element("freejoint")
            body_name = body.attrib.get("name")
            body_builder = self.factory.world_builder.get_body_builder(body_name)
            body_transform = xform_cache.GetLocalToWorldTransform(body_builder.xform.GetPrim())
            body_pos = [*body_transform.ExtractTranslation()]
            body_quat = body_transform.ExtractRotationQuat()
            body_quat = [body_quat.GetReal(), *body_quat.GetImaginary()]
            body.set("pos", " ".join(map(str, body_pos)))
            body.set("quat", " ".join(map(str, body_quat)))
            body.insert(0, freejoint)

    def export(self, keep_usd: bool = True) -> None:
        os.makedirs(name=os.path.dirname(self.file_path), exist_ok=True)

        rough_string = ET.tostring(self.root, "utf-8")
        parsed_string = minidom.parseString(rough_string)
        pretty_string = parsed_string.toprettyxml()

        with open(self.file_path, "w", encoding="utf-8") as file:
            file.write(pretty_string)

        if keep_usd:
            self.factory.save_tmp_model(usd_file_path=self.file_path.replace(".xml", ".usda"))
        else:
            self.factory.save_tmp_model(usd_file_path=self.file_path.replace(".xml", ".usda"),
                                        excludes=["usd", ".usda"])

    @property
    def file_path(self) -> str:
        return self._file_path

    @property
    def factory(self) -> Factory:
        return self._factory

    @property
    def file_name(self) -> str:
        return os.path.splitext(os.path.basename(self.file_path))[0]

    @property
    def mesh_dir_abspath(self) -> str:
        return self._mesh_dir_abs_path

    @property
    def root(self) -> ET.Element:
        return self._root

    @property
    def body_dict(self) -> dict:
        return self._body_dict

    @property
    def mujoco_prim(self) -> Usd.Prim:  # type: ignore
        return self.factory.world_builder.stage.GetPrimAtPath("/mujoco")

    @property
    def mujoco_asset_prim(self) -> Usd.Prim:  # type: ignore
        return self.mujoco_prim.GetChild("asset")

    @property
    def mujoco_meshes_prim(self) -> Usd.Prim:  # type: ignore
        return self.mujoco_asset_prim.GetChild("meshes")

    @property
    def mujoco_materials_prim(self) -> Usd.Prim:  # type: ignore
        return self.mujoco_asset_prim.GetChild("materials")

    @property
    def mujoco_textures_prim(self) -> Usd.Prim:  # type: ignore
        return self.mujoco_asset_prim.GetChild("textures")
