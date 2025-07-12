#!/usr/bin/env python3.10

import os
from typing import List, Optional, Dict

import numpy

from multiverse_parser import logging
from pxr import UsdPhysics, Usd, UsdGeom, Sdf, Gf
from ..factory import Factory, Configuration, InertiaSource
from ..factory import (WorldBuilder,
                       JointAxis, JointType, JointProperty,
                       GeomType, GeomProperty,
                       MeshProperty)
from ..utils import xform_cache, get_relative_transform, validate_joint_prim


def get_usd_mesh_file_path(gprim_prim: Usd.Prim) -> (str, Sdf.Path):  # type: ignore
    prepended_items = gprim_prim.GetPrimStack()[0].referenceList.prependedItems
    if len(prepended_items) == 0:
        # raise NotImplementedError(f"No prepended item found for {gprim_prim}.")
        return gprim_prim.GetStage().GetRootLayer().realPath, gprim_prim.GetPath()
    elif len(prepended_items) == 1:
        prepended_item = prepended_items[0]
        file_abspath = prepended_item.assetPath
        prim_path = prepended_item.primPath
        if not os.path.isabs(file_abspath):
            usd_file_path = gprim_prim.GetStage().GetRootLayer().realPath
            file_abspath = os.path.join(os.path.dirname(usd_file_path), file_abspath)
        return file_abspath, prim_path
    else:
        raise ValueError(f"Multiple prepended items found for {gprim_prim}.")


class LightwheelImporter(Factory):
    stage: Usd.Stage  # type: ignore
    parent_map: Dict[Usd.Prim, Usd.Prim]  # type: ignore
    name_map: Dict[Sdf.Path, str]  # type: ignore
    geom_body_map: Dict[Usd.Prim, Usd.Prim]  # type: ignore

    def __init__(
            self,
            file_path: str,
            with_visual: bool,
            with_collision: bool,
            inertia_source: InertiaSource = InertiaSource.FROM_SRC,
            default_rgba: Optional[numpy.ndarray] = None,
            black_list_names: Optional[List[str]] = None,
    ) -> None:
        self.black_list_names = black_list_names
        self._stage = Usd.Stage.Open(file_path)  # type: ignore

        default_prim = self.stage.GetDefaultPrim()
        model_name = default_prim.GetName()

        self.name_map = {default_prim.GetPath(): model_name}
        for prim in self.stage.Traverse():
            assert prim.IsValid(), f"Invalid prim found in stage: {prim.GetPath()}"
            if any(black_list_name in str(prim.GetPath()) for black_list_name in
                   black_list_names) if black_list_names is not None else False:
                continue
            if not prim.GetParent().IsA(UsdGeom.Xform) and not prim.GetParent().IsA(UsdGeom.Scope): # type: ignore
                continue
            if prim.IsA(UsdGeom.Xform) or prim.IsA(UsdGeom.Gprim) or prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Scope): # type: ignore
                prim_name = prim.GetName()
                idx = 0
                while prim_name in self.name_map.values():
                    prim_name = f"{prim.GetName()}_{idx}"
                    idx += 1
                self.name_map[prim.GetPath()] = prim_name
        self.parent_map = {}
        for joint_prim in [joint_prim for joint_prim in self.stage.Traverse() if
                           joint_prim.IsA(UsdPhysics.Joint)]:  # type: ignore
            child_prim = self.stage.GetPrimAtPath(
                UsdPhysics.Joint(joint_prim).GetBody1Rel().GetTargets()[0])  # type: ignore
            parent_prim_paths = UsdPhysics.Joint(joint_prim).GetBody0Rel().GetTargets()  # type: ignore
            if len(parent_prim_paths) == 0:
                parent_prim = child_prim.GetParent()
            else:
                parent_prim = self.stage.GetPrimAtPath(parent_prim_paths[0])
            self.parent_map[child_prim] = parent_prim
        self.geom_body_map = {}

        super().__init__(file_path=file_path, config=Configuration(
            model_name=model_name,
            fixed_base=True,
            root_name=None,
            with_physics=True,
            with_visual=with_visual,
            with_collision=with_collision,
            default_rgba=default_rgba if default_rgba is not None else numpy.array([0.9, 0.9, 0.9, 1.0]),
            inertia_source=inertia_source
        ))

    def import_model(self, save_file_path: Optional[str] = None) -> str:
        self._world_builder = WorldBuilder(self.tmp_usd_file_path)

        root_prim = self.stage.GetDefaultPrim()
        self.world_builder.add_body(root_prim.GetName())

        for prim_path in self.name_map.keys():
            prim = self.stage.GetPrimAtPath(prim_path)
            if prim.IsA(UsdGeom.Xform) or prim.IsA(UsdGeom.Scope): # type: ignore
                self._import_body(body_prim=prim)

        for prim_path in list(self.name_map.keys()):
            prim = self.stage.GetPrimAtPath(prim_path)
            if prim.IsA(UsdGeom.Gprim): # type: ignore
                self._import_geom(gprim_prim=prim)

        for joint_prim in [joint_prim for joint_prim in self.stage.Traverse() if
                           joint_prim.IsA(UsdPhysics.Joint)]:  # type: ignore
            if any(black_list_name in str(joint_prim.GetPath()) for black_list_name in
                   self.black_list_names) if self.black_list_names is not None else False:
                continue
            self._import_joint_and_inertial(joint_prim=joint_prim)

        self.world_builder.export()

        if save_file_path is not None:
            self.save_tmp_model(usd_file_path=save_file_path)
        return self.tmp_usd_file_path if save_file_path is None else save_file_path

    def _import_body(self, body_prim: Usd.Prim) -> None:  # type: ignore
        if body_prim != self.stage.GetDefaultPrim():
            imported_body_names = [body_builder.xform.GetPrim().GetName() for body_builder in
                                   self.world_builder.body_builders]
            body_name = self.name_map[body_prim.GetPath()]
            assert body_name not in imported_body_names, f"Body {body_name} already imported."
            parent_prim = self.parent_map.get(body_prim, body_prim.GetParent())
            while parent_prim.IsA(UsdGeom.Mesh) and parent_prim.GetName() not in imported_body_names: # type: ignore
                parent_prim = parent_prim.GetParent()
                assert not parent_prim.IsPseudoRoot(), f"Parent prim of {body_prim.GetPath()} is pseudo root, cannot find body."
            assert parent_prim.GetPath() in self.name_map, f"Parent prim {parent_prim.GetPath()} not found in name map."
            parent_body_name = self.name_map[parent_prim.GetPath()]
            if parent_body_name not in imported_body_names:
                self._import_body(body_prim=parent_prim)
            logging.info(f"Importing body: {body_prim.GetPath()} as {body_name} with parent {parent_body_name}...")
            body_builder = self.world_builder.add_body(body_name=body_name,
                                                       parent_body_name=parent_body_name)

            parent_to_body_transformation = get_relative_transform(from_prim=parent_prim,
                                                                   to_prim=body_prim)
            body_builder.xform.ClearXformOpOrder()
            body_builder.xform.AddTransformOp().Set(parent_to_body_transformation)

    def _import_geom(self, gprim_prim: Usd.Prim) -> None:  # type: ignore
        if not gprim_prim.IsA(UsdGeom.Mesh): # type: ignore
            logging.warning(f"Prim {gprim_prim.GetPath()} is not a mesh.")
            return
        if gprim_prim.HasAPI(UsdPhysics.RigidBodyAPI) and UsdPhysics.RigidBodyAPI(gprim_prim).GetRigidBodyEnabledAttr().Get(): # type: ignore
            parent_prim = self.parent_map.get(gprim_prim, gprim_prim.GetParent())
            if parent_prim.IsA(UsdGeom.Gprim) and parent_prim.GetParent() != gprim_prim.GetParent(): # type: ignore
                parent_prim = gprim_prim.GetParent()
            parent_body_name = self.name_map[parent_prim.GetPath()]
            imported_body_names = [body_builder.xform.GetPrim().GetName() for body_builder in
                                   self.world_builder.body_builders]
            if parent_body_name not in imported_body_names:
                self._import_body(body_prim=parent_prim)
            new_body_name = gprim_prim.GetName()
            if self.name_map[gprim_prim.GetPath()] in imported_body_names:
                body_builder = self.world_builder.get_body_builder(body_name=self.name_map[gprim_prim.GetPath()])
                xform_prim = body_builder.xform.GetPrim()
            else:
                idx = 0
                while new_body_name in self.name_map.values():
                    new_body_name = f"{gprim_prim.GetName()}_{idx}"
                    idx += 1

                logging.info(f"Importing body: {new_body_name} with parent {parent_body_name}...")
                body_builder = self.world_builder.add_body(body_name=new_body_name,
                                                           parent_body_name=parent_body_name)
                parent_to_body_transformation = get_relative_transform(from_prim=parent_prim,
                                                                       to_prim=gprim_prim)
                body_builder.xform.ClearXformOpOrder()
                body_builder.xform.AddTransformOp().Set(parent_to_body_transformation)
                xform_prim = body_builder.xform.GetPrim()
            self.name_map[xform_prim.GetPath()] = new_body_name
        else:
            xform_prim = gprim_prim.GetParent()
        while xform_prim.IsA(UsdGeom.Mesh): # type: ignore
            xform_prim = xform_prim.GetParent()
            assert not xform_prim.IsPseudoRoot(), f"Body prim of {gprim_prim.GetPath()} is pseudo root, cannot find body."
        if xform_prim.GetPath() not in self.name_map:
            logging.warning(
                f"Body prim {xform_prim.GetPath()} not found in name map, skipping geometry import for {gprim_prim.GetPath()}.")
            return
        body_name = self.name_map[xform_prim.GetPath()]
        body_builder = self.world_builder.get_body_builder(body_name=body_name)

        geom_name = self.name_map[gprim_prim.GetPath()]
        if geom_name not in body_builder._geom_builders:
            logging.info(f"Importing geometry: {gprim_prim.GetPath()} as {geom_name}...")
            geom_is_visible = True
            geom_is_collidable = gprim_prim.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(gprim_prim).GetCollisionEnabledAttr().Get() # type: ignore
            if gprim_prim.IsA(UsdGeom.Cube): # type: ignore
                geom_type = GeomType.CUBE
            elif gprim_prim.IsA(UsdGeom.Sphere): # type: ignore
                geom_type = GeomType.SPHERE
            elif gprim_prim.IsA(UsdGeom.Cylinder): # type: ignore
                geom_type = GeomType.CYLINDER
            elif gprim_prim.IsA(UsdGeom.Mesh): # type: ignore
                geom_type = GeomType.MESH
            else:
                raise NotImplementedError(f"Geometry type {gprim_prim.GetTypeName()} is not supported yet.")

            geom_rgba = self.config.default_rgba
            geom_density = 1000.0
            geom_property = GeomProperty(geom_type=geom_type,
                                         is_visible=geom_is_visible,
                                         is_collidable=geom_is_collidable,
                                         rgba=geom_rgba,
                                         density=geom_density)
            geom_builder = body_builder.add_geom(geom_name=geom_name,
                                                 geom_property=geom_property)
            geom_builder.build(approximation_method=UsdPhysics.MeshCollisionAPI(gprim_prim).GetApproximationAttr().Get()) # type: ignore
            geom_transform = xform_cache.GetLocalToWorldTransform(gprim_prim)
            geom_scale = numpy.array([geom_transform.GetRow(i).GetLength() for i in range(3)])
            if geom_transform.GetDeterminant3() < 0:
                logging.warning(
                    f"Geom {gprim_prim.GetPath()} has negative scale, flipping the sign from {geom_scale} to {-geom_scale}.")
                geom_scale = -geom_scale
            if not gprim_prim.HasAPI(UsdPhysics.RigidBodyAPI) or not UsdPhysics.RigidBodyAPI(gprim_prim).GetRigidBodyEnabledAttr().Get(): # type: ignore
                geom_transformation = get_relative_transform(from_prim=body_builder.xform.GetPrim(),
                                                             to_prim=gprim_prim)
                geom_pos = geom_transformation.ExtractTranslation()
                geom_pos = numpy.array([*geom_pos])
                geom_quat = geom_transformation.ExtractRotationQuat()
                geom_quat = numpy.array([*geom_quat.GetImaginary(), geom_quat.GetReal()])
                geom_builder.set_transform(pos=geom_pos, quat=geom_quat, scale=geom_scale)
            else:
                geom_builder.set_transform(scale=geom_scale)
            geom_builder.gprim.CreatePurposeAttr(UsdGeom.Gprim(gprim_prim).GetPurposeAttr().Get()) # type: ignore
            self.geom_body_map[gprim_prim] = body_builder.xform.GetPrim()
        else:
            geom_builder = body_builder.get_geom_builder(geom_name=geom_name)

        mesh_file_path, mesh_src_path = get_usd_mesh_file_path(gprim_prim=gprim_prim)
        mesh_name = f"SM_{self.name_map[gprim_prim.GetPath()]}"
        logging.info(f"Importing mesh: {mesh_src_path} from {mesh_file_path} as {mesh_name}...")
        mesh = UsdGeom.Mesh(gprim_prim) # type: ignore
        points = mesh.GetPointsAttr().Get()
        assert points is not None, "Mesh points cannot be None"
        normals = mesh.GetNormalsAttr().Get()
        if normals is None:
            normals = gprim_prim.GetAttribute("primvars:normals").Get()
        assert normals is not None, "Mesh normals cannot be None"

        face_vertex_counts = mesh.GetFaceVertexCountsAttr().Get()
        assert face_vertex_counts is not None, "Mesh face vertex counts cannot be None"
        face_vertex_indices = mesh.GetFaceVertexIndicesAttr().Get()
        assert face_vertex_indices is not None, "Mesh face vertex indices cannot be None"
        mesh_property = MeshProperty(points=numpy.array(points),
                                     normals=numpy.array(normals),
                                     face_vertex_counts=numpy.array(face_vertex_counts),
                                     face_vertex_indices=numpy.array(face_vertex_indices))
        mesh_property.mesh_file_name = mesh_name
        if mesh_property.points.size == 0 or mesh_property.face_vertex_counts.size == 0 or mesh_property.face_vertex_indices.size == 0:
            # TODO: Fix empty mesh
            logging.warning(f"Mesh {gprim_prim.GetName()} has no points or face vertex counts, skipping import.")
            return
        geom_builder.add_mesh(mesh_name=mesh_name,
                              mesh_property=mesh_property)

    def _import_joint_and_inertial(self, joint_prim: Usd.Prim) -> None:  # type: ignore
        if not joint_prim.IsA(UsdPhysics.RevoluteJoint) and not joint_prim.IsA(UsdPhysics.PrismaticJoint): # type: ignore
            return
        logging.info(f"Importing joint: {joint_prim.GetPath()}...")

        validate_joint_prim(joint_prim)

        joint = UsdPhysics.Joint(joint_prim) # type: ignore
        child_prim_path = joint.GetBody1Rel().GetTargets()[0]
        child_prim = self.stage.GetPrimAtPath(child_prim_path)
        joint_transform = xform_cache.GetLocalToWorldTransform(joint_prim)
        joint_rotation = joint_transform.RemoveScaleShear().ExtractRotation()
        joint_scale = numpy.array([joint_transform.GetRow(i).GetLength() for i in range(3)])
        if joint_transform.GetDeterminant3() < 0:
            logging.warning(f"Joint {joint_prim.GetPath()} has negative scale, flipping the sign from {joint_scale} to {-joint_scale}.")
            joint_scale = -joint_scale
        joint_scale = numpy.linalg.norm(numpy.array([joint_rotation.TransformDir(Gf.Vec3d(*v)) for v in numpy.eye(3) * joint_scale]), axis=0) # type: ignore
        if child_prim.IsA(UsdGeom.Gprim): # type: ignore
            parent_prim = self.parent_map[child_prim]
            child_prim = self.geom_body_map[child_prim]
            if parent_prim.IsA(UsdGeom.Gprim): # type: ignore
                parent_prim = self.geom_body_map[parent_prim]
            parent_prim_rotation = xform_cache.GetLocalToWorldTransform(parent_prim).RemoveScaleShear().ExtractRotation()
            parent_to_child_transform = get_relative_transform(from_prim=parent_prim,
                                                               to_prim=child_prim)
            parent_to_child_translate = parent_to_child_transform.ExtractTranslation()
            child_to_joint_pos = Gf.Vec3d(*[joint.GetLocalPos0Attr().Get()[i] * joint_scale[i] for i in range(3)]) # type: ignore
            child_to_joint_pos = parent_prim_rotation.GetInverse().TransformDir(child_to_joint_pos - parent_to_child_translate)
            child_to_joint_pos = numpy.array([*child_to_joint_pos])
        else:
            child_to_joint_pos = numpy.array([0.0, 0.0, 0.0])
        child_body_name = child_prim.GetName()
        joint_name = f"{child_body_name}_{joint.GetPrim().GetName()}"
        child_body_builder = self.world_builder.get_body_builder(body_name=child_body_name)
        child_prim = child_body_builder.xform.GetPrim()
        parent_prim = child_prim.GetParent()

        if joint_prim.IsA(UsdPhysics.RevoluteJoint): # type: ignore
            joint = UsdPhysics.RevoluteJoint(joint) # type: ignore
            joint_axis = joint.GetAxisAttr().Get()
            if numpy.isfinite(joint.GetLowerLimitAttr().Get()) and numpy.isfinite(joint.GetUpperLimitAttr().Get()):
                joint_type = JointType.REVOLUTE
            else:
                joint_type = JointType.CONTINUOUS
        elif joint_prim.IsA(UsdPhysics.PrismaticJoint): # type: ignore
            joint_type = JointType.PRISMATIC
            joint = UsdPhysics.PrismaticJoint(joint) # type: ignore
            joint_axis = joint.GetAxisAttr().Get()
        else:
            raise ValueError(f"Joint type {joint_prim} not supported.")

        joint_property = JointProperty(joint_parent_prim=parent_prim,
                                       joint_child_prim=child_prim,
                                       joint_pos=child_to_joint_pos,
                                       joint_axis=JointAxis.from_string(joint_axis),
                                       joint_type=joint_type)
        joint_builder = child_body_builder.add_joint(joint_name=joint_name,
                                                     joint_property=joint_property)
        if joint_prim.IsA(UsdPhysics.RevoluteJoint) or joint_prim.IsA(UsdPhysics.PrismaticJoint): # type: ignore
            joint_builder.joint.CreateUpperLimitAttr(joint.GetUpperLimitAttr().Get())
            joint_builder.joint.CreateLowerLimitAttr(joint.GetLowerLimitAttr().Get())

        child_body_builder.compute_and_set_inertial(inertia_source=self._config.inertia_source)

    @property
    def stage(self) -> Usd.Stage:  # type: ignore
        return self._stage
