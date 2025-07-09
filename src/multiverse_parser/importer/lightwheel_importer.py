#!/usr/bin/env python3.10

import os
from typing import List, Optional, Dict

import numpy

from multiverse_parser import logging
from ..factory import Factory, Configuration, InertiaSource
from ..factory import (WorldBuilder,
                       BodyBuilder,
                       JointBuilder, JointAxis, JointType, JointProperty,
                       GeomType, GeomProperty,
                       MeshProperty,
                       MaterialProperty)
from ..utils import xform_cache

from pxr import UsdPhysics, Usd, UsdGeom, UsdShade, Sdf, Gf, UsdUtils


def get_usd_mesh_file_path(gprim_prim: Usd.Prim) -> (str, Sdf.Path): # type: ignore
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


def get_relative(from_prim, to_prim):
    return xform_cache.GetLocalToWorldTransform(to_prim) * xform_cache.GetLocalToWorldTransform(from_prim).GetInverse()


class LightwheelImporter(Factory):
    stage: Usd.Stage # type: ignore
    parent_map: Dict[Usd.Prim, Usd.Prim] # type: ignore
    max_mesh_count: int = 1000
    mesh_count: int = 0
    name_map: Dict[Sdf.Path, str] # type: ignore

    def __init__(
            self,
            file_path: str,
            with_visual: bool,
            with_collision: bool,
            inertia_source: InertiaSource = InertiaSource.FROM_SRC,
            default_rgba: Optional[numpy.ndarray] = None,
            black_list_names: Optional[List[str]] = None,
    ) -> None:
        self._stage = Usd.Stage.Open(file_path) # type: ignore

        default_prim = self.stage.GetDefaultPrim()
        model_name = default_prim.GetName()

        self.name_map = {default_prim.GetPath(): model_name}
        for prim in self.stage.Traverse():
            assert prim.IsValid(), f"Invalid prim found in stage: {prim.GetPath()}"
            if any(black_list_name in prim.GetName() for black_list_name in black_list_names) if black_list_names is not None else False:
                continue
            if not prim.GetParent().IsA(UsdGeom.Xform) and not prim.GetParent().IsA(UsdGeom.Scope):
                continue
            if prim.IsA(UsdGeom.Xform) or prim.IsA(UsdGeom.Gprim) or prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Scope):
                prim_name = prim.GetName()
                idx = 0
                while prim_name in self.name_map.values():
                    prim_name = f"{prim.GetName()}_{idx}"
                    idx += 1
                self.name_map[prim.GetPath()] = prim_name

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
        self.parent_map = {}

    def import_model(self, save_file_path: Optional[str] = None) -> str:
        self._world_builder = WorldBuilder(self.tmp_usd_file_path)

        root_prim = self.stage.GetDefaultPrim()
        self.world_builder.add_body(root_prim.GetName())

        for joint_prim in [joint_prim for joint_prim in self.stage.Traverse() if joint_prim.IsA(UsdPhysics.Joint)]: # type: ignore
            child_prim = self.stage.GetPrimAtPath(UsdPhysics.Joint(joint_prim).GetBody1Rel().GetTargets()[0]) # type: ignore
            parent_prim_paths = UsdPhysics.Joint(joint_prim).GetBody0Rel().GetTargets() # type: ignore
            if len(parent_prim_paths) == 0:
                parent_prim = root_prim
            else:
                parent_prim = self.stage.GetPrimAtPath(parent_prim_paths[0])
            self.parent_map[child_prim] = parent_prim

        for prim_path in self.name_map.keys():
            prim = self.stage.GetPrimAtPath(prim_path)
            self._import_body(body_prim=prim)

        for joint_prim in [joint_prim for joint_prim in self.stage.Traverse() if joint_prim.IsA(UsdPhysics.Joint)]: # type: ignore
            self._import_joint(joint_prim=joint_prim)

        self.world_builder.export()

        if save_file_path is not None:
            self.save_tmp_model(usd_file_path=save_file_path)
        return self.tmp_usd_file_path if save_file_path is None else save_file_path
    
    def _import_body(self, body_prim: Usd.Prim) -> None: # type: ignore
        if body_prim != self.stage.GetDefaultPrim():
            imported_body_names = [body_builder.xform.GetPrim().GetName() for body_builder in self.world_builder.body_builders]
            body_name = self.name_map[body_prim.GetPath()]
            if body_name in imported_body_names:
                logging.info(f"Body {body_prim.GetPath()} already imported as {body_name}, skipping...")
                return
            logging.info(f"Importing body: {body_prim.GetPath()} as {self.name_map[body_prim.GetPath()]}...")
            if self.parent_map.get(body_prim) is None:
                parent_prim = body_prim.GetParent()
            else:
                parent_prim = self.parent_map[body_prim]
            if parent_prim.GetPath() not in self.name_map:
                logging.warning(f"Parent prim {parent_prim.GetPath()} not found in name map, skipping body import.")
                return
            parent_body_name = self.name_map[parent_prim.GetPath()]
            if parent_body_name not in [body_builder.xform.GetPrim().GetName() for body_builder in self.world_builder.body_builders]:
                self._import_body(body_prim=parent_prim)
            body_builder = self.world_builder.add_body(body_name=body_name,
                                                       parent_body_name=parent_body_name)
            
            parent_to_body_transformation = get_relative(from_prim=parent_prim, to_prim=body_prim)
            parent_scale = numpy.array([xform_cache.GetLocalToWorldTransform(parent_prim).GetRow(i).GetLength() for i in range(3)])
            body_scale = numpy.array([parent_to_body_transformation.GetRow(i).GetLength() for i in range(3)])
            body_scale_mat = numpy.array([[xform_cache.GetLocalToWorldTransform(body_prim).GetRow(i)[j] for i in range(3)] for j in range(3)])
            det_body_scale = numpy.linalg.det(body_scale_mat)
            if det_body_scale < 0:
                logging.warning(f"Body {body_prim.GetPath()} has negative scale, flipping the sign from {body_scale} to {-body_scale}.")
                body_scale = -body_scale

            mat_scale = Gf.Matrix4d()
            mat_scale.SetScale(Gf.Vec3d(*1.0/body_scale))
            parent_to_body_transformation = mat_scale * parent_to_body_transformation
            parent_to_body_translate = numpy.array([parent_to_body_transformation.GetRow(3)[i] for i in range(3)])
            parent_to_body_translate *= parent_scale
            parent_to_body_transformation.SetTranslateOnly(Gf.Vec3d(*parent_to_body_translate))
            body_builder.xform.ClearXformOpOrder()
            body_builder.xform.AddTransformOp().Set(parent_to_body_transformation)

            if body_prim.IsA(UsdGeom.Gprim):
                body_scale = numpy.array([xform_cache.GetLocalToWorldTransform(body_prim).GetRow(i).GetLength() for i in range(3)])
                body_scale_mat = numpy.array([[xform_cache.GetLocalToWorldTransform(body_prim).GetRow(i)[j] for i in range(3)] for j in range(3)])
                det_body_scale = numpy.linalg.det(body_scale_mat)
                if det_body_scale < 0:
                    logging.warning(f"Body {body_prim.GetPath()} has negative scale, flipping the sign from {body_scale} to {-body_scale}.")
                    body_scale = -body_scale
                geom_scale = Gf.Matrix4d()
                geom_scale.SetScale(Gf.Vec3d(*body_scale))
                self._import_geom(gprim_prim=body_prim, body_builder=body_builder, geom_scale=geom_scale)

    def _import_geom(self, gprim_prim: Usd.Prim, body_builder: BodyBuilder, geom_scale: Gf.Matrix4d) -> None:
        if not gprim_prim.IsA(UsdGeom.Mesh):
            logging.warning(f"Geometry {gprim_prim.GetName()} is not a mesh, skipping import.")
            return
        logging.info(f"Importing geometry: {gprim_prim.GetPath()}...")
        gprim = UsdGeom.Gprim(gprim_prim)
        # geom_is_visible = gprim.GetVisibilityAttr().Get() != UsdGeom.Tokens.invisible
        geom_is_collidable = gprim_prim.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(gprim_prim).GetCollisionEnabledAttr().Get()
        geom_is_visible = not geom_is_collidable 
        if gprim_prim.IsA(UsdGeom.Cube):
            geom_type = GeomType.CUBE
        elif gprim_prim.IsA(UsdGeom.Sphere):
            geom_type = GeomType.SPHERE
        elif gprim_prim.IsA(UsdGeom.Cylinder):
            geom_type = GeomType.CYLINDER
        elif gprim_prim.IsA(UsdGeom.Mesh):
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
        mesh_file_path, mesh_src_path = get_usd_mesh_file_path(gprim_prim=gprim_prim)
        logging.info(f"Importing mesh: {mesh_src_path} from {mesh_file_path}")
        mesh = UsdGeom.Mesh(gprim_prim)
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
        mesh_name = self.name_map[gprim_prim.GetPath()]
        mesh_property.mesh_file_name = mesh_name
        if mesh_property.points.size == 0 or mesh_property.face_vertex_counts.size == 0 or mesh_property.face_vertex_indices.size == 0:
            # TODO: Fix empty mesh
            logging.warning(f"Mesh {gprim_prim.GetName()} has no points or face vertex counts, skipping import.")
            return

        geom_builder = body_builder.add_geom(geom_name=f"{mesh_name}",
                                             geom_property=geom_property)
        geom_builder.build()
        geom_builder.add_mesh(mesh_name=mesh_name,
                              mesh_property=mesh_property)
        geom_builder.gprim.AddTransformOp().Set(geom_scale)

    def _import_joint(self, joint_prim: Usd.Prim) -> None: # type: ignore
        if not joint_prim.IsA(UsdPhysics.RevoluteJoint) and not joint_prim.IsA(UsdPhysics.PrismaticJoint):
            return
        logging.info(f"Importing joint: {joint_prim.GetPath()}...")
        joint = UsdPhysics.Joint(joint_prim)
        child_prim_path = joint.GetBody1Rel().GetTargets()[0]
        child_prim = self.stage.GetPrimAtPath(child_prim_path)
        child_prim_name = self.name_map[child_prim_path]
        child_body_builder = self.world_builder.get_body_builder(body_name=child_prim_name)
        joint_name = f"{child_prim_name}_{joint.GetPrim().GetName()}"
        parent_prim = self.parent_map[child_prim]

        if joint_prim.IsA(UsdPhysics.RevoluteJoint):
            joint = UsdPhysics.RevoluteJoint(joint)
            joint_axis = joint.GetAxisAttr().Get()
            if numpy.isfinite(joint.GetLowerLimitAttr().Get()) and numpy.isfinite(joint.GetUpperLimitAttr().Get()):
                joint_type = JointType.REVOLUTE
            else:
                joint_type = JointType.CONTINUOUS
        elif joint_prim.IsA(UsdPhysics.PrismaticJoint):
            joint_type = JointType.PRISMATIC
            joint = UsdPhysics.PrismaticJoint(joint)
            joint_axis = joint.GetAxisAttr().Get()
        else:
            raise ValueError(f"Joint type {joint_prim} not supported.")
        
        

        body1_to_body2_transform = get_relative(from_prim=parent_prim, to_prim=child_prim)

        body_scale = numpy.array([body1_to_body2_transform.GetRow(i).GetLength() for i in range(3)])
        body_scale_mat = numpy.array([[xform_cache.GetLocalToWorldTransform(child_prim).GetRow(i)[j] for i in range(3)] for j in range(3)])
        det_body_scale = numpy.linalg.det(body_scale_mat)
        if det_body_scale < 0:
            logging.warning(f"Body {child_prim.GetPath()} has negative scale, flipping the sign from {body_scale} to {-body_scale}.")
            body_scale = -body_scale

        body1_to_body2_pos = body1_to_body2_transform.ExtractTranslation()

        joint_quat = joint.GetLocalRot1Attr().Get()
        joint_quat = numpy.array([*joint_quat.GetImaginary(), joint_quat.GetReal()])
        body1_transform = xform_cache.GetLocalToWorldTransform(parent_prim)
        body1_rot = body1_transform.ExtractRotationQuat()
        joint_parent_prim = joint_prim.GetParent()
        joint_transform = get_relative(from_prim=parent_prim, to_prim=joint_parent_prim)
        joint_pos = numpy.array([*joint.GetLocalPos0Attr().Get()]) * body_scale

        joint_property = JointProperty(joint_parent_prim=parent_prim,
                                        joint_child_prim=child_prim,
                                        joint_pos=joint_pos,
                                        joint_quat=None,
                                        joint_axis=JointAxis.from_string(joint_axis),
                                        joint_type=joint_type)
        joint_builder = child_body_builder.add_joint(joint_name=joint_name,
                                                     joint_property=joint_property)
        if joint_prim.IsA(UsdPhysics.RevoluteJoint) or joint_prim.IsA(UsdPhysics.PrismaticJoint):
            joint_builder.joint.CreateUpperLimitAttr(joint.GetUpperLimitAttr().Get())
            joint_builder.joint.CreateLowerLimitAttr(joint.GetLowerLimitAttr().Get())

    @property
    def stage(self) -> Usd.Stage:
        return self._stage