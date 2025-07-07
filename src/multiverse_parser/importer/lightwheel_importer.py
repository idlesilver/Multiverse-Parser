#!/usr/bin/env python3.10

import os
from typing import Optional, Dict

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

from pxr import UsdPhysics, Usd, UsdGeom, UsdShade, Sdf, Gf

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

class LightwheelImporter(Factory):
    stage: Usd.Stage # type: ignore
    parent_map: Dict[Usd.Prim, Usd.Prim] # type: ignore

    def __init__(
            self,
            file_path: str,
            with_visual: bool,
            with_collision: bool,
            inertia_source: InertiaSource = InertiaSource.FROM_SRC,
            default_rgba: Optional[numpy.ndarray] = None
    ) -> None:
        self._stage = Usd.Stage.Open(file_path) # type: ignore

        default_prim = self.stage.GetDefaultPrim()
        model_name = default_prim.GetName()

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
                parent_prim = child_prim.GetParent()
            self.parent_map[child_prim] = parent_prim

        for prim in [prim for prim in self.stage.Traverse() if prim.IsA(UsdGeom.Xform) and prim.GetParent().IsA(UsdGeom.Xform)]: # type: ignore
            self._import_body(body_prim=prim)
        for parent_prim in self.parent_map.values():
            if parent_prim.GetName() in [body_builder.xform.GetPrim().GetName() for body_builder in self.world_builder.body_builders]:
                continue
            self._import_body(body_prim=parent_prim)
        for child_prim in self.parent_map.keys():
            if child_prim.GetName() in [body_builder.xform.GetPrim().GetName() for body_builder in self.world_builder.body_builders]:
                continue
            self._import_body(body_prim=child_prim)

        self.world_builder.export()

        if save_file_path is not None:
            self.save_tmp_model(usd_file_path=save_file_path)
        return self.tmp_usd_file_path if save_file_path is None else save_file_path
    
    def _import_body(self, body_prim: Usd.Prim) -> None: # type: ignore
        if body_prim != self.stage.GetDefaultPrim():
            logging.info(f"Importing body {body_prim.GetName()}...")
            if self.parent_map.get(body_prim) is None:
                parent_prim = body_prim.GetParent()
            else:
                parent_prim = self.parent_map[body_prim]
            parent_prim_name = parent_prim.GetName()
            body_builder = self.world_builder.add_body(body_name=body_prim.GetName(),
                                                       parent_body_name=parent_prim_name)
            xform_local_transformation, _ = xform_cache.ComputeRelativeTransform(body_prim,
                                                                                 parent_prim)
            body_builder.xform.ClearXformOpOrder()
            body_builder.xform.AddTransformOp().Set(xform_local_transformation)

            if body_prim.IsA(UsdGeom.Gprim):
                self._import_geom(gprim_prim=body_prim, body_builder=body_builder)

    def _import_geom(self, gprim_prim: Usd.Prim, body_builder: BodyBuilder) -> None:
        if not gprim_prim.IsA(UsdGeom.Mesh):
            logging.warning(f"Geometry {gprim_prim.GetName()} is not a mesh, skipping import.")
            return
        logging.info(f"Importing geometry {gprim_prim.GetName()}...")
        gprim = UsdGeom.Gprim(gprim_prim)
        geom_is_visible = gprim.GetVisibilityAttr().Get() != UsdGeom.Tokens.invisible
        geom_is_collidable = gprim_prim.HasAPI(UsdPhysics.CollisionAPI)
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
        transformation = gprim.GetLocalTransformation()
        geom_pos = transformation.ExtractTranslation()
        geom_pos = numpy.array([*geom_pos])
        geom_quat = transformation.ExtractRotationQuat()
        geom_quat = numpy.array([*geom_quat.GetImaginary(), geom_quat.GetReal()])
        geom_scale = numpy.array([transformation.GetRow(i).GetLength() for i in range(3)])
        mesh_file_path, mesh_path = get_usd_mesh_file_path(gprim_prim=gprim_prim)
        logging.info(f"Mesh file path: {mesh_file_path}, mesh path: {mesh_path}")

    @property
    def stage(self) -> Usd.Stage:
        return self._stage