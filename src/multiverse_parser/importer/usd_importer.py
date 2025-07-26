#!/usr/bin/env python3.10

import os
from typing import List, Optional, Dict

import numpy

from multiverse_parser import logging
from pxr import UsdPhysics, Usd, UsdGeom, Sdf, Gf, UsdShade
from ..factory import Factory, Configuration, InertiaSource
from ..factory import (WorldBuilder,
                       BodyBuilder,
                       JointAxis, JointType, JointProperty,
                       GeomBuilder, GeomType, GeomProperty,
                       MeshProperty,
                       MaterialProperty)
from ..utils import xform_cache, get_relative_transform, validate_joint_prim, extract_transform


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


class UsdImporter(Factory):
    stage: Usd.Stage  # type: ignore
    parent_map: Dict[Usd.Prim, Usd.Prim]  # type: ignore
    name_map: Dict[Sdf.Path, str]  # type: ignore
    geom_body_map: Dict[Usd.Prim, Usd.Prim]  # type: ignore
    body_builders_with_inertial: Dict[BodyBuilder, Usd.Prim]  # type: ignore

    def __init__(
            self,
            file_path: str,
            fixed_base: bool,
            with_physics: bool,
            with_visual: bool,
            with_collision: bool,
            root_name: Optional[str] = None,
            inertia_source: InertiaSource = InertiaSource.FROM_SRC,
            default_rgba: Optional[numpy.ndarray] = None,
            exclude_names: Optional[List[str]] = None,
    ) -> None:
        self.black_list_names = exclude_names
        logging.info(f"Importing USD file: {file_path} with root name: {root_name}, "
                     f"fixed base: {fixed_base}, with_physics: {with_physics}, "
                     f"with_visual: {with_visual}, with_collision: {with_collision}, "
                     f"inertia_source: {inertia_source}, default_rgba: {default_rgba}, "
                     f"exclude_names: {exclude_names}")
        self._stage = Usd.Stage.Open(file_path)  # type: ignore
        self.stage.Load()
        for prim in self.stage.TraverseAll():
            if prim.IsInstanceable():
                prim.SetInstanceable(False)
        self.stage.Flatten()

        if root_name is not None:
            self.name_map = {}
            for prim in self.stage.Traverse():
                if prim.IsA(UsdGeom.Xform) and prim.GetName() == root_name:
                    root_prim = prim
                    break
            else:
                raise NotImplementedError(f"Root prim {root_name} not found.")
        else:
            root_prim = self.stage.GetDefaultPrim()
            root_name = root_prim.GetName()
            self.name_map = {root_prim.GetPath(): root_name}
        for prim in self.stage.Traverse():
            assert prim.IsValid(), f"Invalid prim found in stage: {prim.GetPath()}"
            if any(black_list_name in str(prim.GetPath()) for black_list_name in
                   exclude_names) if exclude_names is not None else False:
                continue
            if not prim.GetParent().IsA(UsdGeom.Xform) and not prim.IsA(UsdPhysics.Joint) and not prim.GetParent().IsA(UsdGeom.Scope): # type: ignore
                continue
            parent_prim = prim
            while not parent_prim.IsPseudoRoot():
                if parent_prim == root_prim:
                    break
                parent_prim = parent_prim.GetParent()
            if parent_prim.IsPseudoRoot():
                continue
            if prim.IsA(UsdPhysics.Joint) or prim.IsA(UsdGeom.Xform) or prim.IsA(UsdGeom.Gprim) or prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Scope): # type: ignore
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
        self.body_builders_with_inertial = {}
        self.geom_body_map = {}

        super().__init__(file_path=file_path, config=Configuration(
            model_name=root_name,
            fixed_base=True,
            root_name=root_name,
            with_physics=with_physics,
            with_visual=with_visual,
            with_collision=with_collision,
            default_rgba=default_rgba if default_rgba is not None else numpy.array([0.9, 0.9, 0.9, 1.0]),
            inertia_source=inertia_source
        ))

    def import_model(self, save_file_path: Optional[str] = None) -> str:
        self._world_builder = WorldBuilder(self.tmp_usd_file_path)

        if self.config.root_name is not None:
            for prim in self.stage.Traverse():
                if prim.GetName() == self.config.root_name:
                    root_prim = prim
                    break
            else:
                raise NotImplementedError(f"Root prim {self.config.root_name} not found.")
        else:
            root_prim = self.stage.GetDefaultPrim()
            self.config.root_name = root_prim.GetName()

        assert root_prim.IsA(UsdGeom.Xform), f"Root prim {root_prim.GetPath()} is not a UsdGeom.Xform, cannot import model."
        root_pos, root_quat, _ = extract_transform(root_prim)
        root_body_builder = self.world_builder.add_body(root_prim.GetName())
        root_body_builder.set_transform(pos=root_pos, quat=root_quat)

        for prim_path in self.name_map.keys():
            prim = self.stage.GetPrimAtPath(prim_path)
            if prim.IsA(UsdGeom.Xform) or prim.IsA(UsdGeom.Scope): # type: ignore
                self._import_body(body_prim=prim)

        for prim_path in list(self.name_map.keys()):
            prim = self.stage.GetPrimAtPath(prim_path)
            if prim.IsA(UsdGeom.Gprim): # type: ignore
                self._import_geom(gprim_prim=prim)

        if self.config.with_physics:
            for gprim_prim in [geom_prim for geom_prim in self.stage.Traverse() if geom_prim.IsA(UsdGeom.Gprim)]:  # type: ignore
                if gprim_prim.GetParent().GetParent().IsPseudoRoot():
                    continue
                body_prim = gprim_prim.GetParent()
                geom_can_move = gprim_prim.HasAPI(UsdPhysics.RigidBodyAPI) and UsdPhysics.RigidBodyAPI(gprim_prim).GetRigidBodyEnabledAttr().Get() and not UsdPhysics.RigidBodyAPI(gprim_prim).GetKinematicEnabledAttr().Get() # type: ignore
                body_can_move = body_prim.HasAPI(UsdPhysics.RigidBodyAPI) and UsdPhysics.RigidBodyAPI(body_prim).GetRigidBodyEnabledAttr().Get() and not UsdPhysics.RigidBodyAPI(body_prim).GetKinematicEnabledAttr().Get() # type: ignore
                assert not (geom_can_move and body_can_move), \
                    f"Geom prim {gprim_prim.GetPath()} its parent have the same RigidBodyAPI state, either both should have it or neither should have it."
                if (geom_can_move or body_can_move) and gprim_prim in self.geom_body_map:
                    self.body_builders_with_inertial[self.world_builder.get_body_builder(body_name=self.geom_body_map[gprim_prim].GetName())] = gprim_prim if geom_can_move else body_prim

            for joint_prim in [joint_prim for joint_prim in self.stage.Traverse() if
                               joint_prim.IsA(UsdPhysics.Joint)]:  # type: ignore
                if any(black_list_name in str(joint_prim.GetPath()) for black_list_name in
                       self.black_list_names) if self.black_list_names is not None else False:
                    continue
                self._import_joint(joint_prim=joint_prim)

            for body_builder, prim in self.body_builders_with_inertial.items():
                if self._config.inertia_source == InertiaSource.FROM_SRC:
                    body_prim_api = UsdPhysics.MassAPI(prim)  # type: ignore
                    body_mass = body_prim_api.GetMassAttr().Get()
                    body_center_of_mass = body_prim_api.GetCenterOfMassAttr().Get()
                    body_center_of_mass = numpy.array([*body_center_of_mass]) \
                        if body_center_of_mass is not None else numpy.zeros(3)
                    body_diagonal_inertia = body_prim_api.GetDiagonalInertiaAttr().Get()
                    body_diagonal_inertia = numpy.array([*body_diagonal_inertia]) \
                        if body_diagonal_inertia is not None else numpy.zeros(3)
                    body_principal_axes = body_prim_api.GetPrincipalAxesAttr().Get()
                    body_principal_axes = numpy.array([*body_principal_axes.GetImaginary(), body_principal_axes.GetReal()]) \
                        if body_principal_axes is not None else numpy.array([0.0, 0.0, 0.0, 1.0])
                    body_builder.set_inertial(mass=body_mass,
                                              center_of_mass=body_center_of_mass,
                                              diagonal_inertia=body_diagonal_inertia,
                                              principal_axes=body_principal_axes)
                else:
                    body_builder.compute_and_set_inertial(inertia_source=self.config.inertia_source)

        self.world_builder.export()

        if save_file_path is not None:
            self.save_tmp_model(usd_file_path=save_file_path)
        return self.tmp_usd_file_path if save_file_path is None else save_file_path

    def _import_body(self, body_prim: Usd.Prim) -> None:  # type: ignore
        if self.name_map[body_prim.GetPath()] != self.config.root_name:
            imported_body_names = [body_builder.xform.GetPrim().GetName() for body_builder in
                                   self.world_builder.body_builders]
            body_name = self.name_map[body_prim.GetPath()]
            assert body_name not in imported_body_names, f"Body {body_name} already imported."
            parent_prim = self.parent_map.get(body_prim, body_prim.GetParent())
            if parent_prim.GetPath() not in self.name_map:
                return
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

        gprim = UsdGeom.Gprim(gprim_prim)  # type: ignore
        geom_name = self.name_map[gprim_prim.GetPath()]
        geom_is_visible = gprim.GetVisibilityAttr().Get() == UsdGeom.Tokens.inherited and gprim.GetPurposeAttr().Get() != UsdGeom.Tokens.guide  # type: ignore
        geom_is_collidable = gprim_prim.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(  # type: ignore
            gprim_prim).GetCollisionEnabledAttr().Get()  # type: ignore
        if not self.config.with_visual and geom_is_visible or not self.config.with_collision and geom_is_collidable:
            return
        geom_can_move = gprim_prim.HasAPI(UsdPhysics.RigidBodyAPI) and UsdPhysics.RigidBodyAPI(gprim_prim).GetRigidBodyEnabledAttr().Get() # type: ignore
        if geom_can_move: # type: ignore
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

        self.geom_body_map[gprim_prim] = body_builder.xform.GetPrim()

        if geom_name not in body_builder._geom_builders:
            logging.info(f"Importing geometry: {gprim_prim.GetPath()} as {geom_name}...")
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

            geom_rgba = self.config.default_rgba.tolist()
            gprim_rgb = gprim.GetDisplayColorAttr().Get()
            gprim_opacity = gprim.GetDisplayOpacityAttr().Get()
            if gprim_rgb is not None and len(gprim_rgb[0]) > 0:
                geom_rgba[:3] = gprim_rgb[0][:3]
            if gprim_opacity is not None:
                geom_rgba[3] = gprim_opacity[0]
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
            geom_builder.gprim.CreatePurposeAttr(gprim.GetPurposeAttr().Get()) # type: ignore
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

        for primvar in UsdGeom.PrimvarsAPI(gprim_prim).GetPrimvars(): # type: ignore
            if primvar.GetBaseName() == "st" and primvar.GetTypeName().cppTypeName == "VtArray<GfVec2f>":
                texture_coordinates = numpy.array(primvar.Get(), dtype=numpy.float32)
                texture_coordinate_indices = primvar.GetIndicesAttr().Get()
                break
        else:
            texture_coordinates = None
            texture_coordinate_indices = None
        mesh_property = MeshProperty(points=numpy.array(points),
                                     normals=numpy.array(normals),
                                     face_vertex_counts=numpy.array(face_vertex_counts),
                                     face_vertex_indices=numpy.array(face_vertex_indices),
                                     texture_coordinates=texture_coordinates,
                                     texture_coordinate_indices=texture_coordinate_indices)
        mesh_property.mesh_file_name = mesh_name
        if mesh_property.points.size == 0 or mesh_property.face_vertex_counts.size == 0 or mesh_property.face_vertex_indices.size == 0:
            # TODO: Fix empty mesh
            logging.warning(f"Mesh {gprim_prim.GetName()} has no points or face vertex counts, skipping import.")
            return
        geom_builder.add_mesh(mesh_name=mesh_name,
                              mesh_property=mesh_property)

        if geom_is_visible:
            self._import_material(geom_builder=geom_builder, gprim_prim=gprim_prim)

    def _import_material(self, geom_builder: GeomBuilder, gprim_prim: Usd.Prim) -> None:  # type: ignore
        material_id = 0
        if gprim_prim.HasAPI(UsdShade.MaterialBindingAPI): # type: ignore
            material_prim = self._get_material_prim(gprim_prim=gprim_prim)
            if material_prim is not None:
                material_path, material_property = self._get_material_property(material_prim=material_prim)
                material_name = f"M_{gprim_prim.GetName()}_{material_id}"
                material_id += 1
                logging.info(f"Importing material: {material_path} as {material_name}...")
                geom_builder.add_material(material_name=material_name,
                                          material_property=material_property)

        material_paths = []
        for subset_prim in [subset_prim for subset_prim in gprim_prim.GetChildren() if
                            subset_prim.IsA(UsdGeom.Subset) and subset_prim.HasAPI(UsdShade.MaterialBindingAPI)]: # type: ignore
            material_prim = self._get_material_prim(gprim_prim=subset_prim)
            if material_prim is None:
                continue
            material_path, material_property = self._get_material_property(material_prim)
            material_name = f"M_{gprim_prim.GetName()}_{material_id}"
            if material_path not in material_paths:
                material_id += 1
                logging.info(f"Importing material: {material_path} as {material_name}...")
                material_paths.append(material_path)

            geom_builder.add_material(material_name=material_name,
                                      material_property=material_property,
                                      subset=UsdGeom.Subset(subset_prim)) # type: ignore


    def _get_material_prim(self, gprim_prim: Usd.Prim) -> Usd.Prim: # type: ignore
        material_binding_api = UsdShade.MaterialBindingAPI(gprim_prim) # type: ignore
        material_paths = material_binding_api.GetDirectBindingRel().GetTargets()
        if len(material_paths) > 1:
            raise NotImplementedError(f"{gprim_prim.GetTypeName()} {gprim_prim.GetName()} has more than one material.")
        if len(material_paths) == 0:
            return None
        else:
            material_prim = self.stage.GetPrimAtPath(material_paths[0])
            if not material_prim.IsValid():
                return None
        return material_prim


    def _get_material_property(self, material_prim: Usd.Prim) -> (Sdf.Path, MaterialProperty): # type: ignore
        if len(material_prim.GetPrimStack()) >= 2:
            material_prim_stack = material_prim.GetPrimStack()[1]
            material_file_path = material_prim_stack.layer.realPath
            material_path = material_prim_stack.path
        else:
            material_file_path = material_prim.GetStage().GetRootLayer().realPath
            material_path = material_prim.GetPath()
        if material_file_path != self.stage.GetRootLayer().realPath:
            material_property = MaterialProperty.from_material_file_path(
                material_file_path=material_file_path,
                material_path=material_path)
        else:
            material_property = MaterialProperty.from_prim(material_prim=material_prim)
        if material_property.opacity == 0.0:
            logging.warning(f"Opacity of {material_path} is 0.0. Set to 1.0.")
            material_property._opacity = 1.0
        return material_path, material_property

    def _import_joint(self, joint_prim: Usd.Prim) -> None:  # type: ignore
        joint = UsdPhysics.Joint(joint_prim)  # type: ignore
        child_prim_path = joint.GetBody1Rel().GetTargets()[0]
        child_prim = self.stage.GetPrimAtPath(child_prim_path)
        if child_prim.IsA(UsdGeom.Gprim) and child_prim not in self.geom_body_map:
            return
        if not joint_prim.IsA(UsdPhysics.RevoluteJoint) and not joint_prim.IsA(UsdPhysics.PrismaticJoint): # type: ignore
            prim = self.stage.GetPrimAtPath(UsdPhysics.Joint(joint_prim).GetBody1Rel().GetTargets()[0])  # type: ignore
            if prim.IsA(UsdGeom.Xform):  # type: ignore
                body_prim_name = self.name_map[prim.GetPath()]
                body_builder = self.world_builder.get_body_builder(body_name=body_prim_name)
                if body_builder in self.body_builders_with_inertial:
                    del self.body_builders_with_inertial[body_builder]
                for child_gprim_prim in prim.GetAllChildren():
                    body = self.geom_body_map.get(child_gprim_prim, None)
                    if body is None:
                        continue
                    body_builder = self.world_builder.get_body_builder(body_name=body.GetName())
                    if body_builder in self.body_builders_with_inertial:
                        del self.body_builders_with_inertial[body_builder]
            elif prim.IsA(UsdGeom.Gprim):  # type: ignore
                body = self.geom_body_map.get(prim, None)
                body_builder = self.world_builder.get_body_builder(body_name=body.GetName())
                if body_builder in self.body_builders_with_inertial:
                    del self.body_builders_with_inertial[body_builder]
            return

        logging.info(f"Importing joint: {joint_prim.GetPath()}...")

        validate_joint_prim(joint_prim)

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
            child_body_name = child_prim.GetName()
        else:
            child_to_joint_pos = numpy.array([0.0, 0.0, 0.0])
            child_body_name = self.name_map[child_prim.GetPath()]
        joint_name = self.name_map[joint_prim.GetPath()]
        child_body_builder = self.world_builder.get_body_builder(body_name=child_body_name)
        child_prim = child_body_builder.xform.GetPrim()
        parent_prim = child_prim.GetParent()

        joint_quat = joint.GetLocalRot1Attr().Get()
        joint_quat = numpy.array([*joint_quat.GetImaginary(), joint_quat.GetReal()])
        if joint_prim.IsA(UsdPhysics.RevoluteJoint): # type: ignore
            joint = UsdPhysics.RevoluteJoint(joint) # type: ignore
            joint_axis = joint.GetAxisAttr().Get()
            if numpy.isfinite(joint.GetLowerLimitAttr().Get()) and numpy.isfinite(joint.GetUpperLimitAttr().Get()):
                joint_type = JointType.REVOLUTE
            else:
                joint_type = JointType.CONTINUOUS
        elif joint_prim.IsA(UsdPhysics.PrismaticJoint): # type: ignore
            joint = UsdPhysics.PrismaticJoint(joint) # type: ignore
            joint_axis = joint.GetAxisAttr().Get()
            joint_type = JointType.PRISMATIC
        else:
            raise ValueError(f"Joint type {joint_prim} not supported.")

        joint_property = JointProperty(joint_parent_prim=parent_prim,
                                       joint_child_prim=child_prim,
                                       joint_pos=child_to_joint_pos,
                                       joint_quat=joint_quat,
                                       joint_axis=JointAxis.from_string(joint_axis),
                                       joint_type=joint_type)
        joint_builder = child_body_builder.add_joint(joint_name=joint_name, joint_property=joint_property)
        if joint_prim.IsA(UsdPhysics.RevoluteJoint) or joint_prim.IsA(UsdPhysics.PrismaticJoint): # type: ignore
            joint_builder.joint.CreateUpperLimitAttr(joint.GetUpperLimitAttr().Get())
            joint_builder.joint.CreateLowerLimitAttr(joint.GetLowerLimitAttr().Get())

        if child_body_builder not in self.body_builders_with_inertial: # type: ignore
            prim_with_inertial = self.stage.GetPrimAtPath(child_prim_path)
            if not prim_with_inertial.HasAPI(UsdPhysics.MassAPI) and prim_with_inertial.IsA(UsdGeom.Gprim): # type: ignore
                prim_with_inertial = prim_with_inertial.GetParent()
                if not prim_with_inertial.HasAPI(UsdPhysics.MassAPI): # type: ignore
                    logging.warning(f"Prim {prim_with_inertial.GetPath()} does not have a MassAPI.")
            self.body_builders_with_inertial[child_body_builder] = prim_with_inertial

    @property
    def stage(self) -> Usd.Stage:  # type: ignore
        return self._stage
