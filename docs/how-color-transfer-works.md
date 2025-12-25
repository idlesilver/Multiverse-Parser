# MJCF 颜色与纹理来源说明（Multiverse-Parser）

## 颜色来源（USD importer）
- 入口在 `third_party/Multiverse-Parser/src/multiverse_parser/importer/usd_importer.py` 的 `_import_geom`。
- `geom_rgba` 先用配置默认值 `default_rgba`，默认是 `[0.9, 0.9, 0.9, 1.0]`。
- 若 USD 的 `UsdGeom.Gprim` 提供 `displayColor` 与 `displayOpacity`，会覆盖 `geom_rgba`。
- 只有可见几何体才会进入 `_import_material`，并将材质绑定写入 USD 中。
- 纹理坐标只从 `primvars:st`（`VtArray<GfVec2f>`）读取并传入 mesh 属性。

## 颜色来源（MJCF exporter）
- 入口在 `third_party/Multiverse-Parser/src/multiverse_parser/exporter/mjcf_exporter.py` 的 `_build_geom`。
- 若几何体绑定了 MujocoMaterial，则 MJCF `<geom>` 仅引用 `material`，颜色/纹理来自 `<asset>` 中的 `<material>/<texture>`。
- 若没有材质且该几何体不是碰撞几何体，则直接写入 `geom_builder.rgba` 到 MJCF `<geom rgba=...>`。
- 碰撞几何体使用默认 class `*_collision`，其 `rgba` 在 `_build_config` 中由 `factory.config.default_rgba` 统一设置。

## 纹理与材质导出逻辑（MJCF exporter）
- `_build_mujoco_asset_mesh_and_material_prims` 会遍历 `UsdShade.Material` 并创建 MujocoMaterial；若材质包含纹理，会创建 MujocoTexture 并写入 `<asset>`。
- `get_mujoco_geom_api` 中：
  - 有 `MaterialBindingAPI` 时，直接绑定对应 MujocoMaterial。
  - 没有直接材质但有 `Subset + MaterialBindingAPI` 时：
    - `merge_textures=True` 会调用 `merge_texture`，生成 `tmp/textures/T_*.png`，并创建一个合并后的材质。
    - `merge_textures=False` 只导出第一个子材质。

## 如何尽量保持与 USD 一致的纹理材质
1. 在 USD 中为 `Gprim` 或 `Geom Subset` 显式绑定 `UsdShade.Material`，不要只依赖 `displayColor`。
2. 确保 mesh 有 `primvars:st` 且类型为 `VtArray<GfVec2f>`，否则纹理坐标无法带入。
3. 如果一个 mesh 有多个子材质，保持 `merge_textures=True`（默认）以烘焙为单张纹理，避免只导出第一个子材质导致材质丢失。
4. 若目标是纯色材质，保证 USD 的 `displayColor/displayOpacity` 明确设置；若无材质绑定，MJCF 将回退为这些值。
5. 碰撞几何体的颜色统一走 `default_rgba`；若希望与视觉材质一致，避免将可见几何体标记为碰撞用途或单独给视觉几何体材质。
