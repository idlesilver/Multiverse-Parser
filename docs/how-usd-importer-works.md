usd_importer.py 概览与基础图元转换

- 作用：将 USD 场景解析为内部 World/Body/Geom 结构，并准备后续导出（MJCF/URDF）。核心类为 `UsdImporter`。

主要流程

- 初始化（构造函数）：
  - 打开并加载 Stage，取消实例化实例，扁平化场景图（Flatten）。
    - experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/importer/usd_importer.py:62
  - 定位根节点（默认 `defaultPrim` 或按 `root_name`），构建 `name_map`（为 Xform/Scope/Gprim/Mesh/Joint 生成唯一名）。
    - experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/importer/usd_importer.py:69
    - experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/importer/usd_importer.py:81
  - 构建 `parent_map`（从 Joint 的 body1 指向 body0；若 body0 缺失则取 body1 的父）。
    - experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/importer/usd_importer.py:102
  - 初始化 `geom_body_map`、`body_builders_with_inertial`，并保存配置。
    - experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/importer/usd_importer.py:116

- import_model：
  - 创建 `WorldBuilder`，确认根 Xform，并创建根 body，设置根位姿。
    - experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/importer/usd_importer.py:127
    - experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/importer/usd_importer.py:141
  - 遍历 `name_map`：先导入所有 `UsdGeom.Xform/Scope` 为 body（调用 `_import_body`）。
    - experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/importer/usd_importer.py:146
  - 再导入所有 `UsdGeom.Gprim` 为 geom（调用 `_import_geom`）。
    - experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/importer/usd_importer.py:151
  - 如果启用物理：
    - 收集需要惯性信息的 body（依据 `RigidBodyAPI` 在几何或父 body 上的设置）。
      - experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/importer/usd_importer.py:156
    - 导入关节（调用 `_import_joint`），过滤黑名单。
      - experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/importer/usd_importer.py:168
    - 为收集到的 body 设置惯性（从源 `MassAPI` 读取或按配置计算）。
      - experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/importer/usd_importer.py:175
  - 导出中间 USD 文件（供后续导出器使用）。
    - experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/importer/usd_importer.py:195

关键子流程

- _import_body：为 Xform/Scope 创建 body，确保父体先创建，设置父到子的相对变换；防止重复导入。
  - experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/importer/usd_importer.py:201

- _import_geom：
  - 仅处理 `UsdGeom.Mesh`；根据显示/用途与 `CollisionAPI` 筛选可见与可碰撞。
    - experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/importer/usd_importer.py:230
  - 刚体几何会被提为单独 body（若自身有 `RigidBodyAPI`），否则挂到父 body。
    - experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/importer/usd_importer.py:241
  - 类型映射：Cube/Sphere/Cylinder/Mesh → 内部 `GeomType`。
    - experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/importer/usd_importer.py:286
  - 颜色与不透明度来自 `displayColor/opactiy`，密度暂为 1000.0，创建 `GeomProperty` 并 `build()`；
    设置相对位姿与缩放，行列式为负时翻转缩放符号以规避镜像。
    - experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/importer/usd_importer.py:297
  - Mesh 数据拷贝：顶点/法线/索引/UV，写入本地 `meshes/usd/*.usda` 并建立引用；可见则导入材质（direct binding 或 subset）。
    - experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/importer/usd_importer.py:333

- _import_material：解析 direct/subset 材质绑定，从外部文件或 prim 构建材质属性，并建立本地引用与绑定。
  - experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/importer/usd_importer.py:374

- _import_joint：校验、消除缩放剪切，处理负缩放，
  将标在 Gprim 上的 body 解析到真正的 body xform，计算关节局部位姿，
  映射关节类型（Revolute/Prismatic/Continuous），记录上下限与惯性收集。
  - experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/importer/usd_importer.py:437

基础图元（Box/Sphere/Cylinder）如何转换

- 类型判定（Importer 阶段）：
  - Cube → `GeomType.CUBE`；Sphere → `GeomType.SPHERE`；Cylinder → `GeomType.CYLINDER`；Mesh → `GeomType.MESH`。
  - 参见：experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/importer/usd_importer.py:286

- 几何尺寸与 MJCF 类型（Exporter 阶段）：
  - Box：
    - 若存在 `UsdUrdf.UrdfGeometryBoxAPI.size`，取其一半作为 `size`（MJCF 以半尺寸表示）。
    - 否则从局部变换的尺度向量读取；若行列式为负则取相反数以消除镜像。
    - 类型写入 `MujocoGeomAPI.type = "box"`。
    - 参见：experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/exporter/mjcf_exporter.py:124
  - Sphere：
    - `size = [radius, 0, 0]`，type 为 `"sphere"`。
    - 参见：experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/exporter/mjcf_exporter.py:146
  - Cylinder：
    - `size = [radius, height/2, 0]`，type 为 `"cylinder"`（MJCF 使用半高）。
    - 参见：experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/exporter/mjcf_exporter.py:150
  - Capsule（如使用）：同 Cylinder 的 size 生成方式，type 为 `"capsule"`。
    - 参见：experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/exporter/mjcf_exporter.py:156
  - Mesh：来自 `UrdfGeometryMeshAPI.scale` 或局部变换尺度，type 为 `"mesh"`；并关联 `/mujoco/asset/meshes/<name>`。
    - 参见：experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/exporter/mjcf_exporter.py:161

- 位姿与朝向：
  - 采用去掉缩放/剪切后的局部变换，写入 `MujocoGeomAPI.pos/quat`。
  - 参见：experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/exporter/mjcf_exporter.py:132

- 碰撞近似：
  - Mesh 碰撞近似由 `UsdPhysics.MeshCollisionAPI.approximation` 控制；Importer 在 `geom_builder.build(...)` 时写出该属性。
  - 参见：experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/importer/usd_importer.py:312

备注

- 默认密度为 1000.0（WIP：未来可从 USD 读取材质/物理属性）。
- 若几何或关节存在负缩放，代码会翻转符号以规避镜像导致的几何/惯性异常。
- 刚体几何可能被提升为独立 body；惯性可直接从源 `MassAPI` 读取或由几何计算（见 `InertiaSource` 配置）。

