# Multiverse-Parser：MJCF → USD 转换原理与流程（含与 lw_mjcf2usd 对比）

本文面向 `experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser` 的 MJCF→USD 转换实现，给出中文技术报告，并与 `experiments/usd2mjcf/lw_mjcf2usd` 的方案进行对比。重点聚焦“如何把 MJCF 导入并构建 USD 场景”，不展开 GUI。

## 流程总览

- 入口类：`MjcfImporter`（文件：`importer/mjcf_importer.py`）
- 外部依赖：`mujoco` Python（直接解析 MJCF → MjModel），`pxr`（写 USD，使用 UsdMujoco schema）
- 核心思路：不调用 Omniverse 的原生 MJCF 导入器，而是用 Mujoco SDK 解析 MJCF，再用自有的 Factory/Builder 层按 UsdMujoco 语义构建 USD 结构（/mujoco、/mujoco/asset/...）。
- 结果：输出一个包含 Mujoco 语义层（UsdMujoco）的 USD，包含 bodies、geoms、joints、materials、textures、meshes、composite/points 等。

关键调用链（文件与行号）：
- `importer/mjcf_importer.py:__init__` 载入 `mujoco.MjModel.from_xml_path(...)`，初始化配置与复合体（composite）索引。
- `importer/mjcf_importer.py:import_model` 主流程：构建 world、导入 body/geom/joint/points、执行命令、导入惯量、等式约束、导出。

## 关键模块与职责

- `Factory/WorldBuilder/BodyBuilder/GeomBuilder/JointBuilder/...`（`src/multiverse_parser/factory`）：
  - 提供抽象的构建命令收集与导出接口（如 `WorldBuilder.export()`）。
  - 对外暴露 `add_body`、`add_joint`、`add_geom`、`add_point`、`set_transform`、`set_inertial` 等能力。
- `UsdMujoco` Schema（pxr）：
  - 写入 `/mujoco` 顶层 prim 与 `/mujoco/asset/{meshes,materials,textures}`。
  - 附着 Mujoco 专有的 API（如 `MujocoBodyAPI`, `MujocoJointAPI`, `MujocoCompositeAPI` 等）。

## 转换步骤（MjcfImporter.import_model）

1) 初始化与配置
- 读取 MJCF：`self._mj_model = mujoco.MjModel.from_xml_path(...)`。
- `get_model_name()` 从 `<mujoco model="...">` 或文件名推断模型名。
- `_import_config()`：
  - `UsdMujoco.Mujoco.Define(stage, "/mujoco")`；
  - 写入 `MujocoOptionAPI`（如 `timestep`）；
  - 定义 `/mujoco/asset/meshes|materials|textures`。

2) 构建世界与根节点
- `world_builder.add_body(body_name=self.config.root_name)` 作为根 body。
- 遍历 `mj_model.body` 层次，尊重 `root_name`，为每个 body 构建 `BodyBuilder` 并设置相对/绝对变换：
  - 位姿：`pos=mj_body.pos`，`quat=[x,y,z,w]`（从 Mujoco 的 `[w,x,y,z]` 转为 pxr `Gf.Quatf(w,x,y,z)`）。
  - 写入 `MujocoBodyAPI` 的 `pos/quat` 属性。

3) 几何导入（_import_geoms）
- 几何类型映射（`_geom_type_map`）：plane/box/sphere/ellipsoid/cylinder/capsule/mesh。
- 原始几何（盒体、球体、圆柱、胶囊……）→ `GeomBuilder` + 相应 `GeomProperty`，设置 `size/pos/quat` 等；
- Mesh：
  - 从 `mj_model` 提取 mesh 顶点、法线、面索引等（`_get_mesh_data`）；
  - 纹理坐标：由 `MeshProperty.texture_coordinates` 管理（如存在）；
  - 将 mesh 写入临时目录（`tmp_mesh_dir_path`）并在 `/mujoco/asset/meshes` 下定义 `MujocoMesh`，通过 `CreateFileAttr("./<tmp_mesh_file>")` 关联；
  - 通过 `MujocoGeomAPI.CreateMeshRel()` 绑定 mesh。

4) 材质与纹理
- 若 `mat_id != -1`：
  - 解析 `mat_rgba/emission/specular`（`_get_material_data`），或根据 `mat_texid` 绑定纹理：
    - 在 `/mujoco/asset/materials` 定义 `MujocoMaterial` 并关联到 `geom`；
    - 在 `/mujoco/asset/textures` 定义 `MujocoTexture`（支持 `2d/cube/skybox`）；
    - `MujocoTexture.file` 设置为 `<texture_name>.png`（临时导出/转换链由 Factory 辅助）。

5) 关节导入（_import_joints / _import_joint）
- 将 `mujoco.mjtJoint` 映射到内部 `JointType`，构造 `JointProperty`：
  - `joint_parent_prim`/`joint_child_prim`；
  - `joint_pos/joint_axis/joint_quat`；
  - 关节 limit：
    - hinge：角度区间（转为度）；
    - slide：位移区间（米）。
- 应用 `UsdMujoco.MujocoJointAPI`，并记录 `_joint_builders` 供等式约束使用。

6) 复合体/点集（composite/points）
- 通过 `get_bodies_with_composite(xml)` 解析 XML 中 `<composite>` 节点（含 include 递归展开），索引挂在哪个 body 下；
- `_import_point()` 与 `_import_points()`：
  - 在 body 上创建 `PointsBuilder`；
  - `MujocoCompositeAPI` 设置 `type/count/spacing/offset/prefix`；
  - 根据 `geom` 的尺寸（如球直径）生成点宽度等视觉属性。

7) 执行构建与惯量
- `execute_cmds()` 提交累积的 builder 命令；
- 若开启物理：
  - `_import_inertial()`：从 Mujoco 模型计算/读取质心、对角惯量、主轴四元数，调用 `BodyBuilder.set_inertial(...)`；

8) 等式约束（equality）
- `_import_equality()`：
  - 仅处理 `mjEQ_JOINT` 且物理开启时；
  - 在 `/mujoco/equality` 下创建 `MujocoEqualityJoint`，设置两个关节的关系与 `polycoef`。

9) 导出
- `world_builder.export()` 写出 USD（默认写入临时路径 `tmp_usd_file_path`）；
- `import_model(save_file_path=...)` 时可将临时 USD 保存为最终路径。

## 支持与特性梳理

- 几何：plane/box/sphere/ellipsoid/cylinder/capsule/mesh；mesh 顶点/法线/索引/纹理坐标写入并参考到 `/mujoco/asset/meshes`；
- 材质：RGBA、emission、specular；可绑定纹理（2d/cube/skybox）；
- 关节：自由关节之外的 hinge/slide 等（含 limit）；写入 `MujocoJointAPI`；
- Body：相对/绝对变换、`MujocoBodyAPI.pos/quat`；
- Composite/Points：从 XML 解析 `<composite>`，并用 `MujocoCompositeAPI` 表达点集参数；
- 物理惯量：按配置来源（FROM_SRC/计算）设置质量、质心、惯量、主轴；
- 等式约束：`mjEQ_JOINT` 到 `MujocoEqualityJoint`。

## 与 lw_mjcf2usd 的差异对比

对比对象：
- Multiverse-Parser（本实现）：`src/multiverse_parser/importer/mjcf_importer.py`
- Lightwheel lw_mjcf2usd：`lightwheel/MJCF2USD/connection/mjcf2usd_utils.py`

差异点：
- 设计思路与依赖：
  - Multiverse：直接用 `mujoco` 解析 MJCF，自建 USD（UsdMujoco 语义）。
  - lw_mjcf2usd：调用 Omniverse 原生 `MJCFCreateAsset` 导入，然后做一系列“修复/增强”（材质重建、mesh 重名修复、关节属性补写、密度、引用变换下推等）。
- USD 语义层：
  - Multiverse：以 `/mujoco` 与 UsdMujoco API 为核心，完整保留/表达 MuJoCo 概念，USD 层是“带 Mujoco 语义的 USD”。
  - lw_mjcf2usd：以 Omniverse 的通用 USD/PhysX 语义为核心，导入后偏重物理/渲染在 Omniverse 中的可用性与一致性修复。
- XML 预处理：
  - Multiverse：基本不改原 XML（解析到 MjModel 后基于 SDK 结构工作）；支持 `<include>` 递归解析复合体。
  - lw_mjcf2usd：会修改并保存临时 XML（如 `mesh.refquat` 下放至 `geom`、展开 `<replicate>`、修 site 位置、未命名体/几何命名等）。
- 几何与复制：
  - Multiverse：支持 `<composite>`，在 USD 用 `MujocoCompositeAPI` 表达；不涉及通用 `<replicate>` 展开逻辑。
  - lw_mjcf2usd：无 UsdMujoco 语义，直接展开 `<replicate>` 并写入普通 USD 变换层级。
- 材质/纹理：
  - Multiverse：写入 `MujocoMaterial/Texture`，纹理文件名以 `.png` 形式挂在 `/mujoco/asset/textures`，材质属性遵循 Mujoco 语义。
  - lw_mjcf2usd：重建为 MDL OmniPBR 材质，复制纹理到输出旁 `texture/` 并绑定到 visuals。
- 关节与物理：
  - Multiverse：写 `MujocoJointAPI` 并在需要时设置 inertial；更加侧重 MuJoCo 语义表达。
  - lw_mjcf2usd：写 PhysX/UsdPhysics 关节与 MassAPI、Scene（MBP/TGS）、mesh 碰撞近似（none）；更贴近 Omniverse/PhysX 运行时。
- 引用/变换修复：
  - Multiverse：不依赖 Omniverse 的引用组合；直接构建目标层级，通常无需“中间引用变换丢失”修复。
  - lw_mjcf2usd：针对 Omniverse 导入后多重引用合成导致的变换丢失，提供 `fix_reference_missing_transform()` 等下推策略。
- 阶段产物与可移植性：
  - Multiverse：生成带 UsdMujoco 语义的 USD，更贴近 MJCF 概念，适合走“MJCF↔USD↔MJCF”的往返工作流。
  - lw_mjcf2usd：生成面向 Omniverse/PhysX 的通用 USD，适合直接进入 Isaac/Kit 场景和物理模拟。

## 适用建议

- 若目标是在 Omniverse/PhysX 中稳定运行、渲染/碰撞/关节与密度等即插即用：优先考虑 lw_mjcf2usd。
- 若目标是保留/表达更多 MuJoCo 原生语义（包括 composite、Mujoco 材质/纹理）并便于往返转换：优先考虑 Multiverse-Parser。

## 局限与注意事项

- Multiverse-Parser：
  - 依赖 `mujoco` Python 解析，需正确安装 MuJoCo 及其依赖；
  - 输出使用 UsdMujoco schema，非所有 DCC/引擎都理解其语义（需要消费方理解/转译）；
  - 纹理导出依赖内部临时文件与转换链（确保读写路径可用）。
- lw_mjcf2usd：
  - 强依赖 Omniverse 导入器；
  - 通过修改 XML、重建材质/密度/碰撞等“修复”来达成最终 USD，一些 MJCF 特有语义不作为一等公民（例如 composite）；
  - 某些高级材质/关节参数可能未完全映射。

## 关键文件引用

- `experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/importer/mjcf_importer.py`
  - MjcfImporter 主流程（`import_model`）、body/geom/joint/points/equality 导入；材质/纹理/mesh 写入；UsdMujoco 语义创建。
- `experiments/usd2mjcf/Multiverse-Parser/src/multiverse_parser/importer/usd_importer.py`
  - USD→内部结构的导入器（用于另一方向转换，本文不展开）。
- `experiments/usd2mjcf/lw_mjcf2usd/lightwheel/MJCF2USD/connection/mjcf2usd_utils.py`
  - 对比参考：Omniverse 原生导入 + 修复增强链路。

