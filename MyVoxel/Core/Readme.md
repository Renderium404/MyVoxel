# MyVoxel Core

`MyVoxel/Core` 是 MyVoxel 的离散体素数据核心层。

核心层负责定义体素空间、逻辑树结构、稀疏多根组织、物理节点存储、受控修改会话以及修改影响记录。它只提供稳定的数据结构和修改原语，不承担体素化、布尔运算、切削仿真、表面重建、历史管理等上层策略。

---

## 1. 核心层定位

核心层回答以下问题：

- 一个体素地址如何表示，父子地址如何转换；
- 连续空间如何映射到体素网格；
- 一个第 0 层根体素如何保存为稀疏树；
- 多个根体素树如何组成一个完整体素体；
- 逻辑体素状态如何映射到紧凑物理存储；
- 体素体如何复制、共享并在修改时分离；
- 一次连续修改如何被统一执行；
- 一次修改影响了哪些根和哪些材料区域。

核心层不回答以下问题：

- 几何体如何被体素化；
- 两个体素体如何执行布尔运算；
- 磨削轨迹何时、以何种粒度修改体素；
- 何时进行局部合并或全量合并；
- 如何生成、缓存或增量更新三角网格；
- 如何实现撤销、重做、历史版本或持久化格式；
- 如何调度线程、任务和算法阶段。

这些能力必须建立在核心层之上，而不能反向进入核心数据结构。

---

## 2. 核心设计原则

### 2.1 结构具有正式语义

逻辑体素只有三种状态：

```text
Empty
Material
Subdivided
```

其中 `Subdivided` 不是临时实现状态，而是正式结构状态。

以下两棵树具有相同的材料分布，但结构不同：

```text
Material
```

```text
Subdivided
├─ Material
├─ Material
├─ Material
├─ Material
├─ Material
├─ Material
├─ Material
└─ Material
```

因此：

- `split()` 是正式结构修改；
- `merge()` 是正式结构修改；
- `hasNode()` 可以观察显式逻辑节点；
- 未完全合并的树仍然是合法树；
- 序列化、历史或版本系统可以选择保留显式结构。

### 2.2 核心层不自动合并

底层修改只执行调用者明确请求的原语：

```text
setState(address)
split(address)
merge(address)
```

核心层不会因为子节点状态一致而自动向上合并，也不会在 Session 析构时自动整理结构。

合并时机属于算法或更高层系统。例如：

```text
连续切削期间
    保留展开结构

一段轨迹完成后
    合并受影响区域

最终保存或压缩前
    全量合并
```

核心层只提供合并原语，不提供隐式策略。

### 2.3 逻辑结构与物理存储分离

逻辑层可以观察：

```text
Empty
Material
Subdivided
显式节点是否存在
```

物理层内部可以使用：

```text
VoxelNodeBlock
VoxelLeafBlock
Branch
MaskLeaf
VoxelBlockPool
八槽组
Chunk
```

物理压缩形式不是 Shape 的正式语义。

只要逻辑地址、逻辑状态和显式节点关系保持一致，内部可以调整节点块、掩码叶块和节点池实现，而不影响上层算法。

### 2.4 修改原语与算法策略分离

`VoxelTreeEditor`、`VoxelForest` 和 `VoxelShapeSession` 只提供明确的修改原语。

以下策略不属于核心层：

- 修改顺序；
- 合并时机；
- 修改区域选择；
- 并行划分；
- 表面更新时机；
- 历史版本创建时机；
- 是否保留局部细分结构。

### 2.5 Shape 只有一个受控写入口

`VoxelShape` 是值对象和只读查询入口。

对 Shape 共享体素数据的修改必须通过：

```cpp
VoxelShapeSession session =
    shape.session(changeTrackingLevel);
```

上层代码不得绕过 Session 直接修改 `VoxelForest` 或 `VoxelTree`。

`VoxelForest` 和 `VoxelTreeEditor` 的写接口是核心内部原语，不是 Shape 级公共修改入口。

---

## 3. 分层结构

```text
VoxelTypes
    基础索引、层级、角点和状态定义
        │
        ├── VoxelAddress
        │       离散地址拓扑
        │
        ├── VoxelGrid
        │       连续空间与离散地址映射
        │
        └── Storage / Mask
                物理节点布局、节点池和位运算
                    │
                    ▼
VoxelTree
    一个第0层根体素树
        │
        ├── VoxelTreeCursor
        │       局部只读遍历
        │
        ├── VoxelTreeEditor
        │       局部修改原语
        │
        └── VoxelTreeAccessor
                单树地址缓存查询
                    │
                    ▼
VoxelForest
    根索引到VoxelTree的稀疏映射
                    │
                    ▼
VoxelShape
    Grid + Forest + Transform 的值对象
                    │
                    ▼
VoxelShapeSession
    Shape唯一受控修改入口
                    │
                    ▼
VoxelChangeSet
    一次会话产生的只读变化范围
```

依赖必须保持自下而上。底层存储、地址和树结构不能依赖建模、布尔、仿真或表面模块。

---

## 4. 数据结构职责

## 4.1 `VoxelTypes`

### 负责

- 定义 `VoxelIndex`、`VoxelLevel` 等基础类型；
- 定义 `VoxelCorner`；
- 定义逻辑状态 `VoxelState`；
- 定义物理节点状态 `VoxelNodeState`；
- 提供核心层统一使用的基础常量。

### 不负责

- 地址父子关系；
- 空间坐标转换；
- 节点存储分配；
- 树访问和修改。

---

## 4.2 `VoxelAddress`

### 定位

`VoxelAddress` 负责离散体素地址的拓扑关系。

### 负责

- `VoxelCellIndex + VoxelLevel` 的地址表示；
- 父地址和子地址转换；
- 当前体素在父体素中的角点；
- 祖先地址计算；
- 第 0 层根地址计算；
- 构造祖先到后代的角点路径；
- 正确处理负索引地址。

### 不负责

- 世界坐标；
- 体素尺寸；
- 树中是否真实存在节点；
- 访问或修改体素状态。

地址关系必须集中在 `VoxelAddress`，其他结构不得重复实现根地址或角点路径算法。

---

## 4.3 `VoxelGrid`

### 定位

`VoxelGrid` 是连续局部空间与离散体素地址之间的固定映射。

### 负责

- 网格原点；
- 第 0 层体素边长；
- 最高允许层级；
- 各层体素尺寸；
- 点、索引和包围盒之间的空间转换；
- 判断地址层级是否受当前网格支持。

### 不负责

- 保存材料；
- 保存树结构；
- 实例变换；
- 修改策略；
- 世界空间中的任意仿射变换。

`VoxelGrid` 描述 Shape 的局部离散空间。`VoxelShape::transform()` 负责局部空间到外部空间的实例变换。

---

## 4.4 `VoxelBlock`

### 定位

`VoxelBlock` 定义体素树的物理存储单元。

### 负责

- `VoxelNodeBlock` 的紧凑节点描述；
- `VoxelLeafBlock` 的两层、`4 × 4 × 4` 材料位图；
- 节点块与叶块的联合物理槽；
- 重置和底层状态读取所需的固定布局。

### 不负责

- 逻辑地址；
- 物理内存分配；
- COW；
- 节点生命周期策略；
- 上层树结构语义。

物理块不得直接暴露给 Shape、算法或业务层。

---

## 4.5 `VoxelBlockPool`

### 定位

`VoxelBlockPool` 是单棵 `VoxelTree` 使用的稳定地址物理节点池。

### 负责

- 按八槽组分配和回收节点；
- 分配对齐且地址稳定的 Chunk；
- 初始化普通节点块和掩码叶块；
- 检查组和槽是否有效；
- 记录已分配组数量、高水位和存储容量；
- `rewind()` 后复用已经持有的 Chunk。

### 不负责

- 逻辑树遍历；
- 判断节点是否应该细分或合并；
- 跨树共享策略；
- 跨根管理。

节点池只管理物理资源，不决定逻辑结构。

---

## 4.6 `VoxelNodeMask`

### 定位

`VoxelNodeMask` 是 `VoxelNodeBlock` 的内部八位编码、解码和位修改工具。

### 负责

- 从 `storageMask` 和 `leafMask` 解码四种物理状态；
- 批量计算 Empty、Material、Branch 和 MaskLeaf 位置；
- 判断普通节点块是否可折叠；
- 统计和遍历八位掩码；
- 修改状态位。

### 不负责

- 创建或释放物理子节点；
- 逻辑地址；
- 自动合并；
- Shape 级修改。

修改 `Branch` 或 `MaskLeaf` 状态位前后，调用者必须正确管理对应物理存储。

---

## 4.7 `VoxelLeafMask`

### 定位

`VoxelLeafMask` 是 `VoxelLeafBlock` 的内部 64 位材料编码和批量位运算工具。

### 负责

- `4 × 4 × 4` 细层体素与 64 位索引的双向映射；
- 八个粗层组与各自八个细层体素的映射；
- 整体、粗层组和细层体素状态判断；
- 批量归约八个粗层组状态；
- 位展开、位选择、置位计数和位遍历；
- 材料位设置、反转和批量修改。

### 不负责

- Shape 的最高层级；
- 叶块应当出现在树的哪个层级；
- 物理叶块的分配和释放；
- 自动折叠。

“粗层”和“细层”是叶块内部的相对层级，不等同于 Shape 的固定最高层级。

---

## 4.8 `VoxelChildStateMasks`

### 定位

`VoxelChildStateMasks` 是面向逻辑树的八子体素批量状态结果。

### 负责

- 表达八个直接子体素的 Empty、Material 和 Subdivided 位置；
- 返回终止状态位置；
- 按逻辑状态返回掩码；
- 检查状态是否完整且互斥；
- 判断八个子体素是否可以统一折叠；
- 返回统一折叠后的终止状态。

### 不负责

- 物理块类型；
- 节点池；
- 物理位布局。

`VoxelChildMask` 只负责把 `VoxelNodeBlock` 或 `VoxelLeafBlock` 转换为这一逻辑结果。

---

## 4.9 `VoxelTree`

### 定位

`VoxelTree` 保存一个第 0 层根体素的完整逻辑树和共享物理节点池。

### 负责

- 保存根逻辑状态；
- 保存根节点描述块；
- 持有 `VoxelBlockPool`；
- 复制时共享节点池；
- 首次编辑时执行单树级 COW；
- 创建根 Cursor 和根 Editor；
- 重置或释放整棵树；
- 提供存储统计；
- 检查逻辑树和物理节点池一致性。

### 不负责

- 根体素在全局网格中的索引；
- 跨根地址路由；
- Shape 级 COW；
- 修改影响记录；
- 自动合并策略。

`isValid()` 判断结构与存储是否一致。

`isNormalized()` 只表示是否已经消除全部可折叠结构，不是合法性的前置条件。合法树可以处于未完全合并状态。

---

## 4.10 `VoxelTreeCursor`

### 定位

`VoxelTreeCursor` 是单棵树中的轻量只读逻辑游标。

### 负责

- 返回当前位置逻辑状态；
- 判断当前位置是否终止或已细分；
- 返回八个直接子体素的批量逻辑状态；
- 访问指定直接子体素；
- 在适用位置读取压缩材料掩码；
- 屏蔽 Branch 和 MaskLeaf 的物理差异。

### 不负责

- 保存逻辑地址；
- 修改树；
- 缓存跨查询路径；
- 跨根访问；
- 延长树或节点池生命周期。

Cursor 是非拥有引用。树发生修改、释放或销毁后，已有 Cursor 不得继续使用。

---

## 4.11 `VoxelTreeEditor`

### 定位

`VoxelTreeEditor` 是单棵树中的局部即时修改原语。

### 负责

- 创建时保证当前树独占节点池；
- 返回当前位置状态；
- 将当前位置设置为 Empty 或 Material；
- 细分当前终止体素；
- 批量修改直接子体素；
- 进入指定子体素；
- 设置压缩材料掩码；
- 正确创建、替换和释放对应物理后代。

### 不负责

- 自动检查和合并祖先；
- 逻辑地址；
- 跨根路由；
- Session 变化记录；
- 算法级批量修改策略。

Editor 是非拥有引用。修改祖先结构后，已经取得的兄弟或后代 Editor 可能失效。

---

## 4.12 `VoxelTreeAccessor`

### 定位

`VoxelTreeAccessor` 根据逻辑地址定位单棵树中的节点，并复用连续查询的公共路径。

### 负责

- 绑定一棵树、根地址和最高允许层级；
- 将目标地址转换为角点路径；
- 复用上一次查询的公共前缀；
- 返回目标状态；
- 区分精确节点和继承自祖先的隐式状态；
- 返回实际解析地址和 Cursor；
- 提供缓存复用统计。

### 不负责

- 修改；
- COW；
- 跨根路由；
- 自动感知树变化。

树发生任何修改后，必须调用 `reset()`，或者销毁并重新创建 Accessor。

---

## 4.13 `VoxelForest`

### 定位

`VoxelForest` 是第 0 层根索引到 `VoxelTree` 的稀疏映射。

### 负责

- 保存实际存在的根树；
- 根据任意体素地址找到所属根树；
- 跨根查询逻辑状态和显式节点；
- 将地址级修改路由到对应树；
- 提供单地址 `setState()`、`split()` 和 `merge()` 原语；
- 新增、替换、移动和删除完整根树；
- 遍历根体素和压缩 Material 体素；
- 复制森林时共享各根树节点池。

### 不负责

- Shape 级 COW；
- 维护 `VoxelChangeSet`；
- 决定合并时机；
- 自动向祖先传播合并；
- 历史和版本；
- 实例变换。

`merge(address)` 只尝试合并指定地址，不继续处理任何祖先。

第 0 层终止 Empty 根不保存在 Forest 中。将根直接设置为 Empty，或显式合并根后得到 Empty，会删除对应根树。

---

## 4.14 `VoxelShape`

### 定位

`VoxelShape` 是核心层面向上层使用的体素体值对象。

其逻辑组成是：

```text
VoxelGrid
VoxelForest
Transform
```

### 负责

- 保存固定网格和稀疏森林；
- 保存独立实例变换；
- 复制时共享网格和森林数据；
- 提供只读状态查询；
- 提供只读 Forest；
- 创建 `VoxelShapeSession`；
- 判断共享状态；
- 提供值语义。

### 不负责

- 直接修改体素森林；
- 自动创建历史；
- 自动合并；
- 表面缓存；
- 建模或仿真策略。

空间变换不属于共享体素数据。修改 Transform 不会复制 Forest。

---

## 4.15 `VoxelShapeSession`

### 定位

`VoxelShapeSession` 是修改 `VoxelShape` 共享体素数据的唯一受控入口。

### 负责

- 构造时执行一次 Shape 级 COW；
- 在一次连续修改中复用独占的 Forest；
- 执行跨根 `setState()`、`split()` 和 `merge()`；
- 新增、替换和删除完整根树；
- 清空全部根树；
- 为每个成功修改同步维护 `VoxelChangeSet`；
- 提供分段 `takeChanges()`。

### 不负责

- 自动合并；
- 析构时提交或回滚；
- 历史版本保存；
- 操作日志；
- 并发写入协调。

Session 修改立即作用于目标 Shape，不存在独立提交阶段。

Session 存续期间：

- 不得复制或重新赋值目标 Shape；
- 不得创建另一个 Session 并行修改同一 Shape；
- 不得通过其他入口绕过 Session 修改 Forest；
- 不得在 Session 结束后继续持有内部树或森林引用。

---

## 4.16 `VoxelChangeSet`

### 定位

`VoxelChangeSet` 是一次 Session 修改产生的只读影响范围摘要。

### 负责

- 记录发生材料或显式结构变化的根索引；
- 在固定跟踪层级记录材料变化区域；
- 区分纯结构变化和材料变化；
- 表示整根材料区域失效；
- 被 Session 分段取走并交给缓存、表面或版本系统。

### 不负责

- 保存修改前状态；
- 保存修改后状态；
- 记录操作顺序；
- 撤销和重做；
- 重放修改；
- 独立修改 Shape；
- 由调用者人工添加或清空记录。

变化分类如下：

```text
split / merge
    modifiedRootIndices
    不产生材料脏区域

setState
    modifiedRootIndices
    产生材料脏区域

setTree / eraseTree / clear
    modifiedRootIndices
    整根材料区域失效
```

`VoxelChangeSet` 是变化足迹，不是历史对象，也不是可重放 Delta。

---

## 5. 写时复制边界

核心层使用两级 COW。

### 5.1 Shape 级 COW

复制 Shape：

```cpp
VoxelShape copy = original;
```

此时共享：

```text
VoxelGrid
VoxelForest根映射
各VoxelTree值对象
```

创建 Session：

```cpp
VoxelShapeSession session =
    copy.session(changeTrackingLevel);
```

如果共享数据引用数大于 1，Session 构造时复制：

```text
VoxelGrid
VoxelForest根映射
```

各根树内部节点池仍保持共享。

### 5.2 Tree 级 COW

第一次实际编辑某棵 `VoxelTree` 时：

```text
只深复制该树的可达物理节点
其他未修改根树继续共享原节点池
```

因此修改一个局部根树不会复制整个 Shape 的所有节点。

### 5.3 COW 不负责历史

COW 使历史快照成本较低，但 COW 本身不是历史系统。

历史层可以保存：

```text
before Shape
after Shape
VoxelChangeSet
```

具体版本组织方式属于更高层。

---

## 6. 读取流程

### 6.1 一次性 Shape 查询

```cpp
const VoxelState state =
    shape.state(address);

const bool explicitNode =
    shape.hasNode(address);
```

调用链：

```text
VoxelShape
    → VoxelForest
        → VoxelTree
            → VoxelTreeCursor
```

### 6.2 单树连续地址查询

```cpp
VoxelTreeAccessor accessor(
    tree,
    rootAddress,
    maximumLevel);

accessor.seek(firstAddress);
accessor.seek(adjacentAddress);
```

适用于同一根树内连续、相邻或具有较长公共路径的查询。

### 6.3 物理批量状态读取

Cursor 通过：

```cpp
VoxelChildStateMasks childStates =
    cursor.childStateMasks();
```

统一读取逻辑子体素状态。上层不需要判断当前底层使用普通节点块还是掩码叶块。

---

## 7. 修改流程

典型修改流程：

```cpp
VoxelShapeSession session =
    shape.session(changeTrackingLevel);

session.setState(addressA, VoxelState::Empty);
session.split(addressB);

if (shouldMerge)
{
    session.merge(addressC);
}

VoxelChangeSet changes =
    session.takeChanges();
```

调用链：

```text
VoxelShapeSession
    → VoxelForest
        → VoxelTreeEditor
            → VoxelBlockPool
```

同时：

```text
VoxelShapeSession
    → VoxelChangeSet
```

只有实际发生变化的原语才写入 ChangeSet。

---

## 8. 合并规则

### 8.1 局部合并

```cpp
session.merge(address);
```

仅当指定节点：

- 当前是 `Subdivided`；
- 八个直接子体素全部是 Empty，或全部是 Material；

才会合并。

该操作不会继续尝试父节点或更高祖先。

### 8.2 批量合并

核心层当前只提供单节点合并原语。

以下能力属于上层算法：

```text
mergeModified()
mergeSubtree()
mergeAll()
compact()
```

它们应当通过遍历候选地址并调用 Session 的显式合并原语实现，或者由后续专门的 Tree/Forest Operation 组件实现。

无论采用何种方式，都不得在普通 `setState()` 中隐式向上合并。

---

## 9. Empty 与显式结构

### 9.1 根层 Empty

Forest 不保存终止 Empty 根树。

```text
不存在的根树
    等价于第0层根逻辑状态为Empty
```

这是一项稀疏存储规则。

### 9.2 展开的全空结构

以下结构可以合法存在：

```text
Subdivided
└─ 所有后代最终均为Empty
```

由于核心层不自动合并，此时 Forest 中仍然存在显式根树。

因此当前：

```cpp
shape.isEmpty();
forest.isEmpty();
```

表示：

> 当前没有任何显式根树。

它不严格等价于：

> 整个逻辑材料场中不存在 Material。

上层不得使用 `isEmpty()` 代替完整材料存在性查询。

---

## 10. ChangeSet 消费规则

不同系统应消费 ChangeSet 的不同部分。

### 历史和版本

使用：

```cpp
changes.modifiedRootIndices();
```

判断哪些根发生了材料或正式结构变化。

### 表面和网格缓存

使用：

```cpp
changes.dirtyRegions();
```

只处理材料场变化区域。

纯 `split()` 或 `merge()` 不改变材料场，因此通常不需要重新构建表面。

### 整根替换或删除

`setTree()`、`eraseTree()` 和 `clear()` 会把对应根标记为完整材料变化。

消费者不得假设脏区域一定使用稠密位图；完整根可以由专用标志直接表示。

---

## 11. 生命周期与失效规则

### Cursor

以下操作后失效：

- 所属树被修改；
- 所属树被移动、重置或释放；
- 所属节点池被分离或回收；
- 所属树被销毁。

### Editor

以下操作可能使已有兄弟或后代 Editor 失效：

- 修改祖先为终止状态；
- 将 Branch 替换为 MaskLeaf；
- 释放或重新创建父节点八槽组；
- 重置或释放整棵树。

### Accessor

树发生任何修改后必须：

```cpp
accessor.reset();
```

或重新创建 Accessor。

### Session

Session 存续期间目标 Shape 必须保持稳定身份，不得通过其他路径修改其共享数据。

---

## 12. 对上层模块的约束

### 建模和体素化

可以：

- 创建 Session；
- 使用地址级原语；
- 构造完整根树后通过 `setTree()` 安装；
- 根据算法需要显式合并；
- 返回 ChangeSet。

不得：

- 直接取得可写 Forest；
- 直接修改 Shape 的共享数据；
- 伪造 ChangeSet。

### 布尔运算

可以：

- 只读访问输入 Shape；
- 修改输出 Shape 的 Session；
- 按根树或地址执行运算；
- 在算法结束时选择是否合并。

不得把自动规范化写入 Tree、Forest 或 Session 的普通修改路径。

### 磨削仿真

可以：

- 为一段轨迹创建或复用 Session；
- 连续修改局部地址；
- 分段 `takeChanges()`；
- 使用 ChangeSet 驱动增量表面重建；
- 在合适阶段调用显式合并算法。

核心层不决定一帧、一步或一段轨迹的边界。

### 历史和版本

可以：

- 利用 Shape 的 COW 保存快照；
- 将 ChangeSet 作为版本间影响范围元数据；
- 恢复旧 Shape。

不得把 ChangeSet 当作可逆操作日志。

---

## 13. 扩展规则

新增核心能力前，应先判断它属于哪一层。

### 应加入 Address 的能力

- 只依赖地址和层级的拓扑计算；
- 不读取树；
- 不读取网格空间参数。

### 应加入 Grid 的能力

- 连续局部空间和离散地址之间的确定性映射；
- 不读取材料和树。

### 应加入 Tree/Cursor/Editor 的能力

- 只作用于一个根树；
- 不需要知道全局根索引；
- 不需要 Shape 或 ChangeSet。

### 应加入 Forest 的能力

- 跨根路由；
- 根树容器管理；
- 不决定会话、历史和合并策略。

### 应加入 Session 的能力

- 会修改 Shape；
- 必须同步记录 ChangeSet；
- 需要跨根操作；
- 不应暴露可写内部引用。

### 不应加入核心层的能力

- 依赖具体几何类型；
- 依赖砂轮、刀具或机床；
- 依赖表面网格；
- 依赖线程调度；
- 依赖历史 UI；
- 依赖特定业务流程。

---

## 14. 核心层不变量

后续修改必须维持以下不变量：

1. 一个 `VoxelTree` 只表示一个第 0 层根体素。
2. 一个 Forest 根索引最多对应一棵树。
3. Forest 不保存终止 Empty 根树。
4. 逻辑 `Subdivided` 与具体 Branch/MaskLeaf 物理表示分离。
5. `VoxelTree::isValid()` 必须保证所有可达物理组与节点池分配状态一致。
6. Tree 复制共享节点池，首次编辑只分离目标树。
7. Shape 复制共享 Grid 和 Forest，Session 构造时执行 Shape 级分离。
8. Shape 的共享体素数据只能通过 Session 修改。
9. 每个成功 Session 修改必须同步写入 ChangeSet。
10. ChangeSet 对外只读，调用者不能伪造变化记录。
11. 普通修改不得自动合并祖先。
12. 未完全合并的结构仍然合法。
13. Cursor、Editor 和 Accessor 不拥有 Tree 生命周期。
14. 物理节点块和节点池不得泄露到 Shape 和业务层。

---

## 15. 总结

核心层的职责链为：

```text
VoxelAddress
    定义离散位置关系

VoxelGrid
    定义局部空间映射

VoxelBlock / Mask / Pool
    实现紧凑物理存储

VoxelTree
    保存一个根体素树

VoxelTreeCursor
    执行局部只读访问

VoxelTreeEditor
    执行局部修改原语

VoxelTreeAccessor
    加速单树连续地址查询

VoxelForest
    组织全部稀疏根树并执行跨根路由

VoxelShape
    提供体素体值语义和只读入口

VoxelShapeSession
    提供唯一受控写入口

VoxelChangeSet
    描述一次修改的影响范围
```

核心层的目标不是包含所有功能，而是提供稳定、可预测、可优化的数据边界，使建模、布尔、仿真、表面、缓存和历史系统能够在其上独立演进。
