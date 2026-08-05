在 **面向数据设计（DOD）与状态上提（State Ascension）** 架构下，这 7 种原子动作（`VoxelBooleanAction`）构成了一个**完备的代数映射全集**。

我们通过代数符号与全真值表，从数学上推导它的完备性。

---

### 1. 基础映射与输入空间集

在体素八叉树中，任意节点 $N$ 在逻辑上仅可能处于集合 $S$ 中的三种状态之一：


$$S = \{ \text{Empty (E)}, \text{Material (M)}, \text{Subdivided (S)} \}$$

当两个节点（左节点 $L \in S$，右节点 $R \in S$）执行任意布尔运算 $\circ \in \{ \cup, \cap, \setminus, \oplus \}$ 时，输入空间为两个节点的笛卡尔积：


$$I = S \times S = \{ (E, E), (E, M), (E, S), (M, E), (M, M), (M, S), (S, E), (S, M), (S, S) \}$$


输入空间包含 **$3 \times 3 = 9$ 种可能的二元组合**。

目标是寻找一个函数 $A_\circ: I \to \text{Actions}$，将这 9 种输入映射到对目标节点（左节点 $L$）最简且不退化的**物理处置动作集合 $\text{Actions}$** 上。

---

### 2. 全运算代数真值表推导

我们对四种布尔运算在 9 种输入条件下的**理想输出状态 $O$** 及**对左节点 $L$ 的最简物理处置指令**进行逐一推导：

#### 2.1 并集运算 ($\text{Union } \cup$)

* $(E, E) \to E$ ：左侧本身为 $E$，保持不变 $\Rightarrow$ **`KeepLeft`**
* $(E, M) \to M$ ：左侧原为 $E$，需置为 $M$ $\Rightarrow$ **`SetMaterial`**
* $(E, S) \to S$ ：左侧原为 $E$，需直接继承右侧树结构 $S$ $\Rightarrow$ **`CopyRight`**
* $(M, E) \to M$ ：左侧已经是 $M$，保持不变 $\Rightarrow$ **`KeepLeft`**
* $(M, M) \to M$ ：左侧已经是 $M$，保持不变 $\Rightarrow$ **`KeepLeft`**
* $(M, S) \to M$ ：左侧已经是 $M$，保持不变 $\Rightarrow$ **`KeepLeft`**
* $(S, E) \to S$ ：右侧为空，左侧保持 $S$ 不变 $\Rightarrow$ **`KeepLeft`**
* $(S, M) \to M$ ：右侧覆盖全材料，左侧由 $S$ 变 $M$ $\Rightarrow$ **`SetMaterial`**
* $(S, S) \to \text{Recurse}$ ：双方均为分支，需下潜递归处理 $\Rightarrow$ **`Recurse`**

#### 2.2 交集运算 ($\text{Intersection } \cap$)

* $(E, \text{any}) \to E$ ：左侧本身为 $E$，保持不变 $\Rightarrow$ **`KeepLeft`**
* $(M, E) \to E$ ：右侧无材料，左侧由 $M$ 置为 $E$ $\Rightarrow$ **`SetEmpty`**
* $(M, M) \to M$ ：左侧已经是 $M$，保持不变 $\Rightarrow$ **`KeepLeft`**
* $(M, S) \to S$ ：与右侧求交，直接继承右侧树结构 $S$ $\Rightarrow$ **`CopyRight`**
* $(S, E) \to E$ ：右侧为空，左侧由 $S$ 置为 $E$ $\Rightarrow$ **`SetEmpty`**
* $(S, M) \to S$ ：与全材料求交，左侧保持 $S$ 不变 $\Rightarrow$ **`KeepLeft`**
* $(S, S) \to \text{Recurse}$ ：双方均为分支，需下潜递归处理 $\Rightarrow$ **`Recurse`**

#### 2.3 差集运算 ($\text{Difference } \setminus$)

* $(E, \text{any}) \to E$ ：左侧本身为 $E$，保持不变 $\Rightarrow$ **`KeepLeft`**
* $(M, E) \to M$ ：右侧无材料，左侧保持 $M$ 不变 $\Rightarrow$ **`KeepLeft`**
* $(M, M) \to E$ ：材料完全被减去，左侧由 $M$ 置为 $E$ $\Rightarrow$ **`SetEmpty`**
* $(M, S) \to \neg S$ ：材料减去右树，等价于**复制右树并全树取反** $\Rightarrow$ **`CopyInvertedRight`**
* $(S, E) \to S$ ：右侧无材料，左侧保持 $S$ 不变 $\Rightarrow$ **`KeepLeft`**
* $(S, M) \to E$ ：左侧分支全被大材料减去，置为 $E$ $\Rightarrow$ **`SetEmpty`**
* $(S, S) \to \text{Recurse}$ ：双方均为分支，需下潜递归处理 $\Rightarrow$ **`Recurse`**

#### 2.4 异或运算 ($\text{ExclusiveOr } \oplus$)

* $(E, E) \to E$ ：保持不变 $\Rightarrow$ **`KeepLeft`**
* $(E, M) \to M$ ：置为材料 $\Rightarrow$ **`SetMaterial`**
* $(E, S) \to S$ ：继承右树 $\Rightarrow$ **`CopyRight`**
* $(M, E) \to M$ ：保持不变 $\Rightarrow$ **`KeepLeft`**
* $(M, M) \to E$ ：置为空 $\Rightarrow$ **`SetEmpty`**
* $(M, S) \to \neg S$ ：复制右树并全树取反 $\Rightarrow$ **`CopyInvertedRight`**
* $(S, E) \to S$ ：保持不变 $\Rightarrow$ **`KeepLeft`**
* $(S, M) \to \neg S$ ：保持左树结构，**将左树全树取反** $\Rightarrow$ **`InvertLeft`**
* $(S, S) \to \text{Recurse}$ ：双方均为分支，需下潜递归处理 $\Rightarrow$ **`Recurse`**

---

### 3. 完备性与无冗余性证明

汇总上述四个代数运算真值表在目标集（Image）上的映射：

$$\text{Image}(A_\cup \cup A_\cap \cup A_\setminus \cup A_\oplus) = \{ \text{KeepLeft}, \text{SetEmpty}, \text{SetMaterial}, \text{CopyRight}, \text{InvertLeft}, \text{CopyInvertedRight}, \text{Recurse} \}$$

#### 3.1 完备性（Completeness）

推导证明，对于 $4 \times 9 = 36$ 种二元 CSG 布尔运算场景，**不存在任何一个场景的物理处置超越这 7 种动作之外**。
任何输入的运算解都必定落在这 7 种原子指令构成的全集中，映射函数 $A_\circ$ 是完备的。

#### 3.2 最小无冗余性（Minimal Non-Redundancy）

这 7 种动作在代数与物理上具有相互独立性，**缺少任意一种都会破坏 $O(1)$ 的直接裁决能力**：

1. **`KeepLeft`**：代表 $O(1)$ 零操作（No-Op）。
2. **`SetEmpty`**：代表 $O(1)$ 的零解折叠（无需遍历子树，直接擦除）。
3. **`SetMaterial`**：代表 $O(1)$ 的全解折叠（无需遍历子树，直接填满）。
4. **`CopyRight`**：代表 $O(1)$ 的结构平移（将右侧 $S$ 树指针/掩码直接挂载给左侧）。
5. **`InvertLeft`**：代表对左树结构进行 $O(1)$ 或掩码级的按位取反（$\neg L$）。
6. **`CopyInvertedRight`**：代表对右树结构进行复制的同时取反挂载（$\neg R$）。
7. **`Recurse`**：代表唯一必须进行指针/层级下潜的递归分支。

---

### 4. 结论

`VoxelBooleanAction` 的 7 种原子指令，是在布尔代数（Boolean Algebra）与八叉树分形空间下，对 **“零操作、状态折叠、结构复制/取反、层级递归”** 四大类物理行为的**封闭且完备（Closed and Complete）的代数抽象**。