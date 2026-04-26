# 当前水动力学（流体动力学）仿真方案与动力学模型

本文档描述了当前工程中用于水动力学/空气动力学仿真的实现方案及具体数学模型。该仿真方案基于物理引擎（PhysX）与自定义的行为脚本（BehaviorScript）结合，通过实时获取刚体的运动状态（线速度、角速度），计算流体对刚体产生的动态阻力、升力以及阻尼力矩，最终通过物理引擎接口施加到刚体上。

## 1. 仿真方案概述

当前方案属于**基于经验系数的局部空气/水动力学受力模型**（Phenomenological Fluid Dynamics Model）。系统在每个物理更新步长（`on_update`）执行以下流程：

1. **状态获取**：通过 `omni.physics.tensors` 获取刚体在世界坐标系下的线速度与角速度。
2. **坐标系转换**：将世界速度转换到刚体的局部坐标系（Local Frame）下，得到局部速度向量 $(u, v, w)$ 和角速度向量 $(p, q, r)$。
3. **特征面积计算**：通过 USD 几何边界框（Bounding Box）动态计算物体的特征尺寸（$L_x, L_y, L_z$），从而得到等效迎风面积。
4. **受力计算**：计算三个正交方向的阻力（Drag）、基于攻角（Angle of Attack）的升力（Lift）以及由角速度引起的阻尼力矩（Damping Torques）。
5. **施加力与力矩**：将计算得到的局部力和力矩转换回世界坐标系，通过 `physx.apply_force_at_pos` 作用在刚体质心（Center of Mass）上。

---

## 2. 核心动力学模型与方程

在以下方程中：
* $\rho$ 为流体密度（如水密度，对应代码中的 `rho_air`）。
* $(u, v, w)$ 为局部坐标系下 X、Y、Z 轴的线速度。
* $(p, q, r)$ 为局部坐标系下的角速度（滚转、俯仰、偏航）。
* $V_{mag} = \sqrt{u^2 + v^2 + w^2}$ 为总速度大小。
* $L_x, L_y, L_z$ 为刚体在三个轴向上的特征长度（通过包围盒大小获取）。

### 2.1 特征面积 (Reference Areas)

**阻力等效面积** $S_{drag}$：
$$ S_{drag} = \frac{|u| \cdot L_y L_z + |v| \cdot L_x L_z + |w| \cdot L_x L_y}{\max(V_{mag}, 10^{-6})} $$
该公式基于物体在运动方向上的投影面积进行近似估算。

**升力等效面积** $S_{lift}$：
$$ S_{lift} = L_x \cdot L_y $$

### 2.2 阻力模型 (Drag Model)

阻力与速度的平方成正比，并在局部坐标系的三个轴向上独立计算。设 $C_{Dx}, C_{Dy}, C_{Dz}$ 分别为三轴阻力系数：

* **X轴阻力**：$$ F_{drag\_x} = -\frac{1}{2} \rho \cdot S_{drag} \cdot C_{Dx} \cdot |u| \cdot u $$
* **Y轴阻力**：$$ F_{drag\_y} = -\frac{1}{2} \rho \cdot S_{drag} \cdot C_{Dy} \cdot |v| \cdot v $$
* **Z轴阻力**：$$ F_{drag\_z} = -\frac{1}{2} \rho \cdot S_{drag} \cdot C_{Dz} \cdot |w| \cdot w $$

### 2.3 升力模型 (Lift Model)

升力主要由 XZ 平面内的相对速度引起（假设前进方向为 X，上下方向为 Z）。
* **攻角 (Angle of Attack, $\alpha$)**：
  $$ \alpha = \arctan2(-w, u) $$
* **XZ 平面速度的平方**：
  $$ V_{xz}^2 = u^2 + w^2 $$
* **升力系数 (Lift Coefficient, $C_L$)**：通过基础升力系数 $C_{L0}$ 和攻角导数 $C_{L\alpha}$ 线性插值：
  $$ C_L = C_{L0} + C_{L\alpha} \cdot \alpha $$
* **总升力大小**：
  $$ Lift_{mag} = \frac{1}{2} \rho \cdot S_{lift} \cdot V_{xz}^2 \cdot C_L $$
* **升力分量**（垂直于相对速度方向）：
  $$ F_{lift\_x} = Lift_{mag} \cdot \left(\frac{-w}{\sqrt{V_{xz}^2}}\right) $$
  $$ F_{lift\_z} = Lift_{mag} \cdot \left(\frac{u}{\sqrt{V_{xz}^2}}\right) $$

### 2.4 力矩与稳定性模型 (Torque and Stability Model)

力矩主要包括滚转、俯仰、偏航的动阻尼力矩，以及俯仰方向的静稳定力矩。设 $C_{lp}, C_{mq}, C_{nr}$ 为三轴阻尼系数，$C_{m\alpha}$ 为俯仰静稳定系数：

* **滚转阻尼力矩 (Roll Damping)**：
  $$ M_{roll} = -\frac{1}{2} \rho \cdot V_{mag} \cdot S_{lift} \cdot L_x \cdot C_{lp} \cdot p $$
* **俯仰阻尼力矩 (Pitch Damping)**：
  $$ M_{pitch\_damp} = -\frac{1}{2} \rho \cdot V_{mag} \cdot S_{lift} \cdot L_y \cdot C_{mq} \cdot q $$
* **偏航阻尼力矩 (Yaw Damping)**：
  $$ M_{yaw} = -\frac{1}{2} \rho \cdot V_{mag} \cdot S_{lift} \cdot L_z \cdot C_{nr} \cdot r $$
* **俯仰静稳定力矩 (Pitch Static Stability)**：
  $$ M_{pitch\_static} = \frac{1}{2} \rho \cdot V_{xz}^2 \cdot S_{lift} \cdot L_y \cdot C_{m\alpha} \cdot \alpha $$

**总局部力矩**：
$$ T_{local} = \begin{bmatrix} M_{roll} \\ M_{pitch\_damp} + M_{pitch\_static} \\ M_{yaw} \end{bmatrix} $$

**总局部受力**：
$$ F_{local} = \begin{bmatrix} F_{drag\_x} + F_{lift\_x} \\ F_{drag\_y} \\ F_{drag\_z} + F_{lift\_z} \end{bmatrix} $$

## 3. 性能优化说明

1. **张量后端查询**：使用了 `omni.physics.tensors` (RigidBodyView) 接口取代了常规的 USD API。该接口直接与 PhysX 底层交互（支持 NumPy/CUDA 张量），极大地提高了速度获取与遍历的性能。
2. **安全阈值过滤**：当总体速度 $V_{mag} < 10^{-3} \text{ m/s}$ 时，系统会自动跳过复杂的动力学受力计算，有效避免物体在静止时由于数值误差导致的微小抖动或 NAN 异常。
