AERODYNAMICS_SCRIPT_TEMPLATE = """\
import omni.timeline
import numpy as np
import math
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf, PhysxSchema

class AerodynamicsScript:
    \"\"\"
    空气动力学(Aerodynamics) 行为脚本。
    通过读取 prim 的自定义属性和物理状态，在每个物理步(on_physics_step)中，
    计算基于三轴独立的空气阻力(Drag)与动态升力(Lift)，并将力施加于物体的局部中心。
    \"\"\"

    def __init__(self, prim: Usd.Prim):
        self._prim = prim
        self._timeline = omni.timeline.get_timeline_interface()
        self._frame_count = 0
        
        # 订阅物理步的回调
        self._physx_subscription = omni.physx.get_physx_interface().subscribe_physics_step_events(self.on_physics_step)

    def on_physics_step(self, dt: float):
        if not self._prim.IsValid():
            return
            
        stage = self._prim.GetStage()
        if not stage:
            return

        # 获取刚体 API 接口，用于施加受力
        rigid_body_api = UsdPhysics.RigidBodyAPI(self._prim)
        if not rigid_body_api:
            return
            
        # --- 1. 获取自定义参数 ---
        # 空气属性
        rho_air = self._get_attribute(self._prim, "extphysics:aerodynamics:rho_air", 1.225)
        
        # 阻力系数
        C_Dx = self._get_attribute(self._prim, "extphysics:aerodynamics:C_Dx", 1.0)
        C_Dy = self._get_attribute(self._prim, "extphysics:aerodynamics:C_Dy", 1.0)
        C_Dz = self._get_attribute(self._prim, "extphysics:aerodynamics:C_Dz", 1.0)
        
        # 升力系数
        C_L0 = self._get_attribute(self._prim, "extphysics:aerodynamics:C_L0", 0.0)
        C_La = self._get_attribute(self._prim, "extphysics:aerodynamics:C_La", 0.1)

        # 稳定性与阻尼力矩系数
        C_lp = self._get_attribute(self._prim, "extphysics:aerodynamics:C_lp", 0.1)
        C_mq = self._get_attribute(self._prim, "extphysics:aerodynamics:C_mq", 0.1)
        C_nr = self._get_attribute(self._prim, "extphysics:aerodynamics:C_nr", 0.1)
        C_m_alpha = self._get_attribute(self._prim, "extphysics:aerodynamics:C_m_alpha", 0.0)

        # 获取并计算空气暴露比例 lambda_aero
        # 从可能的其它扩展（如水动力）获取体积。如果有的话就解耦，没有则认为全在空气中
        V_total = self._get_attribute(self._prim, "extphysics:volume:total", -1.0)
        V_sub = self._get_attribute(self._prim, "extphysics:volume:submerged", 0.0)
        
        # 调试开关
        debug_print = self._get_attribute(self._prim, "extphysics:aerodynamics:debug_print", False)

        # 如果未定义总包络体积，可以通过 OBB 获取；在这里如果 V_total 没有提供，我们先取 fallback (后文计算包围盒后修正)
        lambda_aero = 1.0
        if V_total > 0.0:
            lambda_aero = max(0.0, 1.0 - (V_sub / max(V_total, 1e-6)))

        # 若完全没在空气中，跳过计算
        if lambda_aero <= 1e-6:
            return

        # --- 2. 获取动力学状态 (局部线速度与朝向) ---
        world_transform = omni.usd.get_world_transform_matrix(self._prim)
        world_to_local = world_transform.GetInverse()
        
        # 获取世界线速度
        # 在实际中，如果你用了 PhysX 的 rigid_body API 你可以获取速度，
        # 这里尝试读取 usd 属性或通过 PhysX 接口查询：
        linear_velocity = rigid_body_api.GetVelocityAttr().Get()
        if not linear_velocity:
            linear_velocity = Gf.Vec3f(0.0, 0.0, 0.0)
            
        # 考虑到更通用的 Isaac Sim/PhysX 环境，上述属性如果没有实时更新，也可通过 usdPhysics 读
        # 注意：此处简化处理，假设世界空间下的速度为 linear_velocity 的值格式
        # 计算局部速度 (使用 numpy 优化向量操作)
        vel_world = Gf.Vec3d(linear_velocity)
        # 只旋转不平移
        vel_local = world_to_local.TransformDir(vel_world)
        
        # 转为 numpy array 以便后续计算
        v_local_np = np.array([vel_local[0], vel_local[1], vel_local[2]])
        u, v, w = v_local_np[0], v_local_np[1], v_local_np[2]
        
        V_mag = np.linalg.norm(v_local_np)

        # 获取角速度
        angular_velocity = rigid_body_api.GetAngularVelocityAttr().Get()
        if not angular_velocity:
            angular_velocity = Gf.Vec3f(0.0, 0.0, 0.0)
            
        ang_vel_world = Gf.Vec3d(angular_velocity)
        ang_vel_local = world_to_local.TransformDir(ang_vel_world)
        p, q, r = ang_vel_local[0], ang_vel_local[1], ang_vel_local[2] # 滚转, 俯仰, 偏航角速度
        
        # 速度过小，避免除零直接跳过
        if V_mag < 1e-3:
            return

        # --- 3. 获取 OBB 用于参考面积估算 ---
        # 尝试读取 OBB 极值
        imageable = UsdGeom.Imageable(self._prim)
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        local_bounds = bbox_cache.ComputeLocalBound(self._prim)
        bounds_range = local_bounds.GetRange()
        
        if not bounds_range.IsEmpty():
            size = bounds_range.GetSize()
            L_x, L_y, L_z = size[0], size[1], size[2]
        else:
            # Fallback size
            L_x, L_y, L_z = 1.0, 1.0, 1.0
            
        # 动态更新 V_total
        if V_total < 0:
            V_total = L_x * L_y * L_z
            lambda_aero = max(0.0, 1.0 - (V_sub / max(V_total, 1e-6)))

        # --- 4. 动态阻力计算 (Drag - 三轴独立模型) ---
        # 阻力参考面积 (考虑到每个面的迎风投影)
        # 此处加上绝对值避免负数面积
        S_ref_drag = (abs(u)*L_y*L_z + abs(v)*L_x*L_z + abs(w)*L_x*L_y) / V_mag
        S_aero_drag = lambda_aero * S_ref_drag
        
        F_drag_x = -0.5 * rho_air * S_aero_drag * C_Dx * abs(u) * u
        F_drag_y = -0.5 * rho_air * S_aero_drag * C_Dy * abs(v) * v
        F_drag_z = -0.5 * rho_air * S_aero_drag * C_Dz * abs(w) * w

        # --- 5. 动态升力计算 (Lift - 依赖迎角) ---
        # 升力参考面积 (通常选取俯视或主翼面积)
        S_aero_lift = lambda_aero * (L_x * L_y)
        
        # 提取 X-Z 平面速度模长的平方
        V_xz_sq = u*u + w*w
        
        # 计算迎角 (Angle of Attack)
        alpha = math.atan2(-w, u)
        
        # 计算动态升力系数
        C_L = C_L0 + C_La * alpha
        
        # 升力方向单位向量 (防除零)
        V_xz_mag_safe = math.sqrt(V_xz_sq + 1e-6)
        lift_dir_x = -w / V_xz_mag_safe
        lift_dir_z = u / V_xz_mag_safe
        
        # 升力大小
        Lift_mag = 0.5 * rho_air * S_aero_lift * V_xz_sq * C_L
        
        F_lift_x = Lift_mag * lift_dir_x
        F_lift_z = Lift_mag * lift_dir_z

        # --- 6. 动态力矩计算 (Moment - 阻尼与静稳定性) ---
        # 旋转气动阻尼力矩: M = -0.5 * rho * V_mag * S * L * C_x * omega
        # 参考长度采用 OBB 尺寸估算
        M_roll_damp = -0.5 * rho_air * V_mag * S_aero_lift * L_x * C_lp * p
        M_pitch_damp = -0.5 * rho_air * V_mag * S_aero_lift * L_y * C_mq * q
        M_yaw_damp = -0.5 * rho_air * V_mag * S_aero_lift * L_z * C_nr * r
        
        # 俯仰静稳定力矩 (依赖迎角 alpha): 抬头为正(或负，取决于约定，此处假设直接叠加)
        M_pitch_static = 0.5 * rho_air * V_xz_sq * S_aero_lift * L_y * C_m_alpha * alpha
        
        T_local = Gf.Vec3f(
            float(M_roll_damp),
            float(M_pitch_damp + M_pitch_static),
            float(M_yaw_damp)
        )

        # --- 7. 物理引擎施加合成力与力矩 ---
        F_total_local = Gf.Vec3f(
            float(F_drag_x + F_lift_x),
            float(F_drag_y),
            float(F_drag_z + F_lift_z)
        )
        
        # 将局部力转回世界空间并施加（由于 PhysxForceAPI 通常是在世界坐标，或者可以使用其它方式）
        # Apply force to centroid
        F_total_world = world_transform.TransformDir(Gf.Vec3d(F_total_local))
        T_total_world = world_transform.TransformDir(Gf.Vec3d(T_local))
        
        # Apply standard physics force
        physx_force = PhysxSchema.PhysxForceAPI(self._prim)
        if not physx_force:
            physx_force = PhysxSchema.PhysxForceAPI.Apply(self._prim)
            
        physx_force.GetForceAttr().Set(Gf.Vec3f(F_total_world))
        physx_force.GetTorqueAttr().Set(Gf.Vec3f(T_total_world))
        physx_force.GetModeAttr().Set("force") # 作为力(而不是冲量)施加

        # --- 8. 调试打印 (Debug Print) ---
        if debug_print:
            self._frame_count += 1
            # 取大约每 60 物理帧打印一次（避免刷屏卡顿）
            if self._frame_count % 60 == 0:
                prim_name = self._prim.GetName()
                alpha_deg = math.degrees(alpha)
                print(f"[Aerodynamics] {prim_name} | V:{V_mag:.2f} m/s | Alpha:{alpha_deg:.1f}°")
                print(f"  -> Vel L[x,y,z] : ({u:.2f}, {v:.2f}, {w:.2f})")
                print(f"  -> Ang L[p,q,r] : ({p:.2f}, {q:.2f}, {r:.2f})")
                print(f"  -> Px Force (W) : ({F_total_world[0]:.1f}, {F_total_world[1]:.1f}, {F_total_world[2]:.1f})")
                print(f"  -> Px Torque(W) : ({T_total_world[0]:.1f}, {T_total_world[1]:.1f}, {T_total_world[2]:.1f})")
                print(f"  -> Lift & Drag  : Lift={Lift_mag:.1f} N, Drag(X)={F_drag_x:.1f} N")

    def _get_attribute(self, prim: Usd.Prim, attr_name: str, default_val: float):
        \"\"\"安全获取 USD 属性，不存在则返回默认值\"\"\"
        attr = prim.GetAttribute(attr_name)
        if attr and attr.IsValid():
            val = attr.Get()
            if val is not None:
                return float(val)
        return default_val

    def on_destroy(self):
        \"\"\"清理订阅\"\"\"
        self._physx_subscription = None
"""
