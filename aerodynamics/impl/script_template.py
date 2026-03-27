AERODYNAMICS_SCRIPT_TEMPLATE = """\
import carb
import numpy as np
import math
from pxr import UsdGeom, Gf, Usd, PhysxSchema, Sdf, PhysicsSchemaTools, UsdPhysics
from omni.kit.scripting import BehaviorScript
import omni.physx
import omni.usd
from omni.physx import get_physx_simulation_interface
import omni.kit.window.property

class AerodynamicsScript(BehaviorScript):
    BEHAVIOR_NS = "aerodynamics"
    EXPOSED_ATTR_NS = "extphysics"

    VARIABLES_TO_EXPOSE = [
        # 空气属性
        {"attr_name": "rho_air", "attr_type": Sdf.ValueTypeNames.Double, "default_value": 1.225, "doc": "Air Density (kg/m^3)", "group": "Air Properties"},
        
        # 阻力系数
        {"attr_name": "C_Dx", "attr_type": Sdf.ValueTypeNames.Double, "default_value": 1.0, "doc": "Drag Coefficient X", "group": "Drag Coefficients"},
        {"attr_name": "C_Dy", "attr_type": Sdf.ValueTypeNames.Double, "default_value": 1.0, "doc": "Drag Coefficient Y", "group": "Drag Coefficients"},
        {"attr_name": "C_Dz", "attr_type": Sdf.ValueTypeNames.Double, "default_value": 1.0, "doc": "Drag Coefficient Z", "group": "Drag Coefficients"},
        
        # 升力系数
        {"attr_name": "C_L0", "attr_type": Sdf.ValueTypeNames.Double, "default_value": 0.0, "doc": "Base Lift Coefficient", "group": "Lift Coefficients"},
        {"attr_name": "C_La", "attr_type": Sdf.ValueTypeNames.Double, "default_value": 0.1, "doc": "Lift Coefficient per Alpha", "group": "Lift Coefficients"},

        # 稳定性与阻尼力矩系数
        {"attr_name": "C_lp", "attr_type": Sdf.ValueTypeNames.Double, "default_value": 0.1, "doc": "Roll Damping", "group": "Stability & Damping"},
        {"attr_name": "C_mq", "attr_type": Sdf.ValueTypeNames.Double, "default_value": 0.1, "doc": "Pitch Damping", "group": "Stability & Damping"},
        {"attr_name": "C_nr", "attr_type": Sdf.ValueTypeNames.Double, "default_value": 0.1, "doc": "Yaw Damping", "group": "Stability & Damping"},
        {"attr_name": "C_m_alpha", "attr_type": Sdf.ValueTypeNames.Double, "default_value": 0.0, "doc": "Pitch Static Stability", "group": "Stability & Damping"},

        # 调试开关
        {"attr_name": "debug_print", "attr_type": Sdf.ValueTypeNames.Bool, "default_value": False, "doc": "Print Debug Data", "group": "Debug"},
    ]

    def on_init(self):
        # 默认值
        self._rho_air = 1.225
        self._C_Dx = 1.0
        self._C_Dy = 1.0
        self._C_Dz = 1.0
        self._C_L0 = 0.0
        self._C_La = 0.1
        self._C_lp = 0.1
        self._C_mq = 0.1
        self._C_nr = 0.1
        self._C_m_alpha = 0.0
        self._debug_print = False
        
        self._rb_path = None
        self._frame_count = 0

        carb.log_info(f"{type(self).__name__}.on_init()->{self.prim_path}")
        
        # Expose variables
        for var in self.VARIABLES_TO_EXPOSE:
            attr_name = var["attr_name"]
            full_attr_name = f"{self.EXPOSED_ATTR_NS}:{self.BEHAVIOR_NS}:{attr_name}"
            attr = self.prim.GetAttribute(full_attr_name)
            if not attr:
                attr = self.prim.CreateAttribute(full_attr_name, var["attr_type"])
                attr.Set(var["default_value"])
                if var.get("doc"):
                    attr.SetDocumentation(var["doc"])
                if var.get("group"):
                    attr.SetDisplayGroup(var["group"])
                    
        omni.kit.window.property.get_window().request_rebuild()

    def on_destroy(self):
        carb.log_info(f"{type(self).__name__}.on_destroy()->{self.prim_path}")
        
        for var in self.VARIABLES_TO_EXPOSE:
            attr_name = var["attr_name"]
            full_attr_name = f"{self.EXPOSED_ATTR_NS}:{self.BEHAVIOR_NS}:{attr_name}"
            attr = self.prim.GetAttribute(full_attr_name)
            if attr:
                self.prim.RemoveProperty(attr.GetName())
                
        omni.kit.window.property.get_window().request_rebuild()

    def _get_exposed_variable(self, attr_name):
        full_attr_name = f"{self.EXPOSED_ATTR_NS}:{self.BEHAVIOR_NS}:{attr_name}"
        attr = self.prim.GetAttribute(full_attr_name)
        if attr:
            return attr.Get()
        return None

    def _update_params(self):
        val = self._get_exposed_variable("rho_air"); 
        if val is not None: self._rho_air = val
        val = self._get_exposed_variable("C_Dx"); 
        if val is not None: self._C_Dx = val
        val = self._get_exposed_variable("C_Dy"); 
        if val is not None: self._C_Dy = val
        val = self._get_exposed_variable("C_Dz"); 
        if val is not None: self._C_Dz = val
        val = self._get_exposed_variable("C_L0"); 
        if val is not None: self._C_L0 = val
        val = self._get_exposed_variable("C_La"); 
        if val is not None: self._C_La = val
        val = self._get_exposed_variable("C_lp"); 
        if val is not None: self._C_lp = val
        val = self._get_exposed_variable("C_mq"); 
        if val is not None: self._C_mq = val
        val = self._get_exposed_variable("C_nr"); 
        if val is not None: self._C_nr = val
        val = self._get_exposed_variable("C_m_alpha"); 
        if val is not None: self._C_m_alpha = val
        val = self._get_exposed_variable("debug_print"); 
        if val is not None: self._debug_print = val

    def on_play(self):
        stage = omni.usd.get_context().get_stage()
        self._prim = stage.GetPrimAtPath(self.prim_path)
        self._update_params()
        
        self._frame_count = 0
        
        # Find RigidBody API
        curr = self._prim
        self._rb_path = None
        while curr and curr.GetPath() != Sdf.Path("/"):
            if curr.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
                self._rb_path = str(curr.GetPath())
                break
            curr = curr.GetParent()
        
        if not self._rb_path and self._prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
            self._rb_path = str(self.prim_path)

        # Initialize omni.physics.tensors RigidBodyView for extremely fast backend physics queries
        import omni.physics.tensors.impl.api as opt
        try:
            self._sim_view = opt.create_simulation_view("numpy")
            self._sim_view.set_subspace_roots("/")
            # Use the raw USD path as the pattern (forward slashes are valid absolute paths)
            pattern = self._rb_path
            self._rb_view = self._sim_view.create_rigid_body_view(pattern)
            carb.log_info(f"Aerodynamics: Initialized Tensor View for {self._rb_path}")
        except Exception as e:
            carb.log_error(f"Aerodynamics: Failed to initialize tensor view for {self._rb_path}: {e}")
            self._rb_view = None

        carb.log_info(f"Aerodynamics: RigidBody path: {self._rb_path}")

    def on_pause(self):
        pass

    def on_stop(self):
        pass

    def on_update(self, current_time: float, delta_time: float):
        if not self._rb_path:
            return

        self._update_params()
        
        stage = omni.usd.get_context().get_stage()
        rb_prim = stage.GetPrimAtPath(self._rb_path)
        
        # World Velocity (using omni.physics.tensors)
        lin_vel_world = np.zeros(3)
        ang_vel_world = np.zeros(3)
        
        # Check if tensor view exists and is successfully matched (backend is valid)
        if getattr(self, "_rb_view", None) is not None and getattr(self._rb_view, "_backend", None) is not None:
            # velocities shape usually is (num_rigid_bodies, 6)
            velocities = self._rb_view.get_velocities()
            if velocities is not None and len(velocities) > 0:
                lin_vel_world = velocities[0][:3]
                ang_vel_world = velocities[0][3:]
        else:
            # Fallback to USD API if tensors are unavailable
            rb_api = UsdPhysics.RigidBodyAPI(rb_prim)
            lin_vel_world = np.array(rb_api.GetVelocityAttr().Get() or [0,0,0])
            ang_vel_world = np.array(rb_api.GetAngularVelocityAttr().Get() or [0,0,0])
        
        # Transform World Velocity to Body Local Frame
        # USD matrices are row-major, meaning rows of rot_mat represent the local X, Y, Z axes in world space.
        # Projecting world vectors onto local axes equals multiplying the row-major matrix with the column vector.
        world_transform = omni.usd.get_world_transform_matrix(rb_prim)
        m_np = np.array(world_transform).reshape(4, 4)
        rot_mat = m_np[:3, :3]
        
        lin_vel_body = rot_mat @ lin_vel_world
        ang_vel_body = rot_mat @ ang_vel_world
        
        u, v, w = lin_vel_body[0], lin_vel_body[1], lin_vel_body[2]
        p, q, r = ang_vel_body[0], ang_vel_body[1], ang_vel_body[2]
        V_mag = float(np.linalg.norm(lin_vel_body))
        
        if V_mag < 1e-3:
            return

        # 获取 OBB
        bbox_cache = UsdGeom.BBoxCache(current_time, [UsdGeom.Tokens.default_])
        local_bounds = bbox_cache.ComputeLocalBound(self._prim)
        bounds_range = local_bounds.GetRange()
        
        if not bounds_range.IsEmpty():
            size = bounds_range.GetSize()
            L_x, L_y, L_z = size[0], size[1], size[2]
        else:
            L_x, L_y, L_z = 1.0, 1.0, 1.0
            
        # --- 动态阻力计算 (Drag) ---
        S_aero_drag = (abs(u)*L_y*L_z + abs(v)*L_x*L_z + abs(w)*L_x*L_y) / max(V_mag, 1e-6)
        
        F_drag_x = -0.5 * self._rho_air * S_aero_drag * self._C_Dx * abs(u) * u
        F_drag_y = -0.5 * self._rho_air * S_aero_drag * self._C_Dy * abs(v) * v
        F_drag_z = -0.5 * self._rho_air * S_aero_drag * self._C_Dz * abs(w) * w

        # --- 动态升力计算 (Lift) ---
        S_aero_lift = (L_x * L_y)
        V_xz_sq = u*u + w*w
        alpha = math.atan2(-w, u)
        
        C_L = self._C_L0 + self._C_La * alpha
        V_xz_mag_safe = math.sqrt(V_xz_sq + 1e-6)
        lift_dir_x = -w / V_xz_mag_safe
        lift_dir_z = u / V_xz_mag_safe
        
        Lift_mag = 0.5 * self._rho_air * S_aero_lift * V_xz_sq * C_L
        F_lift_x = Lift_mag * lift_dir_x
        F_lift_z = Lift_mag * lift_dir_z

        # --- 力矩计算 ---
        M_roll_damp = -0.5 * self._rho_air * V_mag * S_aero_lift * L_x * self._C_lp * p
        M_pitch_damp = -0.5 * self._rho_air * V_mag * S_aero_lift * L_y * self._C_mq * q
        M_yaw_damp = -0.5 * self._rho_air * V_mag * S_aero_lift * L_z * self._C_nr * r
        M_pitch_static = 0.5 * self._rho_air * V_xz_sq * S_aero_lift * L_y * self._C_m_alpha * alpha
        
        T_local = np.array([
            M_roll_damp,
            M_pitch_damp + M_pitch_static,
            M_yaw_damp
        ])

        F_total_local = np.array([
            F_drag_x + F_lift_x,
            F_drag_y,
            F_drag_z + F_lift_z
        ])
        
        # 将局部力转回世界空间并施加 (Local to World)
        # For row-major USD matrices, local vectors are converted to world space via vec @ matrix
        total_force_world = F_total_local @ rot_mat
        total_torque_world = T_local @ rot_mat
        
        # --- 获取物体真实的质心 (Center of Mass) ---
        mass_api = UsdPhysics.MassAPI(rb_prim)
        com_local = np.zeros(3)
        if mass_api:
            com_usd = mass_api.GetCenterOfMassAttr().Get()
            if com_usd is not None:
                com_local = np.array(com_usd)
                
        # 将局部质心转换到世界坐标系下
        body_pos_world = com_local @ m_np[:3, :3] + m_np[3, :3]
        
        stage_id = omni.usd.get_context().get_stage_id()
        body_path_int = int(PhysicsSchemaTools.sdfPathToInt(Sdf.Path(self._rb_path)))
        physx = get_physx_simulation_interface()
        
        carb_force = carb.Float3(float(total_force_world[0]), float(total_force_world[1]), float(total_force_world[2]))
        carb_pos = carb.Float3(float(body_pos_world[0]), float(body_pos_world[1]), float(body_pos_world[2]))
        
        if np.linalg.norm(total_force_world) > 1e-6:
            physx.apply_force_at_pos(stage_id, body_path_int, carb_force, carb_pos, "Force")
            
        carb_torque = carb.Float3(float(total_torque_world[0]), float(total_torque_world[1]), float(total_torque_world[2]))
        
        if np.linalg.norm(total_torque_world) > 1e-6:
            physx.apply_torque(stage_id, body_path_int, carb_torque)
            
        if self._debug_print:
            self._frame_count += 1
            if self._frame_count % 60 == 0:
                print(f"[Aerodynamics] {self.prim_path} | V:{V_mag:.2f} m/s | Alpha:{math.degrees(alpha):.1f}deg")
                print(f"  -> Vel L[x,y,z] : ({u:.2f}, {v:.2f}, {w:.2f})")
                print(f"  -> Ang L[p,q,r] : ({p:.2f}, {q:.2f}, {r:.2f})")
                print(f"  -> Px Force (W) : ({total_force_world[0]:.1f}, {total_force_world[1]:.1f}, {total_force_world[2]:.1f})")
                print(f"  -> Lift & Drag  : Lift={Lift_mag:.1f} N, Drag(X)={F_drag_x:.1f} N")

    def _get_attribute(self, prim: Usd.Prim, attr_name: str, default_val: float):
        attr = prim.GetAttribute(attr_name)
        if attr and attr.IsValid():
            val = attr.Get()
            if val is not None:
                return float(val)
        return default_val
"""
