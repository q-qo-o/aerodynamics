# MIT License
#
# Copyright (c) 2024 <COPYRIGHT_HOLDERS>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

import carb
import omni.ext
import omni.kit.app
import omni.usd
import omni.kit.commands
import omni.kit.context_menu
import os
from pxr import Sdf, Usd, UsdGeom, UsdPhysics

from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items
import omni.kit.window.property

from .ui_builder import UIBuilder
from .script_template import AERODYNAMICS_SCRIPT_TEMPLATE
from .aerodynamics_property_widget import AerodynamicsPropertyWidget


class Extension(omni.ext.IExt):
    """The Extension class"""

    def on_startup(self, ext_id):
        """Method called when the extension is loaded/enabled"""
        carb.log_info(f"on_startup {ext_id}")
        self._ext_id = ext_id
        ext_path = omni.kit.app.get_app().get_extension_manager().get_extension_path(ext_id)

        # UI handler
        self.ui_builder = UIBuilder(window_title="Aerodynamics", menu_path="Window/Aerodynamics")

        # Add to the Stage and Viewport right-click context menus via PrimPathWidget
        from omni.kit.property.usd import PrimPathWidget

        self._add_button_menu = PrimPathWidget.add_button_menu_entry(
            "ExtPhysics/Aerodynamics",
            show_fn=lambda objects: True,
            onclick_fn=lambda payload: self._add_aerodynamics(),
        )

        self._remove_button_menu = PrimPathWidget.add_button_menu_entry(
            "ExtPhysics/Remove Aerodynamics",
            show_fn=lambda objects: self._should_show_remove_menu(),
            onclick_fn=lambda payload: self._remove_aerodynamics(),
        )

        # Register property widget
        self._register_widget()

    def _should_show_add_menu(self):
        objects = omni.usd.get_context().get_selection().get_selected_prim_paths()
        if not objects:
            return False
        stage = omni.usd.get_context().get_stage()
        for path in objects:
            prim = stage.GetPrimAtPath(path)
            if (
                prim.IsValid()
                and prim.HasAPI(UsdPhysics.RigidBodyAPI)
                and not prim.HasAttribute("extphysics:aerodynamics:rho_air")
            ):
                return True
        return False

    def _should_show_remove_menu(self):
        objects = omni.usd.get_context().get_selection().get_selected_prim_paths()
        if not objects:
            return False
        stage = omni.usd.get_context().get_stage()
        for path in objects:
            prim = stage.GetPrimAtPath(path)
            if prim.IsValid() and prim.HasAttribute("extphysics:aerodynamics:rho_air"):
                return True
        return False

    def _register_widget(self):
        property_window = omni.kit.window.property.get_window()
        if property_window:
            property_window.register_widget(
                "prim",
                "aerodynamics_property_widget",
                AerodynamicsPropertyWidget(title="Aerodynamics", attribute_namespace_filter=["extphysics"]),
            )
            self._registered = True
        else:
            self._registered = False

    def _unregister_widget(self):
        property_window = omni.kit.window.property.get_window()
        if property_window and hasattr(self, "_registered") and self._registered:
            property_window.unregister_widget("prim", "aerodynamics_property_widget")
            self._registered = False

    def _add_aerodynamics(self):
        """为选中的 Prim 添加空气动力学属性和行为脚本"""
        stage = omni.usd.get_context().get_stage()
        if not stage:
            return

        selected_paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
        if not selected_paths:
            return

        # 动态文件管理：确保在工程根目录创建脚本
        layer = stage.GetRootLayer()
        if layer and not layer.anonymous:
            # 获取根路径
            stage_dir = os.path.dirname(layer.realPath)
        else:
            # 如果尚未保存，暂存到某个已知默认目录
            stage_dir = os.path.join(os.path.expanduser("~"), "Documents", "IsaacSim_Temp")

        scripts_dir = os.path.join(stage_dir, ".extphysics", "scripts")
        os.makedirs(scripts_dir, exist_ok=True)

        script_path = os.path.join(scripts_dir, "aerodynamics_script.py")

        # 将脚本写入文件
        if not os.path.exists(script_path):
            with open(script_path, "w", encoding="utf-8") as f:
                f.write(AERODYNAMICS_SCRIPT_TEMPLATE)

        # 把绝对路径转成相对于 stage 的路径（如果可以）。作为脚本引用最好是相对路径或绝对路径
        if layer and not layer.anonymous:
            script_usd_path = "./.extphysics/scripts/aerodynamics_script.py"
        else:
            script_usd_path = script_path.replace("\\", "/")

        for path in selected_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                continue

            # 严格检查必需的 Rigid Body API，如果不包含可以强制加，或者是跳过
            # 如果尚未应用刚体 API，则可选择不操作或应用
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                carb.log_warn(f"[Aerodynamics] Prim {path} does not have RigidBodyAPI. Skipping.")
                continue

            # 1. 设置自定义属性
            self._create_or_update_attr(prim, "extphysics:aerodynamics:rho_air", Sdf.ValueTypeNames.Double, 1.225)
            self._create_or_update_attr(prim, "extphysics:aerodynamics:C_Dx", Sdf.ValueTypeNames.Double, 1.0)
            self._create_or_update_attr(prim, "extphysics:aerodynamics:C_Dy", Sdf.ValueTypeNames.Double, 1.0)
            self._create_or_update_attr(prim, "extphysics:aerodynamics:C_Dz", Sdf.ValueTypeNames.Double, 1.0)
            self._create_or_update_attr(prim, "extphysics:aerodynamics:C_L0", Sdf.ValueTypeNames.Double, 0.0)
            self._create_or_update_attr(prim, "extphysics:aerodynamics:C_La", Sdf.ValueTypeNames.Double, 0.1)

            # 添加稳定性/阻尼力矩项
            self._create_or_update_attr(prim, "extphysics:aerodynamics:C_lp", Sdf.ValueTypeNames.Double, 0.1)  # 滚转阻尼
            self._create_or_update_attr(prim, "extphysics:aerodynamics:C_mq", Sdf.ValueTypeNames.Double, 0.1)  # 俯仰阻尼
            self._create_or_update_attr(prim, "extphysics:aerodynamics:C_nr", Sdf.ValueTypeNames.Double, 0.1)  # 偏航阻尼
            self._create_or_update_attr(prim, "extphysics:aerodynamics:C_m_alpha", Sdf.ValueTypeNames.Double, 0.0)  # 俯仰静稳定度

            # 特殊功能/调试选项
            self._create_or_update_attr(
                prim, "extphysics:aerodynamics:debug_print", Sdf.ValueTypeNames.Bool, False
            )  # 数据打印开关

            # 2. 挂载 Behavior Script
            omni.kit.commands.execute("ApplyScriptingAPICommand", paths=[Sdf.Path(path)])
            scripts_attr_name = "omni:scripting:scripts"
            scripts_attr = prim.GetAttribute(scripts_attr_name)

            val = scripts_attr.Get()
            if val is None:
                scripts_attr.Set([script_usd_path])
            else:
                scripts_list = list(val)
                if script_usd_path not in scripts_list:
                    scripts_list.append(script_usd_path)
                    scripts_attr.Set(scripts_list)

            carb.log_info(f"[Aerodynamics] Added aerodynamics properties and script to {path}")

    def _remove_aerodynamics(self):
        """为选中的 Prim 移除空气动力学属性和行为脚本"""
        stage = omni.usd.get_context().get_stage()
        if not stage:
            return

        selected_paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
        if not selected_paths:
            return

        script_usd_path = "./.extphysics/scripts/aerodynamics_script.py"

        for path in selected_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                continue

            # 1. 移除自定义属性
            aero_attrs = [attr for attr in prim.GetAttributes() if attr.GetName().startswith("extphysics:aerodynamics:")]
            for attr in aero_attrs:
                prim.RemoveProperty(attr.GetName())

            # 2. 卸载 Behavior Script
            scripts_attr_name = "omni:scripting:scripts"
            scripts_attr = prim.GetAttribute(scripts_attr_name)

            if scripts_attr:
                val = scripts_attr.Get()
                if val is not None:
                    scripts_list = list(val)
                    if script_usd_path in scripts_list:
                        scripts_list.remove(script_usd_path)
                        if scripts_list:
                            scripts_attr.Set(scripts_list)
                        else:
                            # 如果 scripts 数组清空了，可以直接移除整个属性
                            prim.RemoveProperty(scripts_attr_name)

            carb.log_info(f"[Aerodynamics] Removed aerodynamics properties and script from {path}")

    def _create_or_update_attr(self, prim: Usd.Prim, attr_name: str, value_type: Sdf.ValueTypeName, default_val):
        attr = prim.GetAttribute(attr_name)
        if not attr:
            attr = prim.CreateAttribute(attr_name, value_type, custom=True)
            attr.Set(default_val)
        else:
            if attr.Get() is None:
                attr.Set(default_val)

    def on_shutdown(self):
        """Method called when the extension is disabled"""
        carb.log_info(f"on_shutdown")

        # clean up UI
        self.ui_builder.cleanup()

        # remove menu items
        from omni.kit.property.usd import PrimPathWidget

        if hasattr(self, "_add_button_menu") and self._add_button_menu:
            PrimPathWidget.remove_button_menu_entry(self._add_button_menu)
            self._add_button_menu = None

        if hasattr(self, "_remove_button_menu") and self._remove_button_menu:
            PrimPathWidget.remove_button_menu_entry(self._remove_button_menu)
            self._remove_button_menu = None

        # unregister property widget
        self._unregister_widget()
