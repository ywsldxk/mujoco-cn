"""mujoco-cn: a Chinese MuJoCo scene viewer with keyboard teleoperation."""
from __future__ import annotations

import math
import os
from pathlib import Path

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

import mujoco
import numpy as np
import pygame
from object_import import SUPPORTED_SUFFIXES, attach_object

repo_root = Path(__file__).resolve().parent
os.environ.setdefault("MENAGERIE_CACHE_DIR", str(repo_root / "data" / "mujoco_menagerie_cache"))
import mujoco_menagerie as mm

JOINT_KEYS = {
    "shoulder_lift": (pygame.K_w, pygame.K_s),
    "shoulder_pan": (pygame.K_a, pygame.K_d),
    "elbow_flex": (pygame.K_i, pygame.K_k),
    "wrist_roll": (pygame.K_j, pygame.K_l),
    "wrist_flex": (pygame.K_LCTRL, pygame.K_RCTRL),
}
JOINT_LABELS = {
    "shoulder_pan": "肩部左右转",
    "shoulder_lift": "肩部升降",
    "elbow_flex": "肘部",
    "wrist_flex": "腕部俯仰",
    "wrist_roll": "腕部旋转",
    "gripper": "夹爪",
}
JOINT_RATE = math.radians(42.0)
GRIPPER_RATE = math.radians(55.0)
HOME_POSE = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0004,
    "elbow_flex": 0.4735,
    "wrist_flex": 1.1772,
    "wrist_roll": 1.5844,
    "gripper": 0.7277,
}
BOTTLE_START = np.array([0.22, 0.024, 0.0, 1.0, 0.0, 0.0, 0.0])
LEFT_SECTIONS = ["文件", "选项", "仿真", "观察", "物理", "渲染", "可视化", "分组开关", "日志"]
RIGHT_SECTIONS = ["关节", "控制", "等式约束"]


def build_model(imported_paths=(), mesh_scale: float = 0.001):
    spec = mm.get("robotstudio_so101").spec("scene")
    bottle = spec.worldbody.add_body(name="mineral_water_bottle", pos=[0.34, 0.0, 0.0])
    bottle.add_freejoint(name="bottle_free")
    bottle.add_geom(
        name="bottle_body", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        pos=[0, 0, 0.096], size=[0.032, 0.086], mass=0.32,
        rgba=[0.58, 0.82, 0.94, 0.58], friction=[1.2, 0.01, 0.001], condim=3,
    )
    bottle.add_geom(
        name="bottle_label", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        pos=[0, 0, 0.105], size=[0.0325, 0.025], mass=0,
        rgba=[0.92, 0.97, 1.0, 1.0], contype=0, conaffinity=0,
    )
    bottle.add_geom(
        name="bottle_shoulder", type=mujoco.mjtGeom.mjGEOM_ELLIPSOID,
        pos=[0, 0, 0.184], size=[0.031, 0.031, 0.019], mass=0,
        rgba=[0.58, 0.82, 0.94, 0.58], friction=[1.2, 0.01, 0.001], condim=3,
    )
    bottle.add_geom(
        name="bottle_neck", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        pos=[0, 0, 0.216], size=[0.014, 0.018], mass=0,
        rgba=[0.58, 0.82, 0.94, 0.58], friction=[1.2, 0.01, 0.001], condim=3,
    )
    bottle.add_geom(
        name="bottle_cap", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        pos=[0, 0, 0.245], size=[0.017, 0.009], mass=0,
        rgba=[0.08, 0.35, 0.78, 1.0], friction=[1.2, 0.01, 0.001], condim=3,
    )
    for index, path in enumerate(imported_paths):
        attach_object(spec, path, index, mesh_scale)
    spec.visual.global_.offwidth = 1280
    spec.visual.global_.offheight = 800
    return spec.compile()


def main() -> None:
    pygame.init()
    pygame.display.set_caption("mujoco-cn")
    screen = pygame.display.set_mode((1600, 900), pygame.RESIZABLE)
    clock = pygame.time.Clock()
    font_path = Path(r"C:\Windows\Fonts\msyh.ttc")
    font = pygame.font.Font(str(font_path), 18) if font_path.exists() else pygame.font.Font(None, 22)
    small_font = pygame.font.Font(str(font_path), 15) if font_path.exists() else pygame.font.Font(None, 19)
    imported_paths: list[str] = []
    mesh_scale = 0.001
    model = build_model(imported_paths, mesh_scale)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    actuator_ids = {}
    joint_ids = {}
    targets = {}
    for joint in (*JOINT_KEYS.keys(), "gripper"):
        joint_ids[joint] = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        actuator_ids[joint] = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint)
        if joint_ids[joint] < 0 or actuator_ids[joint] < 0:
            raise RuntimeError(f"模型中找不到关节或执行器: {joint}")
        targets[joint] = HOME_POSE[joint]
        qadr = model.jnt_qposadr[joint_ids[joint]]
        data.qpos[qadr] = targets[joint]
        data.ctrl[actuator_ids[joint]] = targets[joint]

    bottle_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "bottle_free")
    bottle_qadr = int(model.jnt_qposadr[bottle_joint])
    bottle_dadr = int(model.jnt_dofadr[bottle_joint])
    data.qpos[bottle_qadr:bottle_qadr + 7] = BOTTLE_START
    mujoco.mj_forward(model, data)
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [0.20, 0.0, 0.12]
    camera.distance = 1.05
    camera.azimuth = 145
    camera.elevation = -24
    renderer = mujoco.Renderer(model, height=800, width=1280)
    visopt = mujoco.MjvOption()
    mujoco.mjv_defaultOption(visopt)
    expanded_left: set[str] = set()
    expanded_right: set[str] = set()
    paused = False
    running = True
    dragging = False
    active_slider: str | None = None
    last_rects: dict = {}

    def reset_bottle() -> None:
        data.qpos[bottle_qadr:bottle_qadr + 7] = BOTTLE_START
        data.qvel[bottle_dadr:bottle_dadr + 6] = 0
        mujoco.mj_forward(model, data)

    def clamp_target(joint: str, value: float) -> float:
        lo, hi = model.actuator_ctrlrange[actuator_ids[joint]]
        return float(np.clip(value, lo, hi))

    def rebuild_scene() -> None:
        nonlocal model, data, actuator_ids, joint_ids, targets
        nonlocal bottle_qadr, bottle_dadr, renderer

        old_targets = dict(targets)
        old_pose = {
            joint: float(data.qpos[model.jnt_qposadr[joint_ids[joint]]])
            for joint in joint_ids
        }
        old_bottle_qpos = data.qpos[bottle_qadr:bottle_qadr + 7].copy()
        old_bottle_qvel = data.qvel[bottle_dadr:bottle_dadr + 6].copy()

        new_model = build_model(imported_paths, mesh_scale)
        new_data = mujoco.MjData(new_model)
        new_joint_ids = {}
        new_actuator_ids = {}
        new_targets = {}
        for joint in (*JOINT_KEYS.keys(), "gripper"):
            jid = mujoco.mj_name2id(new_model, mujoco.mjtObj.mjOBJ_JOINT, joint)
            aid = mujoco.mj_name2id(new_model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint)
            if jid < 0 or aid < 0:
                raise RuntimeError(f"重新加载后找不到关节或执行器: {joint}")
            new_joint_ids[joint] = jid
            new_actuator_ids[joint] = aid
            lo, hi = new_model.actuator_ctrlrange[aid]
            new_targets[joint] = float(np.clip(old_targets.get(joint, 0.0), lo, hi))
            new_data.qpos[new_model.jnt_qposadr[jid]] = old_pose.get(joint, 0.0)
            new_data.ctrl[aid] = new_targets[joint]

        new_bottle_joint = mujoco.mj_name2id(new_model, mujoco.mjtObj.mjOBJ_JOINT, "bottle_free")
        new_bottle_qadr = int(new_model.jnt_qposadr[new_bottle_joint])
        new_bottle_dadr = int(new_model.jnt_dofadr[new_bottle_joint])
        new_data.qpos[new_bottle_qadr:new_bottle_qadr + 7] = old_bottle_qpos
        new_data.qvel[new_bottle_dadr:new_bottle_dadr + 6] = old_bottle_qvel
        mujoco.mj_forward(new_model, new_data)
        new_renderer = mujoco.Renderer(new_model, height=800, width=1280)

        renderer.close()
        model, data = new_model, new_data
        joint_ids, actuator_ids, targets = new_joint_ids, new_actuator_ids, new_targets
        bottle_qadr, bottle_dadr = new_bottle_qadr, new_bottle_dadr
        renderer = new_renderer

    def choose_object_files() -> tuple[list[str], float | None]:
        import tkinter as tk
        from tkinter import filedialog, messagebox, simpledialog

        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        selected = filedialog.askopenfilenames(
            title="选择要导入的对象（可多选）",
            filetypes=[
                ("对象文件", "*.xml *.obj *.stl"),
                ("MuJoCo MJCF", "*.xml"),
                ("三角网格", "*.obj *.stl"),
            ],
        )
        selected = list(selected)
        unsupported = [path for path in selected if Path(path).suffix.lower() not in SUPPORTED_SUFFIXES]
        if unsupported:
            messagebox.showerror("文件格式不支持", "仅支持 MJCF XML、OBJ 和 STL。", parent=root)
            selected = []
        scale = mesh_scale
        if selected and any(Path(path).suffix.lower() in {".obj", ".stl"} for path in selected):
            answer = simpledialog.askfloat(
                "网格单位",
                "OBJ/STL 缩放到米的比例（毫米模型通常填 0.001，米模型填 1）",
                initialvalue=mesh_scale,
                minvalue=1e-7,
                maxvalue=1e6,
                parent=root,
            )
            if answer is None:
                selected = []
            else:
                scale = float(answer)
        root.destroy()
        return selected, scale

    def draw_text(text: str, x: int, y: int, color=(240, 242, 245), face=None) -> None:
        screen.blit((face or font).render(text, True, color), (x, y))

    def draw_button(rect, label: str, key: str) -> None:
        pygame.draw.rect(screen, (78, 100, 119), rect, border_radius=5)
        draw_text(label, rect.x + 8, rect.y + 5, face=small_font)
        last_rects["buttons"][key] = rect

    def draw_section_panel(x: int, width: int, sections: list[str], expanded: set[str], side: str) -> None:
        pygame.draw.rect(screen, (54, 54, 54), (x, 0, width, screen.get_height()))
        pygame.draw.line(screen, (125, 125, 125), (x, 0), (x, screen.get_height()), 1)
        y = 14
        for section in sections:
            header = pygame.Rect(x + 10, y, width - 20, 40)
            for line_y in range(header.height):
                blend = line_y / max(1, header.height - 1)
                red = int(166 - blend * 92)
                green = int(65 - blend * 42)
                blue = int(65 - blend * 42)
                pygame.draw.line(screen, (red, green, blue), (header.x, header.y + line_y), (header.right, header.y + line_y))
            pygame.draw.rect(screen, (33, 33, 33), header, 1, border_radius=7)
            draw_text(section, header.x + 14, header.y + 8)
            arrow = "▼" if section in expanded else "◁"
            draw_text(arrow, header.right - 28, header.y + 8)
            last_rects["headers"][(side, section)] = header
            y += 48
            if section not in expanded:
                continue
            body_height = 0
            if side == "left":
                body_height = {
                    "文件": 82, "选项": 160, "仿真": 48, "观察": 156,
                    "物理": 68, "渲染": 46, "可视化": 68, "分组开关": 68, "日志": 46,
                }.get(section, 44)
            else:
                body_height = {"关节": 156, "控制": 210, "等式约束": 50}.get(section, 44)
            body = pygame.Rect(x + 10, y, width - 20, body_height)
            pygame.draw.rect(screen, (28, 28, 28), body, border_radius=5)
            last_rects["bodies"][(side, section)] = body
            tx, ty = body.x + 10, body.y + 8
            if side == "left":
                if section == "文件":
                    btn1 = pygame.Rect(tx, ty, body.width - 20, 30)
                    btn2 = pygame.Rect(tx, ty + 36, (body.width - 26) // 2, 30)
                    btn3 = pygame.Rect(btn2.right + 6, ty + 36, btn2.width, 30)
                    draw_button(btn1, "导入对象（可多选）", "import_objects")
                    draw_button(btn2, "保存截图", "screenshot")
                    draw_button(btn3, "退出", "exit")
                elif section == "选项":
                    for line in ["W/S：肩部升降　A/D：左右转", "I/K：肘部　J/L：腕部旋转", "左/右 Ctrl：腕部俯仰", "C/B：合爪　V/N：张爪", "空格：切换夹爪　F2：重置瓶子", "鼠标左键拖动：转视角", "滚轮：缩放　Esc：退出"]:
                        draw_text(line, tx, ty, face=small_font)
                        ty += 21
                elif section == "仿真":
                    btn1 = pygame.Rect(tx, ty, (body.width - 26) // 2, 30)
                    btn2 = pygame.Rect(btn1.right + 6, ty, btn1.width, 30)
                    draw_button(btn1, "继续" if paused else "暂停", "run_pause")
                    draw_button(btn2, "重置瓶子", "reset_bottle")
                elif section == "观察":
                    for joint in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"):
                        qpos = float(data.qpos[model.jnt_qposadr[joint_ids[joint]]])
                        draw_text(f"{JOINT_LABELS[joint]}：{math.degrees(qpos):.1f}°", tx, ty, face=small_font)
                        ty += 23
                elif section == "物理":
                    draw_text(f"重力：{model.opt.gravity[2]:.2f} m/s²", tx, ty, face=small_font)
                    draw_text(f"仿真时间：{data.time:.2f} s", tx, ty + 24, face=small_font)
                elif section == "渲染":
                    draw_text("MuJoCo OpenGL 实时视图", tx, ty, face=small_font)
                elif section == "可视化":
                    for label, flag in [("接触点", mujoco.mjtVisFlag.mjVIS_CONTACTPOINT), ("接触力", mujoco.mjtVisFlag.mjVIS_CONTACTFORCE)]:
                        rect = pygame.Rect(tx, ty, 18, 18)
                        pygame.draw.rect(screen, (100, 175, 205) if visopt.flags[flag] else (75, 75, 75), rect)
                        draw_text(label, tx + 28, ty - 1, face=small_font)
                        last_rects["toggles"][label] = rect
                        ty += 28
                elif section == "分组开关":
                    for label, index in [("视觉模型组", 2), ("碰撞体组", 3)]:
                        rect = pygame.Rect(tx, ty, 18, 18)
                        pygame.draw.rect(screen, (100, 175, 205) if visopt.geomgroup[index] else (75, 75, 75), rect)
                        draw_text(label, tx + 28, ty - 1, face=small_font)
                        last_rects["toggles"][label] = rect
                        last_rects["group_indices"][label] = index
                        ty += 28
                elif section == "日志":
                    draw_text("场景已加载", tx, ty, face=small_font)
            elif section == "关节":
                for joint in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"):
                    qpos = float(data.qpos[model.jnt_qposadr[joint_ids[joint]]])
                    draw_text(f"{JOINT_LABELS[joint]}：{math.degrees(qpos):.1f}°", tx, ty, face=small_font)
                    ty += 23
            elif section == "控制":
                for joint in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"):
                    draw_text(JOINT_LABELS[joint], tx, ty, face=small_font)
                    lo, hi = model.actuator_ctrlrange[actuator_ids[joint]]
                    val = targets[joint]
                    track = pygame.Rect(tx + 92, ty + 3, body.width - 114, 14)
                    pygame.draw.rect(screen, (70, 70, 70), track, border_radius=4)
                    ratio = (val - lo) / (hi - lo) if hi > lo else 0.0
                    knob_x = track.x + int(np.clip(ratio, 0.0, 1.0) * track.width)
                    pygame.draw.rect(screen, (114, 168, 196), (track.x, track.y, max(2, knob_x - track.x), track.height), border_radius=4)
                    pygame.draw.circle(screen, (225, 235, 241), (knob_x, track.centery), 7)
                    draw_text(f"{math.degrees(val):.0f}°", tx, ty + 16, face=small_font)
                    last_rects["sliders"][joint] = track
                    ty += 34
            elif section == "等式约束":
                draw_text("此模型没有等式约束", tx, ty, face=small_font)
            y += body_height + 8
        pygame.draw.line(screen, (105, 105, 105), (x + width - 1, 0), (x + width - 1, screen.get_height()), 1)

    def slider_from_x(joint: str, mouse_x: int) -> None:
        track = last_rects["sliders"].get(joint)
        if track is None:
            return
        lo, hi = model.actuator_ctrlrange[actuator_ids[joint]]
        ratio = float(np.clip((mouse_x - track.x) / max(1, track.width), 0.0, 1.0))
        targets[joint] = clamp_target(joint, float(lo + ratio * (hi - lo)))

    try:
        while running:
            dt = min(clock.tick(60) / 1000.0, 0.05)
            width, height = screen.get_size()
            side_width = min(270, max(220, width // 5))
            view_rect = pygame.Rect(side_width, 0, max(1, width - 2 * side_width), height)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_F2:
                        reset_bottle()
                    elif event.key == pygame.K_SPACE:
                        lo, hi = model.actuator_ctrlrange[actuator_ids["gripper"]]
                        targets["gripper"] = float(lo if targets["gripper"] > (lo + hi) / 2 else hi)
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    pos = event.pos
                    header_hit = next((key for key, rect in last_rects["headers"].items() if rect.collidepoint(pos)), None)
                    if header_hit:
                        side, section = header_hit
                        expanded = expanded_left if side == "left" else expanded_right
                        if section in expanded:
                            expanded.remove(section)
                        else:
                            expanded.add(section)
                    else:
                        btn = next((key for key, rect in last_rects["buttons"].items() if rect.collidepoint(pos)), None)
                        if btn == "exit":
                            running = False
                        elif btn == "import_objects":
                            selected, selected_scale = choose_object_files()
                            if selected:
                                previous_count = len(imported_paths)
                                old_scale = mesh_scale
                                imported_paths.extend(selected)
                                if selected_scale is not None:
                                    mesh_scale = selected_scale
                                try:
                                    rebuild_scene()
                                except Exception as exc:
                                    del imported_paths[previous_count:]
                                    mesh_scale = old_scale
                                    print(f"导入对象失败: {exc}")
                                    import tkinter as tk
                                    from tkinter import messagebox
                                    error_root = tk.Tk()
                                    error_root.withdraw()
                                    messagebox.showerror("导入失败", str(exc), parent=error_root)
                                    error_root.destroy()
                        elif btn == "screenshot":
                            out_dir = repo_root / "data" / "mujoco_menagerie_demo"
                            out_dir.mkdir(parents=True, exist_ok=True)
                            pygame.image.save(screen, out_dir / "screenshot.png")
                        elif btn == "run_pause":
                            paused = not paused
                        elif btn == "reset_bottle":
                            reset_bottle()
                        else:
                            toggle = next((key for key, rect in last_rects["toggles"].items() if rect.collidepoint(pos)), None)
                            if toggle in ("接触点", "接触力"):
                                flag = mujoco.mjtVisFlag.mjVIS_CONTACTPOINT if toggle == "接触点" else mujoco.mjtVisFlag.mjVIS_CONTACTFORCE
                                visopt.flags[flag] = 1 - visopt.flags[flag]
                            elif toggle in ("视觉模型组", "碰撞体组"):
                                idx = last_rects["group_indices"][toggle]
                                visopt.geomgroup[idx] = 1 - visopt.geomgroup[idx]
                            else:
                                slider = next((key for key, rect in last_rects["sliders"].items() if rect.inflate(0, 14).collidepoint(pos)), None)
                                if slider:
                                    active_slider = slider
                                    slider_from_x(slider, pos[0])
                                elif view_rect.collidepoint(pos):
                                    dragging = True
                elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                    dragging = False
                    active_slider = None
                elif event.type == pygame.MOUSEMOTION:
                    if active_slider:
                        slider_from_x(active_slider, event.pos[0])
                    elif dragging:
                        camera.azimuth = (camera.azimuth - event.rel[0] * 0.35) % 360
                        camera.elevation = float(np.clip(camera.elevation + event.rel[1] * 0.3, -80, -5))
                elif event.type == pygame.MOUSEWHEEL and view_rect.collidepoint(pygame.mouse.get_pos()):
                    camera.distance = float(np.clip(camera.distance * math.exp(-event.y * 0.1), 0.5, 2.5))

            keys = pygame.key.get_pressed()
            for joint, (negative_key, positive_key) in JOINT_KEYS.items():
                direction = int(bool(keys[positive_key])) - int(bool(keys[negative_key]))
                targets[joint] = clamp_target(joint, targets[joint] + direction * JOINT_RATE * dt)
            close = bool(keys[pygame.K_c] or keys[pygame.K_b])
            open_ = bool(keys[pygame.K_v] or keys[pygame.K_n])
            gripper_direction = int(open_) - int(close)
            targets["gripper"] = clamp_target(
                "gripper", targets["gripper"] + gripper_direction * GRIPPER_RATE * dt
            )
            for joint, actuator_id in actuator_ids.items():
                data.ctrl[actuator_id] = targets[joint]
            if not paused:
                for _ in range(max(1, round(dt / model.opt.timestep))):
                    mujoco.mj_step(model, data)

            renderer.update_scene(data, camera=camera, scene_option=visopt)
            frame = renderer.render()
            image = pygame.surfarray.make_surface(frame.swapaxes(0, 1))
            screen.fill((20, 25, 30))
            scale = min(view_rect.width / image.get_width(), view_rect.height / image.get_height())
            size = (max(1, int(image.get_width() * scale)), max(1, int(image.get_height() * scale)))
            image = pygame.transform.smoothscale(image, size)
            image_rect = image.get_rect(center=view_rect.center)
            screen.blit(image, image_rect)
            last_rects = {"headers": {}, "bodies": {}, "buttons": {}, "toggles": {}, "group_indices": {}, "sliders": {}}
            draw_section_panel(0, side_width, LEFT_SECTIONS, expanded_left, "left")
            draw_section_panel(width - side_width, side_width, RIGHT_SECTIONS, expanded_right, "right")
            pygame.display.flip()
    finally:
        renderer.close()
        pygame.quit()


if __name__ == "__main__":
    main()

