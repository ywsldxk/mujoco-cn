"""Import independent MJCF object files and triangle meshes into a scene."""

from __future__ import annotations

from pathlib import Path
from xml.sax.saxutils import quoteattr

import mujoco


SUPPORTED_SUFFIXES = {".xml", ".obj", ".stl"}


def attach_object(scene: mujoco.MjSpec, path: str | Path, index: int, mesh_scale: float) -> None:
    """Attach one MJCF object or OBJ/STL mesh as a movable world object.

    MJCF input should describe an object body, not a complete environment. Mesh
    files are wrapped in a free body and use the mesh as their collision hull.
    """
    source = Path(path).expanduser().resolve()
    suffix = source.suffix.lower()
    if suffix not in SUPPORTED_SUFFIXES:
        raise ValueError(f"不支持的对象格式: {source.suffix}")

    if suffix == ".xml":
        child = mujoco.MjSpec.from_file(str(source))
    else:
        meshdir = quoteattr(source.parent.as_posix())
        filename = quoteattr(source.name)
        safe_name = f"import_mesh_{index}"
        xml = (
            f'<mujoco model="imported_object_{index}">'
            f'<compiler meshdir={meshdir}/>'
            f'<asset><mesh name="{safe_name}" file={filename} '
            f'scale="{mesh_scale} {mesh_scale} {mesh_scale}"/></asset>'
            '<worldbody><body name="object"><freejoint/>'
            f'<geom name="collision" type="mesh" mesh="{safe_name}" '
            'mass="0.25" condim="3" friction="1.0 0.01 0.001"/>'
            '</body></worldbody></mujoco>'
        )
        child = mujoco.MjSpec.from_string(xml)

    # Spread imported objects in a compact grid in front of the arm.
    column = index % 3
    row = index // 3
    frame = scene.worldbody.add_frame(
        name=f"import_slot_{index}",
        pos=[0.38 + 0.14 * column, 0.16 + 0.14 * row, 0.0],
    )
    scene.attach(child, prefix=f"imported_{index}_", frame=frame)
