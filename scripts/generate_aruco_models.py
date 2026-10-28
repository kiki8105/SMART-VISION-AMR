#!/usr/bin/python3
"""Generate exact OpenCV ArUco images and Gazebo Classic insertable models."""

import argparse
import json
import math
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

import cv2


REPO_ROOT = Path(__file__).resolve().parents[1]


def model_name(dictionary, marker_id):
    return f"smart_vision_aruco_{dictionary.removeprefix('DICT_').lower()}_id_{marker_id}"


def element(parent, tag, text=None, **attributes):
    child = ET.SubElement(parent, tag, attributes)
    if text is not None:
        child.text = str(text)
    return child


def save_xml(root, path):
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def create_model(destination, dictionary_name, dictionary, marker_id, size, margin, pixels):
    """Write a white-backed board whose printed face points toward local -X."""
    marker = (cv2.aruco.generateImageMarker(dictionary, marker_id, pixels)
              if hasattr(cv2.aruco, "generateImageMarker")
              else cv2.aruco.drawMarker(dictionary, marker_id, pixels))
    margin_pixels = round(pixels * margin / size)
    if margin_pixels < 1:
        raise ValueError("white margin must occupy at least one texture pixel")
    # Quantize the physical white margin to the same scale as the image pixels.
    actual_margin = margin_pixels * size / pixels
    board_size = size + 2 * actual_margin
    texture = cv2.copyMakeBorder(marker, margin_pixels, margin_pixels,
                                margin_pixels, margin_pixels,
                                cv2.BORDER_CONSTANT, value=255)
    if hasattr(cv2.aruco, "ArucoDetector"):
        _, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(texture)
    else:
        _, ids, _ = cv2.aruco.detectMarkers(texture, dictionary)
    if ids is None or ids.flatten().tolist() != [marker_id]:
        raise RuntimeError(f"Generated image failed independent marker detection: {marker_id}")

    scripts = destination / "materials/scripts"
    textures = destination / "materials/textures"
    meshes = destination / "meshes"
    for folder in (scripts, textures, meshes):
        folder.mkdir(parents=True)
    name = destination.name
    texture_filename = name + ".png"
    if not cv2.imwrite(str(textures / texture_filename), texture):
        raise RuntimeError("Could not write marker texture")
    material = f"SmartVision/{name}"
    (scripts / "marker.material").write_text(
        f"""material {material}
{{
  technique
  {{
    pass
    {{
      lighting off
      texture_unit
      {{
        texture {texture_filename}
        tex_address_mode clamp
        filtering none
      }}
    }}
  }}
}}
""", encoding="utf-8")
    half = board_size / 2
    # From local -X, +Y is image-left and +Z is image-up. Normal/winding agree.
    (meshes / "marker.obj").write_text(
        f"""# Front face: local -X. UVs cover one complete marker image.
v 0 {half:.12g} {-half:.12g}
v 0 {-half:.12g} {-half:.12g}
v 0 {-half:.12g} {half:.12g}
v 0 {half:.12g} {half:.12g}
vt 0 0
vt 1 0
vt 1 1
vt 0 1
vn -1 0 0
f 1/1/1 2/2/1 3/3/1
f 1/1/1 3/3/1 4/4/1
""", encoding="utf-8")

    config = ET.Element("model")
    element(config, "name", f"SMART VISION ArUco {dictionary_name.removeprefix('DICT_')} ID {marker_id}")
    element(config, "version", "1.0")
    element(config, "sdf", "model.sdf", version="1.6")
    element(config, "description", f"{dictionary_name}, ID={marker_id}, black marker edge={size} m; front=-X")
    save_xml(config, destination / "model.config")

    sdf = ET.Element("sdf", version="1.6")
    model = element(sdf, "model", name=name)
    element(model, "static", "true")
    # Insert defaults put the board just above the floor; spawn -z overrides it.
    element(model, "pose", "0 0 0.20 0 0 0")
    link = element(model, "link", name="board")
    thickness = 0.004
    for kind in ("collision", "visual"):
        body = element(link, kind, name=f"board_{kind}")
        geometry = element(body, "geometry")
        box = element(geometry, "box")
        element(box, "size", f"{thickness} {board_size:.12g} {board_size:.12g}")
        if kind == "visual":
            white = element(body, "material")
            element(white, "ambient", "1 1 1 1")
            element(white, "diffuse", "1 1 1 1")
            element(body, "cast_shadows", "false")
    printed = element(link, "visual", name="printed_front")
    element(printed, "pose", f"{-thickness/2 - .0001} 0 0 0 0 0")
    geometry = element(printed, "geometry")
    mesh = element(geometry, "mesh")
    element(mesh, "uri", f"model://{name}/meshes/marker.obj")
    skin = element(printed, "material")
    script = element(skin, "script")
    element(script, "uri", f"model://{name}/materials/scripts")
    element(script, "uri", f"model://{name}/materials/textures")
    element(script, "name", material)
    element(skin, "lighting", "false")
    element(printed, "cast_shadows", "false")
    save_xml(sdf, destination / "model.sdf")
    (destination / "marker.json").write_text(json.dumps({
        "dictionary": dictionary_name, "id": marker_id,
        "marker_length_m": size, "white_margin_m": actual_margin,
        "board_length_m": board_size, "marker_pixels": pixels,
        "texture_pixels": int(texture.shape[0]), "border_bits": 1,
        "texture_file": "materials/textures/" + texture_filename,
        "front_normal": [-1, 0, 0], "pose_estimation_length_excludes_white_margin": True,
        "generator_reference": "https://chev.me/arucogen/",
    }, indent=2) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ids", type=int, nargs="+", default=[0])
    parser.add_argument("--dictionary", default="DICT_4X4_50")
    parser.add_argument("--marker-size", type=float, default=.20,
                        help="Black bordered square edge in metres; excludes white margin")
    parser.add_argument("--margin", type=float, default=.04,
                        help="White margin on each side in metres")
    parser.add_argument("--pixels", type=int, default=600)
    parser.add_argument("--output-dir", type=Path, default=REPO_ROOT / "gazebo_models")
    parser.add_argument("--no-register", action="store_true",
                        help="Do not add symlinks to ~/.gazebo/models")
    args = parser.parse_args()
    if not hasattr(cv2, "aruco"):
        parser.error("cv2.aruco is missing; install python3-opencv on Ubuntu")
    if not args.dictionary.startswith("DICT_") or not hasattr(cv2.aruco, args.dictionary):
        parser.error("Unknown OpenCV predefined dictionary")
    for name, value in (("marker-size", args.marker_size), ("margin", args.margin)):
        if not math.isfinite(value) or value <= 0:
            parser.error(f"--{name} must be finite and positive")
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dictionary))
    if args.pixels < (dictionary.markerSize + 2) * 8:
        parser.error("--pixels is too small for a readable marker")
    if args.pixels % (dictionary.markerSize + 2):
        parser.error("--pixels must be divisible by dictionary marker bits + 2 border bits")
    ids = list(dict.fromkeys(args.ids))
    if any(marker_id < 0 or marker_id >= len(dictionary.bytesList) for marker_id in ids):
        parser.error(f"IDs must be between 0 and {len(dictionary.bytesList)-1}")
    output = args.output_dir.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    registration = Path.home() / ".gazebo/models"
    for marker_id in ids:
        name = model_name(args.dictionary, marker_id)
        target = output / name
        registered = registration / name
        if not args.no_register and (registered.exists() or registered.is_symlink()):
            if registered.resolve() != target:
                parser.error(f"Gazebo model name already belongs to another path: {registered}")
        # Generate in isolation; never partially replace an existing model.
        with tempfile.TemporaryDirectory(prefix="aruco-model-") as temporary:
            created = Path(temporary) / name
            create_model(created, args.dictionary, dictionary, marker_id,
                         args.marker_size, args.margin, args.pixels)
            if target.exists():
                files = sorted(p.relative_to(created) for p in created.rglob("*") if p.is_file())
                if any(not (target / p).is_file() or (target / p).read_bytes() != (created / p).read_bytes()
                       for p in files):
                    parser.error(f"Existing model differs; choose another --output-dir: {target}")
            else:
                shutil.copytree(created, target)
        if not args.no_register:
            registration.mkdir(parents=True, exist_ok=True)
            if not registered.is_symlink():
                registered.symlink_to(target, target_is_directory=True)
        print(f"MODEL={target}\nMARKER_ID={marker_id}\nMARKER_LENGTH_M={args.marker_size}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
