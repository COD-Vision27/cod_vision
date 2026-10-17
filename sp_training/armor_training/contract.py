"""Single source of truth for data labels and the existing C++ decoder ABI."""
import hashlib
import json
from pathlib import Path
import numpy as np

COLORS = ("blue", "red", "gray", "purple")
NUMBERS = ("sentry", "one", "two", "three", "four", "five", "outpost", "base_small", "base_big")
POINT_ORDER = ("top_left", "bottom_left", "bottom_right", "top_right")
SCHEMA = "sp-armor-v5-v1"
ANCHORS = (((10,13),(16,30),(33,23)),((30,61),(62,45),(59,119)),((116,90),(156,198),(373,326)))
STRIDES = (8,16,32)


def split_for(scene_id, seed=2027):
    value = int(hashlib.sha256(f"{seed}:{scene_id}".encode()).hexdigest()[:12], 16) % 100
    return "train" if value < 80 else "val" if value < 90 else "test"


def validate_object(obj):
    color, number = obj["color_id"], obj["number_id"]
    if type(color) is not int or color not in range(4) or type(number) is not int or number not in range(9):
        raise ValueError("color_id must be 0..3 and number_id 0..8")
    points = np.asarray(obj["points"], dtype=np.float64)
    if points.shape != (4, 2) or not np.isfinite(points).all() or (points < 0).any() or (points > 1).any():
        raise ValueError("four finite normalized points required, in TL,BL,BR,TR order")
    edges = np.roll(points, -1, axis=0) - points
    following = np.roll(edges, -1, axis=0)
    cross = edges[:, 0] * following[:, 1] - edges[:, 1] * following[:, 0]
    if not (cross < -1e-9).all():
        raise ValueError("points must form a convex counterclockwise image polygon: TL,BL,BR,TR")
    if len(obj.get("visible", [True] * 4)) != 4 or not all(obj.get("visible", [True] * 4)):
        raise ValueError("v5 training requires four visible points; reject the whole ambiguous frame")
    return points


def label_line(obj):
    p = validate_object(obj)
    low, high = p.min(0), p.max(0)
    values = [obj["color_id"] * 9 + obj["number_id"], *((low + high) / 2), *(high - low), *p.flatten()]
    return " ".join([str(values[0]), *(f"{float(x):.8f}" for x in values[1:])])


def parse_label(line):
    values = np.asarray([float(x) for x in line.split()], dtype=float)
    if values.shape != (13,) or not np.isfinite(values).all():
        raise ValueError("label must contain class, cx cy w h, and 4 xy points (13 values)")
    cls = int(values[0])
    if cls != values[0] or cls not in range(36):
        raise ValueError("class must be an integer in 0..35")
    obj = {"color_id": cls // 9, "number_id": cls % 9, "points": values[5:].reshape(4, 2).tolist()}
    p = validate_object(obj)
    low, high = p.min(0), p.max(0)
    if not np.allclose(values[1:5], np.r_[(low + high) / 2, high - low], atol=2e-7):
        raise ValueError("bbox must match keypoint extent")
    return obj


def write_sample(root, sample_id, scene_id, image, objects, provenance, seed=2027):
    import cv2
    root = Path(root)
    if not sample_id or any(x not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for x in sample_id):
        raise ValueError("sample_id must be an ASCII filename stem")
    for obj in objects:
        validate_object(obj)
    split = split_for(scene_id, seed)
    paths = [root / folder / split / f"{sample_id}.{ext}" for folder, ext in (("images", "png"), ("labels", "txt"), ("meta", "json"))]
    if any(p.exists() for p in paths) or any((root / "meta").glob(f"*/{sample_id}.json")):
        raise FileExistsError(f"refusing to overwrite sample {sample_id}")
    for p in paths:
        p.parent.mkdir(parents=True, exist_ok=True)
    record = {"schema": SCHEMA, "sample_id": sample_id, "scene_id": scene_id, "split_seed": seed, "split": split,
              "image": paths[0].relative_to(root).as_posix(), "label": paths[1].relative_to(root).as_posix(),
              "width": image.shape[1], "height": image.shape[0], "objects": objects, "provenance": provenance}
    # Metadata is the commit marker; the indexer only sees complete samples.
    try:
        if not cv2.imwrite(str(paths[0]), image):
            raise OSError("PNG write failed")
        paths[1].write_text("\n".join(label_line(o) for o in objects) + ("\n" if objects else ""))
        record["image_sha256"] = hashlib.sha256(paths[0].read_bytes()).hexdigest()
        paths[2].write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n")
    except Exception:
        for p in paths:
            p.unlink(missing_ok=True)
        raise
    return record


def decode_output(rows, scale=1.0, offset=(0, 0), threshold=0.5):
    """Python counterpart of YOLOV5::parse (before NMS/name/type filtering)."""
    rows = np.asarray(rows)
    if rows.ndim != 2 or rows.shape[1] != 22 or not np.isfinite(rows).all() or scale <= 0:
        raise ValueError("expected finite N x 22 output and positive scale")
    result = []
    for row in rows:
        score = float(1 / (1 + np.exp(-np.clip(row[8], -80, 80))))
        if score < threshold:
            continue
        result.append({"points": (row[:8].reshape(4, 2)[[0, 3, 2, 1]] / scale + offset).tolist(),
                       "color_id": int(row[9:13].argmax()), "number_id": int(row[13:22].argmax()), "confidence": score})
    return result
