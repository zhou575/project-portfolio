import argparse
import shutil
from pathlib import Path

import cv2


IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".webp"}
CLASS_KEYS = {
    ord("1"): "weapon",
    ord("2"): "supplies",
    ord("3"): "vehicle",
}
WINDOW_NAME = "dataset-classifier"


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source-root",
        type=Path,
        default=Path("../video_to_pic/raw_picture"),
        help="Folder containing extracted frames grouped by video.",
    )
    parser.add_argument(
        "--target-root",
        type=Path,
        default=Path("unnamed_picture"),
        help="Folder containing class subfolders such as weapon/supplies/vehicle.",
    )
    parser.add_argument(
        "--max-width",
        type=int,
        default=1600,
        help="Maximum preview width.",
    )
    parser.add_argument(
        "--max-height",
        type=int,
        default=900,
        help="Maximum preview height.",
    )
    return parser.parse_args()


def collect_images(root: Path):
    images = [p for p in root.rglob("*") if p.is_file() and p.suffix.lower() in IMAGE_EXTS]
    images.sort(key=lambda p: str(p).lower())
    return images


def ensure_target_dirs(target_root: Path):
    for class_name in CLASS_KEYS.values():
        (target_root / class_name).mkdir(parents=True, exist_ok=True)


def collect_existing_labels(target_root: Path):
    labeled = set()
    for class_name in CLASS_KEYS.values():
        class_dir = target_root / class_name
        if not class_dir.exists():
            continue
        for image_path in class_dir.rglob("*"):
            if image_path.is_file() and image_path.suffix.lower() in IMAGE_EXTS:
                labeled.add(image_path.name.lower())
    return labeled


def fit_image(image, max_width: int, max_height: int):
    height, width = image.shape[:2]
    scale = min(max_width / width, max_height / height, 1.0)
    if scale == 1.0:
        return image
    new_size = (max(1, int(width * scale)), max(1, int(height * scale)))
    return cv2.resize(image, new_size, interpolation=cv2.INTER_AREA)


def draw_overlay(image, rel_path: str, index: int, total: int):
    overlay = image.copy()
    lines = [
        f"{index + 1}/{total}  {rel_path}",
        "1 weapon   2 supplies   3 vehicle",
        "space skip   b back   q quit",
    ]

    y = 30
    for line in lines:
        cv2.putText(
            overlay,
            line,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            overlay,
            line,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (20, 20, 20),
            4,
            cv2.LINE_AA,
        )
        y += 32
    return overlay


def unique_destination(dst_dir: Path, src_path: Path):
    candidate = dst_dir / src_path.name
    if not candidate.exists():
        return candidate

    stem = src_path.stem
    suffix = src_path.suffix.lower()
    counter = 1
    while True:
        candidate = dst_dir / f"{stem}_{counter}{suffix}"
        if not candidate.exists():
            return candidate
        counter += 1


def show_image(image_path: Path, source_root: Path, index: int, total: int, max_width: int, max_height: int):
    image = cv2.imread(str(image_path))
    if image is None:
        raise RuntimeError(f"Failed to read image: {image_path}")
    image = fit_image(image, max_width=max_width, max_height=max_height)
    rel_path = str(image_path.relative_to(source_root))
    image = draw_overlay(image, rel_path=rel_path, index=index, total=total)
    cv2.imshow(WINDOW_NAME, image)


def main():
    args = parse_args()
    source_root = args.source_root.resolve()
    target_root = args.target_root.resolve()

    if not source_root.exists() or not source_root.is_dir():
        raise FileNotFoundError(f"source-root not found: {source_root}")

    ensure_target_dirs(target_root)
    all_images = collect_images(source_root)
    if not all_images:
        raise RuntimeError(f"No images found under: {source_root}")

    existing_labels = collect_existing_labels(target_root)
    pending_images = [p for p in all_images if p.name.lower() not in existing_labels]

    if not pending_images:
        print("All images are already labeled.")
        return

    print(f"Source root: {source_root}")
    print(f"Target root: {target_root}")
    print(f"Total images: {len(all_images)}")
    print(f"Already labeled: {len(all_images) - len(pending_images)}")
    print(f"Pending images: {len(pending_images)}")

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    index = 0
    history = []

    while 0 <= index < len(pending_images):
        image_path = pending_images[index]
        show_image(
            image_path=image_path,
            source_root=source_root,
            index=index,
            total=len(pending_images),
            max_width=args.max_width,
            max_height=args.max_height,
        )

        key = cv2.waitKeyEx(0)

        if key in CLASS_KEYS:
            class_name = CLASS_KEYS[key]
            dst_dir = target_root / class_name
            dst_path = unique_destination(dst_dir, image_path)
            shutil.copy2(image_path, dst_path)
            history.append((index, dst_path))
            print(f"[{index + 1}/{len(pending_images)}] {image_path.name} -> {class_name}")
            index += 1
            continue

        if key == ord(" "):
            print(f"[{index + 1}/{len(pending_images)}] skip {image_path.name}")
            index += 1
            continue

        if key in (ord("b"), ord("B")):
            if not history:
                print("No labeled image to undo.")
                continue
            last_index, last_dst = history.pop()
            if last_dst.exists():
                last_dst.unlink()
            index = last_index
            print(f"Undo -> {pending_images[index].name}")
            continue

        if key in (ord("q"), ord("Q"), 27):
            print("Quit requested.")
            break

    cv2.destroyAllWindows()

    labeled_count = len(collect_existing_labels(target_root))
    print("====================")
    print(f"Labeled files now: {labeled_count}")
    print(f"Target root: {target_root}")


if __name__ == "__main__":
    main()
