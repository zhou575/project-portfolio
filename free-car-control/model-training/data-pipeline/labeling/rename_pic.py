import argparse
import shutil
from pathlib import Path


DEFAULT_INPUT_DIR = Path("unnamed_picture")
DEFAULT_OUTPUT_DIR = Path("renamed_picture")
DEFAULT_CLASSES = ["weapon", "supplies", "vehicle"]
IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".webp"}


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=DEFAULT_INPUT_DIR,
        help="Input folder containing class subfolders.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Output folder for renamed class images.",
    )
    parser.add_argument(
        "--classes",
        nargs="+",
        default=DEFAULT_CLASSES,
        help="Class folder names to process.",
    )
    return parser.parse_args()


def clear_folder(folder: Path):
    if folder.exists():
        for item in folder.iterdir():
            if item.is_file() or item.is_symlink():
                item.unlink()
            elif item.is_dir():
                shutil.rmtree(item)
    else:
        folder.mkdir(parents=True, exist_ok=True)


def collect_images(root: Path):
    images = [p for p in root.rglob("*") if p.is_file() and p.suffix.lower() in IMAGE_EXTS]
    images.sort(key=lambda path: str(path).lower())
    return images


def rename_one_class(class_name: str, input_dir: Path, output_dir: Path):
    src_dir = input_dir / class_name
    dst_dir = output_dir / class_name

    if not src_dir.exists() or not src_dir.is_dir():
        print(f"[skip] folder not found: {src_dir.resolve()}")
        return 0

    dst_dir.mkdir(parents=True, exist_ok=True)

    image_files = collect_images(src_dir)
    if not image_files:
        print(f"[skip] no images found: {src_dir.resolve()}")
        return 0

    print(f"\nProcessing class: {class_name}")
    print(f"Source: {src_dir.resolve()}")
    print(f"Output: {dst_dir.resolve()}")
    print(f"Images: {len(image_files)}")

    for idx, src_path in enumerate(image_files, start=1):
        ext = src_path.suffix.lower()
        dst_name = f"{class_name}_{idx}{ext}"
        shutil.copy2(src_path, dst_dir / dst_name)

    print(f"Done: {class_name} -> {len(image_files)} images")
    return len(image_files)


def main():
    args = parse_args()
    input_dir = args.input_dir
    output_dir = args.output_dir
    class_names = args.classes

    if not input_dir.exists() or not input_dir.is_dir():
        raise FileNotFoundError(f"Input folder not found: {input_dir.resolve()}")

    clear_folder(output_dir)

    total_count = 0
    for class_name in class_names:
        total_count += rename_one_class(class_name, input_dir, output_dir)

    print("\n====================")
    print("All classes processed")
    print(f"Total images: {total_count}")
    print(f"Output root: {output_dir.resolve()}")


if __name__ == "__main__":
    main()
