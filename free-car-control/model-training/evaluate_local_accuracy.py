import argparse
from pathlib import Path

import torch
import torch.nn as nn
from torch.utils.data import DataLoader, Subset
from torchvision import datasets, transforms

from train_tiny_classifier import TinyClassifier, build_splits, set_seed


def resolve_input_shape(args, checkpoint):
    if args.shape_mode == "auto":
        ckpt_w = checkpoint.get("img_width")
        ckpt_h = checkpoint.get("img_height")
        if ckpt_w and ckpt_h:
            return int(ckpt_w), int(ckpt_h), "checkpoint"

        ckpt_size = checkpoint.get("img_size")
        if ckpt_size:
            size = int(ckpt_size)
            return size, size, "checkpoint"

        # Fallback to CLI defaults if checkpoint does not contain shape info.
        return args.img_size, args.img_size, "cli-fallback"

    if args.shape_mode == "square":
        return args.img_size, args.img_size, "cli-square"

    if args.shape_mode == "rect":
        if args.img_width <= 0 or args.img_height <= 0:
            raise ValueError(f"Invalid rect shape: {args.img_width}x{args.img_height}")
        return args.img_width, args.img_height, "cli-rect"

    raise ValueError(f"Unknown shape-mode: {args.shape_mode}")


def evaluate(model: nn.Module, loader: DataLoader, device: torch.device):
    model.eval()
    total = 0
    correct = 0
    per_class_total = {}
    per_class_correct = {}

    with torch.no_grad():
        for images, labels in loader:
            images = images.to(device)
            labels = labels.to(device)
            logits = model(images)
            preds = logits.argmax(dim=1)

            total += labels.size(0)
            correct += (preds == labels).sum().item()

            for label, pred in zip(labels.tolist(), preds.tolist()):
                per_class_total[label] = per_class_total.get(label, 0) + 1
                if label == pred:
                    per_class_correct[label] = per_class_correct.get(label, 0) + 1

    acc = correct / max(1, total)
    return acc, per_class_correct, per_class_total


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--data-root",
        type=str,
        default="dataset",
        help="Dataset root folder (ImageFolder structure), e.g. dataset or D:/my_data/cls_dataset",
    )
    parser.add_argument(
        "--test-root",
        type=str,
        default="",
        help="Optional independent test dataset root. When set, test accuracy is computed on this dataset directly.",
    )
    parser.add_argument("--checkpoint", type=str, default="artifacts/best_model.pt")
    parser.add_argument(
        "--shape-mode",
        type=str,
        default="auto",
        choices=["auto", "square", "rect"],
        help="auto: use checkpoint shape; square: use --img-size; rect: use --img-width/--img-height.",
    )
    parser.add_argument("--img-size", type=int, default=96)
    parser.add_argument("--img-width", type=int, default=96)
    parser.add_argument("--img-height", type=int, default=96)
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--num-workers", type=int, default=2)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--split-mode",
        type=str,
        default="random",
        choices=["random", "grouped"],
        help="Split strategy used to build test split.",
    )
    args = parser.parse_args()

    set_seed(args.seed)
    device = torch.device("cpu")

    data_root = Path(args.data_root)
    if not data_root.exists() or not data_root.is_dir():
        raise FileNotFoundError(f"data-root not found or not a directory: {data_root}")
    test_root = Path(args.test_root) if args.test_root else None
    if test_root is not None and (not test_root.exists() or not test_root.is_dir()):
        raise FileNotFoundError(f"test-root not found or not a directory: {test_root}")

    checkpoint = torch.load(args.checkpoint, map_location="cpu")
    classes = checkpoint["classes"]
    width_mult = checkpoint.get("width_mult", 0.75)
    img_width, img_height, shape_source = resolve_input_shape(args, checkpoint)
    print(f"Eval input shape: {img_width}x{img_height} (source={shape_source})")

    model = TinyClassifier(num_classes=len(classes), width_mult=width_mult)
    model.load_state_dict(checkpoint["model_state"])
    model.to(device)

    eval_tf = transforms.Compose([
        transforms.Resize((img_height, img_width)),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ])

    dataset = datasets.ImageFolder(str(data_root), transform=eval_tf)
    if dataset.classes != classes:
        raise RuntimeError(f"data-root classes mismatch: checkpoint={classes}, dataset={dataset.classes}")

    if test_root is None:
        _, _, test_indices = build_splits(
            dataset,
            args.seed,
            train_ratio=0.8,
            val_ratio=0.1,
            split_mode=args.split_mode,
        )
        test_dataset = Subset(dataset, test_indices)
        test_count = len(test_indices)
        test_label = "Test split"
    else:
        test_dataset_full = datasets.ImageFolder(str(test_root), transform=eval_tf)
        if test_dataset_full.classes != classes:
            raise RuntimeError(
                f"test-root classes mismatch: checkpoint={classes}, test={test_dataset_full.classes}"
            )
        test_dataset = test_dataset_full
        test_count = len(test_dataset_full)
        test_label = "Independent test"

    test_loader = DataLoader(
        test_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=False,
    )
    full_loader = DataLoader(
        dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=False,
    )

    test_acc, test_per_correct, test_per_total = evaluate(model, test_loader, device)
    full_acc, full_per_correct, full_per_total = evaluate(model, full_loader, device)

    print(f"{test_label} accuracy: {test_acc:.4f} ({test_count} images)")
    for idx, name in enumerate(classes):
        c = test_per_correct.get(idx, 0)
        t = test_per_total.get(idx, 0)
        print(f"  [Test] {name}: {c}/{t} = {c / max(1, t):.4f}")

    print(f"Full dataset accuracy: {full_acc:.4f} ({len(dataset)} images)")
    for idx, name in enumerate(classes):
        c = full_per_correct.get(idx, 0)
        t = full_per_total.get(idx, 0)
        print(f"  [Full] {name}: {c}/{t} = {c / max(1, t):.4f}")


if __name__ == "__main__":
    main()
