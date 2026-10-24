import argparse
import json
import random
from pathlib import Path

import torch
from torch.utils.data import DataLoader, Subset
from torchvision import datasets, transforms

from train_tiny_classifier import TinyClassifier, set_seed


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


def evaluate(model: torch.nn.Module, loader: DataLoader, device: torch.device):
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

    return correct / max(1, total), per_class_correct, per_class_total, total


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=str, default="dataset")
    parser.add_argument(
        "--test-root",
        type=str,
        default="",
        help="Optional independent test dataset root. When set, random sampling uses this dataset instead of --data-root.",
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
    parser.add_argument("--num-workers", type=int, default=0)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--sample-size", type=int, default=300)
    parser.add_argument("--out-json", type=str, default="artifacts/random_sample_metrics.json")
    args = parser.parse_args()

    set_seed(args.seed)
    random.seed(args.seed)
    device = torch.device("cpu")

    data_root = Path(args.data_root)
    if not data_root.exists() or not data_root.is_dir():
        raise FileNotFoundError(f"data-root not found or not a directory: {data_root}")
    test_root = Path(args.test_root) if args.test_root else None
    if test_root is not None and (not test_root.exists() or not test_root.is_dir()):
        raise FileNotFoundError(f"test-root not found or not a directory: {test_root}")

    checkpoint = torch.load(args.checkpoint, map_location="cpu")
    classes = checkpoint["classes"]
    width_mult = checkpoint.get("width_mult", 0.6)
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

    dataset_root = test_root if test_root is not None else data_root
    dataset = datasets.ImageFolder(str(dataset_root), transform=eval_tf)
    if dataset.classes != classes:
        raise RuntimeError(f"dataset classes mismatch: checkpoint={classes}, dataset={dataset.classes}")
    all_indices = list(range(len(dataset)))
    random.shuffle(all_indices)

    sample_size = min(args.sample_size, len(all_indices))
    selected = all_indices[:sample_size]

    loader = DataLoader(
        Subset(dataset, selected),
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=False,
    )

    acc, per_correct, per_total, total = evaluate(model, loader, device)

    print(f"Random sample accuracy: {acc:.4f} ({total} images)")
    for idx, name in enumerate(classes):
        c = per_correct.get(idx, 0)
        t = per_total.get(idx, 0)
        print(f"  {name}: {c}/{t} = {c / max(1, t):.4f}")

    out_data = {
        "sample_size": total,
        "seed": args.seed,
        "accuracy": acc,
        "per_class": {
            name: {
                "correct": per_correct.get(idx, 0),
                "total": per_total.get(idx, 0),
                "accuracy": per_correct.get(idx, 0) / max(1, per_total.get(idx, 0)),
            }
            for idx, name in enumerate(classes)
        },
    }

    out_json = Path(args.out_json)
    out_json.parent.mkdir(parents=True, exist_ok=True)
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(out_data, f, ensure_ascii=False, indent=2)

    print(f"Saved: {out_json}")

if __name__ == "__main__":
    main()
