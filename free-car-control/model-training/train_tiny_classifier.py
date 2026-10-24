# SPDX-License-Identifier: GPL-3.0-or-later

import argparse
import json
import random
import re
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from onnxsim import simplify
from torch.utils.data import DataLoader, Subset
from torchvision import datasets, transforms


def set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)

# 输入尺寸可以变，矩形和正方形
def resolve_input_shape(args):
    if args.shape_mode == "square":
        img_width = args.img_size
        img_height = args.img_size
    else:
        img_width = args.img_width
        img_height = args.img_height

    if img_width <= 0 or img_height <= 0:
        raise ValueError(f"Invalid input shape: {img_width}x{img_height}")

    return img_width, img_height


class DWConvBlock(nn.Module):
    def __init__(self, in_channels: int, out_channels: int, stride: int):
        super().__init__()
        self.block = nn.Sequential(
            nn.Conv2d(
                in_channels,
                in_channels,
                kernel_size=3,
                stride=stride,
                padding=1,
                groups=in_channels,
                bias=False,
            ),
            nn.BatchNorm2d(in_channels),
            nn.ReLU(inplace=True),
            nn.Conv2d(in_channels, out_channels, kernel_size=1, stride=1, padding=0, bias=False),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.block(x)

# 模型定义：TinyClassifier，基于深度可分离卷积，适合小模型和低算力场景
class TinyClassifier(nn.Module):
    def __init__(self, num_classes: int, width_mult: float = 1.0):
        super().__init__()

        def c(ch: int) -> int:
            return max(8, int(ch * width_mult))

        self.features = nn.Sequential(
            nn.Conv2d(3, c(12), kernel_size=3, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(c(12)),
            nn.ReLU(inplace=True),
            DWConvBlock(c(12), c(16), stride=1),
            DWConvBlock(c(16), c(24), stride=2),
            DWConvBlock(c(24), c(32), stride=1),
            DWConvBlock(c(32), c(48), stride=2),
            DWConvBlock(c(48), c(48), stride=1),
            nn.Conv2d(c(48), c(64), kernel_size=1, stride=1, padding=0, bias=False),
            nn.BatchNorm2d(c(64)),
            nn.ReLU(inplace=True),
        )
        self.pool = nn.AdaptiveAvgPool2d(1)
        self.classifier = nn.Linear(c(64), num_classes)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.features(x)
        x = self.pool(x).flatten(1)
        return self.classifier(x)


def _extract_target_id(sample_path: str) -> str:
    """
    Parse target id from filename so frames of the same physical object stay in one split.
    Examples:
      bulletproof_vest_003__on__lq_roi_000123.png -> bulletproof_vest_003
      ambulance_069__on__roi_000001.png           -> ambulance_069
    """
    name = Path(sample_path).name
    lower = name.lower()

    on_pos = lower.find("__on__")
    if on_pos > 0:
        return name[:on_pos]

    # Fallback: remove common frame suffixes like "_roi_000123"
    m = re.match(r"^(.*?)(?:_lq)?_roi_\d+(?:\.[^.]+)*$", lower)
    if m and m.group(1):
        return m.group(1)

    # Last fallback: strip extension only.
    return Path(name).stem


def _split_grouped_indices(indices_by_target, train_ratio: float, val_ratio: float, rng: random.Random):
    groups = list(indices_by_target.values())
    rng.shuffle(groups)

    n_groups = len(groups)
    if n_groups == 0:
        return [], [], []

    n_train = int(round(n_groups * train_ratio))
    n_val = int(round(n_groups * val_ratio))
    n_train = max(1, min(n_train, n_groups))
    n_val = max(0, min(n_val, n_groups - n_train))

    # Keep test split non-empty whenever possible.
    if n_groups >= 2 and (n_train + n_val) >= n_groups:
        if n_train > 1:
            n_train -= 1
        elif n_val > 0:
            n_val -= 1

    # Keep val split non-empty whenever possible (for early stopping stability).
    if n_groups >= 3 and n_val == 0:
        if n_train > 1:
            n_train -= 1
            n_val = 1
        else:
            n_val = 1

    train_groups = groups[:n_train]
    val_groups = groups[n_train:n_train + n_val]
    test_groups = groups[n_train + n_val:]

    train_indices = [idx for group in train_groups for idx in group]
    val_indices = [idx for group in val_groups for idx in group]
    test_indices = [idx for group in test_groups for idx in group]
    return train_indices, val_indices, test_indices


def _build_random_frame_splits(image_folder: datasets.ImageFolder, seed: int, train_ratio: float, val_ratio: float):
    per_class_indices = {idx: [] for idx in range(len(image_folder.classes))}
    for sample_index, (_, class_idx) in enumerate(image_folder.samples):
        per_class_indices[class_idx].append(sample_index)

    train_indices = []
    val_indices = []
    test_indices = []

    rng = random.Random(seed)
    for _, indices in per_class_indices.items():
        rng.shuffle(indices)
        n_total = len(indices)
        n_train = int(n_total * train_ratio)
        n_val = int(n_total * val_ratio)
        train_indices.extend(indices[:n_train])
        val_indices.extend(indices[n_train:n_train + n_val])
        test_indices.extend(indices[n_train + n_val:])

    rng.shuffle(train_indices)
    rng.shuffle(val_indices)
    rng.shuffle(test_indices)
    return train_indices, val_indices, test_indices


def _build_grouped_target_splits(image_folder: datasets.ImageFolder, seed: int, train_ratio: float, val_ratio: float):
    per_class_target_indices = {idx: {} for idx in range(len(image_folder.classes))}
    for sample_index, (sample_path, class_idx) in enumerate(image_folder.samples):
        target_id = _extract_target_id(sample_path)
        target_map = per_class_target_indices[class_idx]
        if target_id not in target_map:
            target_map[target_id] = []
        target_map[target_id].append(sample_index)

    train_indices = []
    val_indices = []
    test_indices = []

    rng = random.Random(seed)
    for class_idx in per_class_target_indices:
        cls_train, cls_val, cls_test = _split_grouped_indices(
            per_class_target_indices[class_idx],
            train_ratio=train_ratio,
            val_ratio=val_ratio,
            rng=rng,
        )
        train_indices.extend(cls_train)
        val_indices.extend(cls_val)
        test_indices.extend(cls_test)

    rng.shuffle(train_indices)
    rng.shuffle(val_indices)
    rng.shuffle(test_indices)
    return train_indices, val_indices, test_indices


def build_splits(
    image_folder: datasets.ImageFolder,
    seed: int,
    train_ratio: float,
    val_ratio: float,
    split_mode: str = "random",
):
    if split_mode == "random":
        return _build_random_frame_splits(image_folder, seed, train_ratio, val_ratio)
    if split_mode == "grouped":
        return _build_grouped_target_splits(image_folder, seed, train_ratio, val_ratio)
    raise ValueError(f"Unknown split_mode: {split_mode}. Expected one of: random, grouped")


# 数据增强
'''
light:
    数据增强流程(轻微）：
    轻微视角变化10度
    亮度/对比度/饱和度微调15%，色调5%处理光照变化
strong:
    resize固定尺寸
    RandomAffine模拟轻微旋转轻微平移轻微缩放轻微剪切
    旋转不过8度平移小于宽高6%，缩放在0.92-1.08，剪切4度
    ColorJitter亮度/对比度12%饱和度微调10%，色调3%处理光照变化
    GaussianBlur模拟轻微失焦 轻微运动模糊低质量成像但概率只给e.15，因为不能太常发生
    RandomErasing真实场景经常不是整块目标都清楚完整，轻度随机擦除能逼模型不要只盯着某一个固定小区域
'''
def build_train_transform(img_width: int, img_height: int, augment_level: str):
    if augment_level == "light":
        return transforms.Compose([
            transforms.Resize((img_height, img_width)),
            transforms.RandomRotation(10),
            transforms.ColorJitter(brightness=0.15, contrast=0.15, saturation=0.15, hue=0.05),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ])

    if augment_level == "strong":
        return transforms.Compose([
            transforms.Resize((img_height, img_width)),
            transforms.RandomApply([
                transforms.RandomAffine(
                    degrees=8,
                    translate=(0.06, 0.06),
                    scale=(0.92, 1.08),
                    shear=4,
                )
            ], p=0.8),
            transforms.ColorJitter(
                brightness=0.12,
                contrast=0.12,
                saturation=0.10,
                hue=0.03,
            ),
            transforms.RandomApply([
                transforms.GaussianBlur(kernel_size=3, sigma=(0.1, 1.0))
            ], p=0.15),
            transforms.ToTensor(),
            transforms.RandomErasing(
                p=0.15,
                scale=(0.02, 0.08),
                ratio=(0.5, 1.8),
                value="random",
            ),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ])

    raise ValueError(f"Unknown augment_level: {augment_level}. Expected one of: light, strong")


# 数据增强
def get_data_loaders(
    data_root: Path,
    test_root: Path | None,
    img_width: int,
    img_height: int,
    batch_size: int,
    num_workers: int,
    seed: int,
    split_mode: str = "random",
    augment_level: str = "strong",
    use_cuda: bool = False,
):
    train_tf = build_train_transform(
        img_width=img_width,
        img_height=img_height,
        augment_level=augment_level,
    )

    eval_tf = transforms.Compose([
        transforms.Resize((img_height, img_width)),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ])

    base_dataset = datasets.ImageFolder(str(data_root), transform=None)
    train_indices, val_indices, test_indices = build_splits(
        base_dataset,
        seed,
        train_ratio=0.8,
        val_ratio=0.1,
        split_mode=split_mode,
    )

    train_dataset = datasets.ImageFolder(str(data_root), transform=train_tf)
    val_dataset = datasets.ImageFolder(str(data_root), transform=eval_tf)

    train_subset = Subset(train_dataset, train_indices)
    val_subset = Subset(val_dataset, val_indices)

    if test_root is None:
        test_dataset = datasets.ImageFolder(str(data_root), transform=eval_tf)
        test_subset = Subset(test_dataset, test_indices)
    else:
        test_dataset = datasets.ImageFolder(str(test_root), transform=eval_tf)
        if test_dataset.classes != base_dataset.classes:
            raise RuntimeError(
                f"test-root classes mismatch: train={base_dataset.classes}, test={test_dataset.classes}"
            )
        test_subset = test_dataset

    loader_common = {
        "batch_size": batch_size,
        "num_workers": num_workers,
        "pin_memory": use_cuda,
    }
    if num_workers > 0:
        # Keep workers alive across epochs to reduce worker startup overhead.
        loader_common["persistent_workers"] = True
        loader_common["prefetch_factor"] = 2

    train_loader = DataLoader(train_subset, shuffle=True, **loader_common)
    val_loader = DataLoader(val_subset, shuffle=False, **loader_common)
    test_loader = DataLoader(test_subset, shuffle=False, **loader_common)
    return train_loader, val_loader, test_loader, base_dataset.classes, base_dataset.class_to_idx


def run_epoch(model, loader, criterion, optimizer, device):
    train_mode = optimizer is not None
    if train_mode:
        model.train()
    else:
        model.eval()

    total_loss = 0.0
    total_correct = 0
    total_count = 0

    non_blocking = device.type == "cuda"
    with torch.set_grad_enabled(train_mode):
        for images, labels in loader:
            images = images.to(device, non_blocking=non_blocking)
            labels = labels.to(device, non_blocking=non_blocking)

            logits = model(images)
            loss = criterion(logits, labels)

            if train_mode:
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

            total_loss += loss.item() * labels.size(0)
            total_correct += (logits.argmax(dim=1) == labels).sum().item()
            total_count += labels.size(0)

    avg_loss = total_loss / max(1, total_count)
    avg_acc = total_correct / max(1, total_count)
    return avg_loss, avg_acc


def benchmark_torch_cpu(
    model: nn.Module,
    img_width: int,
    img_height: int,
    loops: int = 500,
    warmup: int = 50,
):
    model = model.cpu().eval()
    torch.set_num_threads(1)
    x = torch.randn(1, 3, img_height, img_width)
    with torch.no_grad():
        for _ in range(warmup):
            _ = model(x)
        start = time.perf_counter()
        for _ in range(loops):
            _ = model(x)
        end = time.perf_counter()
    avg_ms = (end - start) * 1000.0 / loops
    return avg_ms


def export_onnx(model: nn.Module, onnx_path: Path, img_width: int, img_height: int):
    model.eval()
    dummy = torch.randn(1, 3, img_height, img_width)
    torch.onnx.export(
        model,
        dummy,
        str(onnx_path),
        opset_version=12,
        do_constant_folding=True,
        input_names=["input"],
        output_names=["output"],
        dynamic_axes={"input": {0: "batch"}, "output": {0: "batch"}},
    )
    onnx_model, check = simplify(str(onnx_path))
    if not check:
        raise RuntimeError("onnxsim simplify failed")
    import onnx

    onnx.save(onnx_model, str(onnx_path))


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
        help="Optional independent test dataset root. When set, training/validation use --data-root and final test uses --test-root.",
    )
    parser.add_argument(
        "--shape-mode",
        type=str,
        default="square",
        choices=["square", "rect"],
        help="Input shape mode: square uses --img-size, rect uses --img-width/--img-height.",
    )
    parser.add_argument("--img-size", type=int, default=64, help="Square input size when --shape-mode square.")
    parser.add_argument("--img-width", type=int, default=64, help="Input width when --shape-mode rect.")
    parser.add_argument("--img-height", type=int, default=48, help="Input height when --shape-mode rect.")
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--epochs", type=int, default=35)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--weight-decay", type=float, default=2e-4)
    parser.add_argument("--num-workers", type=int, default=4)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--width-mult", type=float, default=0.6)
    parser.add_argument("--patience", type=int, default=8)
    parser.add_argument("--target-acc", type=float, default=0.95)
    parser.add_argument(
        "--split-mode",
        type=str,
        default="random",
        choices=["random", "grouped"],
        help="Data split strategy: random(frame-level) or grouped(target-id-level).",
    )
    parser.add_argument(
        "--augment-level",
        type=str,
        default="strong",
        choices=["light", "strong"],
        help="Training augmentation strength.",
    )
    parser.add_argument("--out-dir", type=str, default="artifacts")
    args = parser.parse_args()

    set_seed(args.seed)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    data_root = Path(args.data_root)
    test_root = Path(args.test_root) if args.test_root else None

    if not data_root.exists() or not data_root.is_dir():
        raise FileNotFoundError(f"data-root not found or not a directory: {data_root}")

    class_dirs = [p for p in data_root.iterdir() if p.is_dir()]
    if len(class_dirs) < 2:
        raise RuntimeError(f"data-root must contain at least 2 class folders: {data_root}")
    if test_root is not None and (not test_root.exists() or not test_root.is_dir()):
        raise FileNotFoundError(f"test-root not found or not a directory: {test_root}")

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    use_cuda = device.type == "cuda"
    if use_cuda:
        # Fixed input size workload benefits from cuDNN autotune.
        torch.backends.cudnn.benchmark = True

    img_width, img_height = resolve_input_shape(args)

    print(f"Using device: {device}")
    print(f"Data root: {data_root}")
    print(f"Test root: {test_root if test_root is not None else '[split-from-train]'}")
    print(f"Split mode: {args.split_mode}")
    print(f"Augment level: {args.augment_level}")
    print(f"Input shape mode: {args.shape_mode}, size: {img_width}x{img_height}")

    train_loader, val_loader, test_loader, classes, class_to_idx = get_data_loaders(
        data_root=data_root,
        test_root=test_root,
        img_width=img_width,
        img_height=img_height,
        batch_size=args.batch_size,
        num_workers=args.num_workers,
        seed=args.seed,
        split_mode=args.split_mode,
        augment_level=args.augment_level,
        use_cuda=use_cuda,
    )

    model = TinyClassifier(num_classes=len(classes), width_mult=args.width_mult).to(device)
    n_params = sum(p.numel() for p in model.parameters())
    print(f"Classes: {classes}")
    print(f"Train/Val/Test: {len(train_loader.dataset)}/{len(val_loader.dataset)}/{len(test_loader.dataset)}")
    print(f"Model params: {n_params}")

    criterion = nn.CrossEntropyLoss()
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=args.epochs)

    best_val_acc = 0.0
    best_epoch = -1
    best_path = out_dir / "best_model.pt"
    bad_epochs = 0
    reached_target_acc = False

    for epoch in range(1, args.epochs + 1):
        epoch_start = time.perf_counter()
        train_loss, train_acc = run_epoch(model, train_loader, criterion, optimizer, device)
        val_loss, val_acc = run_epoch(model, val_loader, criterion, None, device)
        scheduler.step()
        epoch_sec = time.perf_counter() - epoch_start
        train_img_s = len(train_loader.dataset) / max(epoch_sec, 1e-8)

        print(
            f"Epoch {epoch:02d}/{args.epochs} | "
            f"train_loss={train_loss:.4f} train_acc={train_acc:.4f} | "
            f"val_loss={val_loss:.4f} val_acc={val_acc:.4f} | "
            f"time={epoch_sec:.2f}s speed={train_img_s:.1f} img/s"
        )

        if val_acc > best_val_acc:
            best_val_acc = val_acc
            best_epoch = epoch
            bad_epochs = 0
            torch.save(
                {
                    "model_state": model.state_dict(),
                    "classes": classes,
                    "class_to_idx": class_to_idx,
                    "img_size": args.img_size if args.shape_mode == "square" else None,
                    "img_width": img_width,
                    "img_height": img_height,
                    "shape_mode": args.shape_mode,
                    "augment_level": args.augment_level,
                    "width_mult": args.width_mult,
                    "mean": [0.485, 0.456, 0.406],
                    "std": [0.229, 0.224, 0.225],
                },
                best_path,
            )
        else:
            bad_epochs += 1

        if best_val_acc >= args.target_acc:
            reached_target_acc = True
            print(
                f"Reached target val_acc={args.target_acc:.4f} at epoch {epoch}, "
                f"best_val_acc={best_val_acc:.4f}. Stop training and continue next stage."
            )
            break

        if bad_epochs >= args.patience:
            print(f"Early stopping at epoch {epoch}, best epoch = {best_epoch}")
            break

    checkpoint = torch.load(best_path, map_location="cpu")
    model.load_state_dict(checkpoint["model_state"])
    model = model.to(device)
    test_loss, test_acc = run_epoch(model, test_loader, criterion, None, device)
    print(f"Best val_acc={best_val_acc:.4f} @ epoch {best_epoch}")
    print(f"Test loss={test_loss:.4f}, test acc={test_acc:.4f}")

    model_cpu = model.cpu().eval()
    onnx_path = out_dir / "tiny_classifier_fp32.onnx"
    export_onnx(model_cpu, onnx_path, img_width, img_height)

    cpu_ms = benchmark_torch_cpu(model_cpu, img_width, img_height)
    print(f"Torch CPU(1 thread) avg latency: {cpu_ms:.3f} ms")

    metrics = {
        "best_val_acc": best_val_acc,
        "best_epoch": best_epoch,
        "target_acc": args.target_acc,
        "reached_target_acc": reached_target_acc,
        "test_acc": test_acc,
        "test_loss": test_loss,
        "torch_cpu_1thread_ms": cpu_ms,
        "num_params": n_params,
        "classes": classes,
        "onnx_path": str(onnx_path),
        "shape_mode": args.shape_mode,
        "augment_level": args.augment_level,
        "img_size": args.img_size if args.shape_mode == "square" else None,
        "img_width": img_width,
        "img_height": img_height,
        "width_mult": args.width_mult,
    }

    labels_path = out_dir / "labels.txt"
    with open(labels_path, "w", encoding="utf-8") as f:
        for name in classes:
            f.write(f"{name}\n")

    metrics["labels_path"] = str(labels_path)

    with open(out_dir / "metrics.json", "w", encoding="utf-8") as f:
        json.dump(metrics, f, ensure_ascii=False, indent=2)

    print(f"Saved: {best_path}")
    print(f"Saved: {onnx_path}")
    print(f"Saved: {labels_path}")
    print(f"Saved: {out_dir / 'metrics.json'}")


if __name__ == "__main__":
    main()
