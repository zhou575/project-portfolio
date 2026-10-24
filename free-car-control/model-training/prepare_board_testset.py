import argparse # 解析命令行参数
import random #  用来随机抽样图片
import shutil # 用来做文件复制和文件夹删除
from pathlib import Path # 用于更方便地处理路径

'''
从一个按类别分文件夹存放的数据集里，每个类别随机抽取固定数量的图片，复制到新的输出目录里，形成一份给板端/测试用的小测试集
'''

def main():
    parser = argparse.ArgumentParser()
    # 原始数据集根目录dataset
    parser.add_argument(
        "--data-root",
        type=str,
        default="dataset",
        help="Dataset root folder (ImageFolder structure), e.g. dataset or D:/my_data/cls_dataset",
    )
    # 目录结构要像这样 dataset/class_A/1.jpg
    parser.add_argument("--out-dir", type=str, default="board_pack/test_images")
    # 每个类别抽取多少张图片
    parser.add_argument("--per-class", type=int, default=30)
    # 随机种子
    parser.add_argument("--seed", type=int, default=42)
    # 类别文件夹重命名映射,给输出目录文件夹改名
    parser.add_argument(
        "--label-map",
        type=str,
        default="",
        help="Optional mapping like 'vehicle=vehicle,weapon=weapon,supplies=supplies'",
    )
    # 解析参数
    args = parser.parse_args()

    # 固定随机种子
    random.seed(args.seed)
    # 构造并检查数据集路径
    data_root = Path(args.data_root)
    if not data_root.exists() or not data_root.is_dir():
        raise FileNotFoundError(f"data-root not found or not a directory: {data_root}")

    # 构造并创建输出目录
    out_dir = Path(args.out_dir)
    # 确保输出目录存在
    out_dir.mkdir(parents=True, exist_ok=True)

    # 初始化 label_map
    # 传car=vehicle,gun=weapon
    # 得到label_map = {"car": "vehicle","gun": "weapon"}
    label_map = {}
    if args.label_map:
        for item in args.label_map.split(","):
            if "=" in item:
                src, dst = item.split("=", 1)
                label_map[src.strip()] = dst.strip()

    # 找到所有类别文件夹
    # 获取数据集根目录下所有类别子文件夹，并按名称排序
    classes = [p for p in data_root.iterdir() if p.is_dir()]
    classes.sort(key=lambda p: p.name)
    # 检查类别目录是否为空
    if not classes:
        raise RuntimeError(f"No class folders found under: {data_root}")

    # 定义允许的图片后缀
    valid_ext = {".jpg", ".jpeg", ".png", ".bmp"}

    # 逐类别处理
    for class_dir in classes:
        # 找当前类别下所有合法图片
        # 列出当前类别文件夹里所有合法图片文件，并按文件名排序
        imgs = [p for p in class_dir.iterdir() if p.is_file() and p.suffix.lower() in valid_ext]
        imgs.sort(key=lambda p: p.name)

        # 检查这个类别图片够不够
        if len(imgs) < args.per_class:
            raise RuntimeError(f"{class_dir.name} images not enough: {len(imgs)} < {args.per_class}")
        # 随机抽样
        chosen = random.sample(imgs, args.per_class)
        # 决定输出类别名
        out_name = label_map.get(class_dir.name, class_dir.name)
        # 构造目标类别目录
        dst_class = out_dir / out_name
        if dst_class.exists():
            shutil.rmtree(dst_class)
        # 重新创建目标类别目录
        dst_class.mkdir(parents=True, exist_ok=True)

        # 复制抽中的图片
        for p in chosen:
            shutil.copy2(p, dst_class / p.name)

        # 打印当前类别抽样结果
        print(f"{class_dir.name}: {len(chosen)} images")
    # 全部类别处理完后打印总结果
    print(f"Saved test set to: {out_dir}")


if __name__ == "__main__":
    main()
