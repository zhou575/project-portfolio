# 文件工作方式如下
# 批量读取当前目录下 raw_video 文件夹中的视频文件，逐帧提取并保存到 raw_picture 文件夹中，每个视频对应一个同名子文件夹。
'''
    当前目录/
    ├─ raw_video/
    │  ├─ video1.avi
    │  ├─ video2.avi
    │  └─ video3.mp4
    └─ raw_picture/
    ├─ video1/
    │  ├─ video1_f000000_t0000.000.png
    │  ├─ video1_f000001_t0000.050.png
    │  └─ ...
    ├─ video2/
    │  ├─ video2_f000000_t0000.000.png
    │  └─ ...
    └─ video3/
        ├─ video3_f000000_t0000.000.png
        └─ ...
'''


import argparse

import cv2
from pathlib import Path


def is_black_frame(frame, mean_thr=3.0, max_thr=8):
    """
    判断是否为接近全黑的无效帧
    mean_thr: 灰度均值阈值
    max_thr : 灰度最大值阈值
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    mean_val = gray.mean()
    max_val = gray.max()

    # 同时满足“整体很暗”且“最亮点也不亮”，认为是黑帧
    return mean_val <= mean_thr and max_val <= max_thr


def extract_frames_from_video(
    video_path,
    out_dir,
    step=1,
    skip_black=True,
    mean_thr=3.0,
    max_thr=8,
):
    video_path = Path(video_path)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        print(f"[跳过] 无法打开视频: {video_path}")
        return 0, 0, 0

    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    print(f"\n开始处理视频: {video_path.name}")
    print(f"输出目录: {out_dir}")
    print(f"FPS: {fps}")
    print(f"总帧数: {total_frames}")

    frame_idx = 0
    saved_count = 0
    skipped_black_count = 0
    skipped_step_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 按步长抽帧
        if frame_idx % step != 0:
            skipped_step_count += 1
            frame_idx += 1
            continue

        # 跳过全黑无效帧
        if skip_black and is_black_frame(frame, mean_thr=mean_thr, max_thr=max_thr):
            skipped_black_count += 1
            frame_idx += 1
            continue

        timestamp_sec = frame_idx / fps if fps > 0 else 0.0
        save_name = f"{video_path.stem}_f{frame_idx:06d}_t{timestamp_sec:08.3f}.png"
        save_path = out_dir / save_name

        cv2.imwrite(str(save_path), frame)
        saved_count += 1

        frame_idx += 1

    cap.release()

    print("处理完成：")
    print(f"保存帧数: {saved_count}")
    print(f"跳过黑帧: {skipped_black_count}")
    print(f"按步长跳过: {skipped_step_count}")

    return saved_count, skipped_black_count, skipped_step_count


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--video-dir",
        type=Path,
        default=Path("raw_video"),
        help="Folder containing source videos.",
    )
    parser.add_argument(
        "--picture-dir",
        type=Path,
        default=Path("raw_picture"),
        help="Folder where extracted frames will be saved.",
    )
    parser.add_argument(
        "--step",
        type=int,
        default=5,
        help="Save one frame every N frames. Use 1 to save every frame.",
    )
    parser.add_argument(
        "--keep-black",
        action="store_true",
        help="Keep near-black frames instead of skipping them.",
    )
    parser.add_argument("--mean-thr", type=float, default=3.0)
    parser.add_argument("--max-thr", type=float, default=8)
    args = parser.parse_args()

    if args.step < 1:
        raise ValueError("--step must be >= 1")

    # 当前路径下的视频目录
    raw_video_dir = args.video_dir
    # 当前路径下的图片输出目录
    raw_picture_dir = args.picture_dir

    raw_picture_dir.mkdir(parents=True, exist_ok=True)

    if not raw_video_dir.exists() or not raw_video_dir.is_dir():
        raise FileNotFoundError(f"未找到视频目录: {raw_video_dir.resolve()}")

    # 支持的视频后缀
    video_exts = {".avi", ".mp4", ".mov", ".mkv", ".wmv", ".flv", ".m4v"}

    video_files = sorted(
        [p for p in raw_video_dir.iterdir() if p.is_file() and p.suffix.lower() in video_exts]
    )

    if not video_files:
        print(f"在 {raw_video_dir.resolve()} 中没有找到视频文件。")
        return

    total_saved = 0
    total_black = 0
    total_step = 0

    print(f"共找到 {len(video_files)} 个视频文件。")

    print(f"Frame step: {args.step}")
    print(f"Skip black frames: {not args.keep_black}")

    for video_path in video_files:
        # 每个视频在 raw_picture 下对应一个同名文件夹
        out_dir = raw_picture_dir / video_path.stem

        saved_count, skipped_black_count, skipped_step_count = extract_frames_from_video(
            video_path=video_path,
            out_dir=out_dir,
            step=args.step,
            skip_black=not args.keep_black,
            mean_thr=args.mean_thr,
            max_thr=args.max_thr,
        )

        total_saved += saved_count
        total_black += skipped_black_count
        total_step += skipped_step_count

    print("\n====================")
    print("全部视频处理完成")
    print(f"总保存帧数: {total_saved}")
    print(f"总跳过黑帧: {total_black}")
    print(f"总按步长跳过: {total_step}")
    print(f"图片总目录: {raw_picture_dir.resolve()}")


if __name__ == "__main__":
    main()
