#!/usr/bin/env python3
"""
正确可视化 LeRobot 数据集的包装脚本
解决过滤 episode 时索引不匹配的问题
"""

import argparse
from pathlib import Path
import torch
import tqdm
import logging

from lerobot.datasets.lerobot_dataset import LeRobotDataset

logging.basicConfig(level=logging.INFO)


def visualize_episode(
    repo_id: str,
    episode_index: int,
    root: str = None,
    batch_size: int = 32,
    num_workers: int = 4,
    mode: str = "local",
    web_port: int = 9090,
    ws_port: int = 9087,
):
    """正确可视化指定的 episode"""
    
    # 🔧 修复：加载完整数据集，不过滤
    logging.info("Loading full dataset")
    dataset = LeRobotDataset(repo_id, root=root)
    
    logging.info(f"Dataset loaded: {len(dataset)} frames, {len(dataset.meta.episodes)} episodes")
    
    # 获取指定 episode 的正确索引范围
    from_idx = dataset.meta.episodes["dataset_from_index"][episode_index]
    to_idx = dataset.meta.episodes["dataset_to_index"][episode_index]
    length = to_idx - from_idx
    
    logging.info(f"Episode {episode_index}: {length} frames (index {from_idx} to {to_idx-1})")
    
    # 创建正确的索引范围
    episode_indices = list(range(from_idx, to_idx))
    
    # 创建 DataLoader（使用正确的索引）
    from torch.utils.data import Subset, DataLoader
    
    episode_subset = Subset(dataset, episode_indices)
    dataloader = DataLoader(
        episode_subset,
        batch_size=batch_size,
        num_workers=num_workers,
        shuffle=False,
    )
    
    logging.info(f"Starting Rerun visualization (mode={mode})")
    
    # 初始化 Rerun
    import rerun as rr
    
    rr.init(f"{repo_id}/episode_{episode_index}", spawn=(mode == "local"))
    
    if mode == "web":
        rr.serve(web_port=web_port, ws_port=ws_port)
    
    # 可视化数据
    for batch_idx, batch in enumerate(tqdm.tqdm(dataloader, desc=f"Episode {episode_index}")):
        frame_indices = batch["frame_index"]
        
        for i in range(len(frame_indices)):
            frame_idx = frame_indices[i].item()
            rr.set_time_sequence("frame", frame_idx)
            
            # 记录图像
            if "observation.images.cam_chest" in batch:
                img = batch["observation.images.cam_chest"][i]  # (C, H, W)
                img_np = img.permute(1, 2, 0).numpy()  # (H, W, C)
                rr.log("camera/chest", rr.Image(img_np))
            
            # 支持右臂和左臂手腕相机
            if "observation.images.cam_wrist_right" in batch:
                img = batch["observation.images.cam_wrist_right"][i]
                img_np = img.permute(1, 2, 0).numpy()
                rr.log("camera/wrist_right", rr.Image(img_np))
            
            if "observation.images.cam_wrist_left" in batch:
                img = batch["observation.images.cam_wrist_left"][i]
                img_np = img.permute(1, 2, 0).numpy()
                rr.log("camera/wrist_left", rr.Image(img_np))
            
            if "observation.task_phase" in batch:
                task_phase = batch["observation.task_phase"][i].numpy()
                rr.log("task_phase", rr.Scalars(task_phase))
            
            # 记录状态（所有关节+夹爪）
            if "observation.state" in batch:
                state = batch["observation.state"][i].numpy()
                rr.log("state/joints", rr.Scalars(state))
            
            # 记录动作（所有关节+夹爪）
            if "action" in batch:
                action = batch["action"][i].numpy()
                rr.log("action/joints", rr.Scalars(action))
    
    logging.info("✅ Visualization complete!")
    
    if mode == "local":
        input("Press Enter to close...")


def main():
    parser = argparse.ArgumentParser(description="Visualize LeRobot dataset episode (fixed version)")
    parser.add_argument("--repo-id", type=str, required=True, help="Dataset repository ID")
    parser.add_argument("--episode-index", type=int, required=True, help="Episode to visualize")
    parser.add_argument("--root", type=str, default=None, help="Dataset root directory")
    parser.add_argument("--batch-size", type=int, default=32, help="Batch size")
    parser.add_argument("--num-workers", type=int, default=4, help="Number of workers")
    parser.add_argument("--mode", type=str, default="local", choices=["local", "web"], help="Visualization mode")
    parser.add_argument("--web-port", type=int, default=9090, help="Web port (for web mode)")
    parser.add_argument("--ws-port", type=int, default=9087, help="WebSocket port (for web mode)")
    
    args = parser.parse_args()
    
    visualize_episode(
        repo_id=args.repo_id,
        episode_index=args.episode_index,
        root=args.root,
        batch_size=args.batch_size,
        num_workers=args.num_workers,
        mode=args.mode,
        web_port=args.web_port,
        ws_port=args.ws_port,
    )


if __name__ == "__main__":
    main()
