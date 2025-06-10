
import argparse
import os
import torch
import numpy as np
from tqdm import tqdm

import re
import sys
sys.path.append('/home/larry/codeGit/implict_voxel/src/feat_voxel_map_unique')

from Model.VoxelDataset import VoxelDataset     # dataloader
from Model.Point2VoxelNetUnique import FullModel, WeightedLoss

import matplotlib.pyplot as plt
import pandas as pd




IS_VSCODE_DEBUG = True



def parse_args():
    parser = argparse.ArgumentParser('Weight Model training')
    # parser.add_argument('--data_dir', type=str, default='/home/larry/Desktop/p2v_debug/')
    parser.add_argument('--data_dir', type=str, default='/home/larry/Desktop/p2v_debug/select')
    parser.add_argument('--model', type=str, default='/home/larry/codeGit/implict_voxel/src/feat_voxel_map_unique/checkpoint/bg3-300.pth')
    return parser.parse_args()



def visualize_cpp_running(data_folder, model:FullModel):
    print(f"Visualize the folder: {data_folder}")

    # load file.
    files = [f for f in os.listdir(data_folder) if re.match(r'\d+-\d+\.csv', f)]
    # files.sort(key=lambda x: [int(i) for i in x.split('-')[:2]])  # 按数字排序

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    for file in files:

        ax.clear()

        file_path = os.path.join(data_folder, file)
        
        scan_id = int(file.split('-')[0])
        point_id = int((file.split('-')[1]).split('.')[0])

        # 读取整个文件
        data = np.loadtxt(file_path, delimiter=',')
        
        # 检查数据形状是否符合预期 (52, 3)
        if data.shape != (52, 3):
            print(f"警告: 文件 {file} 有 {data.shape[0]} 行，跳过")
            continue
        
        # 分离 points 和 p2v
        voxel_points = data[:50]    # 前50行
        query_point = data[-2]      # the 51 line
        slam_p2v = data[-1]         # 最后1行

        batch_point_cloud = torch.tensor(voxel_points).unsqueeze(0).to('cpu')
        batch_query_point = torch.tensor(query_point).unsqueeze(0).to('cpu')

        model.eval()
        with torch.no_grad():
            pred_weight, pred_p2v, _ = model(batch_point_cloud, batch_query_point)
            pred_p2v = pred_p2v.squeeze(0).cpu().numpy()
            pred_weight = pred_weight.squeeze(0).cpu().numpy()
        

        SHOW_PLOT = True
        if SHOW_PLOT:
            # view
            ax.scatter(*query_point, c='red', s=50, label='Query Point')
            
            # 绘制p2v向量（红色箭头）
            ax.scatter(voxel_points[:, 0], voxel_points[:, 1], voxel_points[:, 2], c='black', s=10, alpha=0.9, label='Voxel Points')

            ax.quiver(*query_point, *slam_p2v, color='red', arrow_length_ratio=0.3, label='c++ running p2v')
            # Predict p2v
            ax.quiver(*query_point, *pred_p2v, color='black', arrow_length_ratio=0.3, label='python-predict p2v')
            
            # 添加 weight 值文本
            # ax.text2D(0.0, 0.98, f"Query : [{query_point[0]:.3f}, {query_point[1]:.3f}, {query_point[2]:.3f}]", transform=ax.transAxes, fontsize=12)
            # ax.text2D(0.0, 0.95, f"C++'s predict  : [{slam_p2v[0]:.2f}, {slam_p2v[1]:.2f}, {slam_p2v[2]:.2f}]", transform=ax.transAxes, fontsize=12)
            # ax.text2D(0.0, 0.92, f"Python predict : [{pred_p2v[0]:.2f}, {pred_p2v[1]:.2f}, {pred_p2v[2]:.2f}],  Weight predict: {pred_weight:.2f}", transform=ax.transAxes, fontsize=12)
            ax.text2D(0.0, 0.92, f"predict : [{pred_p2v[0]:.2f}, {pred_p2v[1]:.2f}, {pred_p2v[2]:.2f}],  Weight predict: {pred_weight:.2f}", transform=ax.transAxes, fontsize=12)

            # 设置图形属性
            ax.set_title(f"Filename: {file}")
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.legend()
            
            # 调整视角以获得更好的3D效果
            ax.view_init(elev=20, azim=45)

            # set x,y,z limit based on index
            voxel_index = (int(query_point[0]),int(query_point[1]),int(query_point[2]))
            lower_bound = np.array(voxel_index)*0.5
            upper_bound = (np.array(voxel_index) + 1) * 0.5
            ax.set_xlim(lower_bound[0], upper_bound[0])
            ax.set_ylim(lower_bound[1], upper_bound[1])
            ax.set_zlim(lower_bound[2], upper_bound[2])

            # set equal axis for better view. (should be called after set_xlim)
            ax.set_box_aspect([1, 1, 1])    # Equal aspect ratio for X, Y, Z. (different matplotlib version)
            ax.set_aspect('equal')          # May not fully work in 3D        (different matplotlib version)

            # Allow interactive rotation
            plt.show(block=False)
        


        if IS_VSCODE_DEBUG :     # for python debugger, make a breakpoint on "pass"
            pass
        

        # debug: check difference
        diff = slam_p2v - pred_p2v
        eps = 0.01
        if(abs(diff[0]) > eps or abs(diff[1])> eps or abs(diff[2])>eps):
            print(f'Scan id: {scan_id}, point_id: {point_id}')


    # plot



if __name__ == '__main__':
    args = parse_args()
    device = 'cpu'
    checkpoint = torch.load(args.model)
    start_epoch = checkpoint['epoch']
    model = FullModel(voxel_size=0.5).to(device)
    model.load_state_dict(checkpoint['model_state_dict'])

    visualize_cpp_running(args.data_dir, model)
