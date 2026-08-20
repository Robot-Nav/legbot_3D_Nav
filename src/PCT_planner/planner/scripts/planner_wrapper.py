import os
import sys
import pickle
import numpy as np
from scipy.interpolate import griddata  # 新增：用于XY坐标的高度插值

from utils import *

sys.path.append('../')
from lib import a_star, ele_planner, traj_opt

rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../..'


class TomogramPlanner(object):
    def __init__(self, cfg):
        self.cfg = cfg

        self.use_quintic = self.cfg.planner.use_quintic
        self.max_heading_rate = self.cfg.planner.max_heading_rate

        self.tomo_dir = rsg_root + self.cfg.wrapper.tomo_dir

        self.tomogram=None
        self.resolution = None
        self.center = None
        self.n_slice = None
        self.slice_h0 = None
        self.slice_dh = None
        self.map_dim = []
        self.offset = None

        self.layer_elev_grids = None  # 存储每个layer的高度网格（elev_g）
        self.grid_xy_coords = None    # 存储网格对应的物理XY坐标

        self.start_idx = np.zeros(3, dtype=np.int32)
        self.end_idx = np.zeros(3, dtype=np.int32)

    def loadTomogram(self, tomo_file):
        with open(self.tomo_dir + tomo_file + '.pickle', 'rb') as handle:
            data_dict = pickle.load(handle)

            self.tomogram = np.asarray(data_dict['data'], dtype=np.float32)

            self.resolution = float(data_dict['resolution'])
            self.center = np.asarray(data_dict['center'], dtype=np.double)
            self.n_slice = self.tomogram.shape[1]
            self.slice_h0 = float(data_dict['slice_h0'])
            self.slice_dh = float(data_dict['slice_dh'])
            self.map_dim = [self.tomogram.shape[2], self.tomogram.shape[3]]
            self.offset = np.array([int(self.map_dim[0] / 2), int(self.map_dim[1] / 2)], dtype=np.int32)

        trav = self.tomogram[0]
        trav_gx = self.tomogram[1]
        trav_gy = self.tomogram[2]
        elev_g = self.tomogram[3]
        elev_g = np.nan_to_num(elev_g, nan=-100)
        elev_c = self.tomogram[4]
        elev_c = np.nan_to_num(elev_c, nan=1e6)
        # 存储所有layer的高度网格（关键：后续用于XY查高度）
        self.layer_elev_grids = elev_g
        # 预计算网格对应的物理XY坐标（避免重复计算）,可行性分析的trav导入，在目标点decide_layer，选择最低cost的layer
        self._precompute_grid_xy()
        self.initPlanner(trav, trav_gx, trav_gy, elev_g, elev_c)
      


    def _precompute_grid_xy(self):
        """预计算每个网格点对应的物理XY坐标（匹配map的center和resolution）"""
        # 生成网格索引
        grid_h, grid_w = self.map_dim[0], self.map_dim[1]
        y_grid_idx, x_grid_idx = np.meshgrid(np.arange(grid_w), np.arange(grid_h))
        # 转换为物理坐标（反向复用pos2idx的逻辑）
        x_grid = (x_grid_idx - self.offset[0]) * self.resolution + self.center[0]
        y_grid = (y_grid_idx - self.offset[1]) * self.resolution + self.center[1]
        # 存储为 [grid_h, grid_w, 2] （x,y）
        self.grid_xy_coords = np.stack([x_grid, y_grid], axis=-1)

    def get_layer_height_by_xy(self, layer_idx, x, y):
        """
        输入layer索引和物理XY坐标，返回该layer下此XY对应的真实高度
        :param layer_idx: 分层索引（0~n_slice-1）
        :param x: 物理X坐标
        :param y: 物理Y坐标
        :return: 插值后的高度值（无有效值返回-100）
        """
        # 边界检查
        if layer_idx < 0 or layer_idx >= self.n_slice:
            return -100.0

        elev_grid = self.layer_elev_grids[layer_idx]
        xy_coords = self.grid_xy_coords.reshape(-1, 2)  
        elev_vals = elev_grid.reshape(-1)              

        valid_mask = elev_vals > -99.0
        valid_xy = xy_coords[valid_mask]
        valid_elev = elev_vals[valid_mask]

        if len(valid_xy) == 0:
            return -100.0

        query_height = griddata(valid_xy, valid_elev, (x, y), method='nearest', fill_value=-100.0)
        return float(query_height)
    

    

    def match_best_layer(self, x, y, target_height):
        """
        迭代所有layer，找到与目标高度最匹配的layer（核心方法）
        :param x: 物理X坐标
        :param y: 物理Y坐标
        :param target_height: 目标高度（如start_pos/end_pos的z值）
        :return: 最佳匹配的layer索引、该layer下XY对应的高度、高度差值
        """
        min_cost = float('inf')
        best_layer = 0

        # 迭代所有layer（可优化：先粗筛再细查，减少迭代次数）
        for layer_idx in range(self.n_slice):
            # 获取该layer下XY对应的真实高度
            layer_height = self.get_layer_height_by_xy(layer_idx, x, y)
            # 跳过无效层
            if layer_height == -100.0:
                continue
            # 计算高度差值（绝对值）
            diff = abs(layer_height - target_height)
            if diff > self.slice_dh :  
                continue
            # 先将物理XY坐标转换为网格索引（用于查costmap）
            grid_idx = self.pos2idx(np.array([x, y])).astype(int)
            height, width = self.tomogram[0][layer_idx].shape
            # 安全地 clip 行和列索引
            row = np.clip(grid_idx[1], 0, height - 1)
            col = np.clip(grid_idx[0], 0, width - 1)
            # 然后安全访问
            layer_cost_ = self.tomogram[0][layer_idx][row][col]
            #去除代价不存在的层级，即没有tomogram分析点云
            if layer_cost_==0:
                continue
            # 更新最佳匹配
            if layer_cost_ < min_cost:
                min_cost = layer_cost_
                best_layer = layer_idx
        return best_layer
        
    def initPlanner(self, trav, trav_gx, trav_gy, elev_g, elev_c):
        diff_t = trav[1:] - trav[:-1]
        diff_g = np.abs(elev_g[1:] - elev_g[:-1])

        gateway_up = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t < -8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[1:]))
        gateway_up[:-1] = np.logical_and(mask_t, mask_g)

        gateway_dn = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t > 8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[:-1]))
        gateway_dn[1:] = np.logical_and(mask_t, mask_g)
        
        gateway = np.zeros_like(trav, dtype=np.int32)
        gateway[gateway_up] = 2
        gateway[gateway_dn] = -2

        self.planner = ele_planner.OfflineElePlanner(
            max_heading_rate=self.max_heading_rate, use_quintic=self.use_quintic
        )
        self.planner.init_map(
            20, 15, self.resolution, self.n_slice, 1,
            trav.reshape(-1, trav.shape[-1]).astype(np.double),
            elev_g.reshape(-1, elev_g.shape[-1]).astype(np.double),
            elev_c.reshape(-1, elev_c.shape[-1]).astype(np.double),
            gateway.reshape(-1, gateway.shape[-1]),
            trav_gy.reshape(-1, trav_gy.shape[-1]).astype(np.double),
            -trav_gx.reshape(-1, trav_gx.shape[-1]).astype(np.double)
        )



    def plan(self, start_pos, end_pos):
        # TODO: calculate slice index. By default the start and end pos are all at slice 0

        # print("pos origin start:", start_pos)
        # print("pos origin end:", end_pos)

        # 匹配起始点的最佳layer
        start_layer = self.match_best_layer(
            start_pos[0], start_pos[1], start_pos[2]
        )
        # 匹配目标点的最佳layer
        end_layer= self.match_best_layer(
            end_pos[0], end_pos[1], end_pos[2]
        )
        # print("start_layer:" ,start_layer)
        # print("end_layer:" ,end_layer)
        

        self.start_idx[1:] = self.pos2idx(start_pos[:2])
        self.end_idx[1:] = self.pos2idx(end_pos[:2])
        self.start_idx[0] = start_layer
        self.end_idx[0] = end_layer
    

        self.planner.plan(self.start_idx, self.end_idx, True)
        path_finder: a_star.Astar = self.planner.get_path_finder()
        path = path_finder.get_result_matrix()
        if len(path) == 0:
            return None

        optimizer: traj_opt.GPMPOptimizer = (
            self.planner.get_trajectory_optimizer()
            if not self.use_quintic
            else self.planner.get_trajectory_optimizer_wnoj()
        )

        opt_init = optimizer.get_opt_init_value()
        init_layer = optimizer.get_opt_init_layer()
        traj_raw = optimizer.get_result_matrix()
        layers = optimizer.get_layers()
        heights = optimizer.get_heights()
        # print("heights origin:", heights)


        opt_init = np.concatenate([opt_init.transpose(1, 0), init_layer.reshape(-1, 1)], axis=-1)
        traj = np.concatenate([traj_raw, layers.reshape(-1, 1)], axis=-1)
        y_idx = (traj.shape[-1] - 1) // 2
        traj_3d = np.stack([traj[:, 0], traj[:, y_idx], heights / self.resolution], axis=1)
        traj_3d = transTrajGrid2Map(self.map_dim, self.center, self.resolution, traj_3d)

        # print(traj_raw,"traj_raw")
        return traj_3d
    

    def pos2idx(self, pos):
        pos = pos - self.center
        idx = np.round(pos / self.resolution).astype(np.int32) + self.offset
        idx = np.array([idx[1], idx[0]], dtype=np.float32)
        return idx
