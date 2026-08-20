from .scene import ScenePCD, SceneMap, SceneTrav


class SceneBuilding():
    pcd = ScenePCD()
    pcd.file_name = 'building2_9.pcd'

    # 初始化场景地图对象
    map = SceneMap()
    # 设置地图分辨率，单位为米
    map.resolution = 0.1
    # 设置地面高度，单位为米
    map.ground_h = 0.0
    # 设置切片高度间隔，单位为米
    map.slice_dh = 0.5

    # 初始化场景可通行性分析对象
    trav = SceneTrav()
    # 设置内核大小，用于形态学操作
    trav.kernel_size = 7
    # 设置最小间隔，单位为米
    trav.interval_min = 0.50
    # 设置机器人正常工作高度，单位为米
    trav.interval_free = 0.5
    # 设置最大坡度(弧度)
    # trav.slope_max = 0.40
    trav.slope_max = 0.4
    # 设置机器狗最大台阶跨越高度，单位为米
    # trav.step_max = 0.11
    trav.step_max = 0.3
    # 设置可站立区域的最小比例
    trav.standable_ratio = 0.5
    # 设置障碍物成本值
    trav.cost_barrier = 50.0
    # 设置安全边距，单位为米
    trav.safe_margin = 0.5
    # 设置膨胀半径，单位为米
    trav.inflation = 0.1
    


