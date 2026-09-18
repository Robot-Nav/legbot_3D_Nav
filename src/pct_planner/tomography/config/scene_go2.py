from .scene_building import SceneBuilding
from copy import deepcopy
class SceneGo2:
    pcd = deepcopy(SceneBuilding.pcd)
    map = deepcopy(SceneBuilding.map)
    trav = deepcopy(SceneBuilding.trav)
    trav.interval_min = 0.50
    trav.interval_free = 0.55
    trav.step_max = 0.16
    trav.inflation = 0.20
