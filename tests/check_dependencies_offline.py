"""Import native modules; do not execute planning or CUDA kernels."""
import sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
root = Path(get_package_share_directory('pct_planner'))/'planner'
sys.path[:0] = [str(root),str(root/'lib')]
import a_star
import ele_planner
import traj_opt
import py_map_manager
import open3d
import cupy
print(f'PASS PCT native imports, Open3D {open3d.__version__}, CuPy {cupy.__version__}')
