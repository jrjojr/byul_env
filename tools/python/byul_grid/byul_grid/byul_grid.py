import sys

from PySide6.QtWidgets import QApplication

from config import BYUL_GRID_ROOT, BYUL_GRID_PATH, BYUL_WRAPPER_ROOT
from world.world import World
from gui.grid_viewer import GridViewer

if __name__ == "__main__":
    # config 로딩용 로딩을 해야 필요한 디렉토리가 추가된다 sys.path에...
    print(f'BYUL_GRID_ROOT : {BYUL_GRID_ROOT}')
    print(f'BYUL_GRID_PATH : {BYUL_GRID_PATH}')
    print(f'BYUL_WRAPPER_ROOT : {BYUL_WRAPPER_ROOT}')

    app = QApplication(sys.argv)
    world = World(block_size=100)
    viewer = GridViewer(world)
    viewer.resize(1000, 900)
    viewer.show()
    sys.exit(app.exec())
