import sys
from pathlib import Path

# 🧠 최초 1회 기준 루트 설정
if getattr(sys, "frozen", False):
    BYUL_GRID_ROOT = Path(sys.executable).resolve().parent
else:
    BYUL_GRID_ROOT = Path(__file__).resolve().parents[1]

BYUL_GRID_PATH = BYUL_GRID_ROOT / 'byul_grid'
BYUL_WRAPPER_ROOT = BYUL_GRID_ROOT.parent / 'byul_wrapper'
GUI_PATH = BYUL_GRID_PATH / 'gui'
IMAGES_PATH = BYUL_GRID_ROOT / 'assets' / 'images'

# sys.path에 디렉토리 추가 (중복 방지)
if str(BYUL_GRID_ROOT) not in sys.path:
    sys.path.insert(0, str(BYUL_GRID_ROOT))

# sys.path에 디렉토리 추가 (중복 방지)
if str(BYUL_GRID_PATH) not in sys.path:
    sys.path.insert(0, str(BYUL_GRID_PATH))

# sys.path에 디렉토리 추가 (중복 방지)
if str(BYUL_WRAPPER_ROOT) not in sys.path:
    sys.path.insert(0, str(BYUL_WRAPPER_ROOT))

if str(GUI_PATH) not in sys.path:
    sys.path.insert(0, str(GUI_PATH))

if __name__ == '__main__':
    print(f'BYUL_GRID_ROOT : {BYUL_GRID_ROOT}')
    print(f'BYUL_GRID_PATH : {BYUL_GRID_PATH}')
    print(f'BYUL_WRAPPER_ROOT : {BYUL_WRAPPER_ROOT}')
    print(f'GUI_PATH : {GUI_PATH}')
    print(f'IMAGES_PATH : {IMAGES_PATH}')

    print(f'sys.path : {sys.path}')
