import shutil
import re
import os
import json
import time
from pathlib import Path

from PySide6.QtCore import QObject, Signal, Slot, QRect, QTimer

from grid.grid_cell import GridCell, CellFlag
from grid.grid_block import GridBlock
from grid.grid_block_manager import GridBlockManager
from navgrid import c_navgrid

from utils.log_to_panel import g_logger


class Village(QObject):
    """
    Village은 GridBlockManager로부터 마을을 구성하는 셀블락들을 불러온다
    정해진 크기만큼 가장자리에 벽을 세워서 마을의 구역을 설정한다.
    마을 입구도 설정해야 한다. 여러개가 있을 수도 있고 하나만 있을 수도 있고
    없을수도 있겠다 새들이 사는곳에 입구가 있는것도 이상하네
    """
    def __init__(self, block_mgr:GridBlockManager,
                 center_x = 0, center_y =0, 
                 width=4000, height=4000,
                 ):
        super().__init__()

        self.block_mgr = block_mgr

        self.center_x = center_x
        self.center_y = center_y
        self.width = width
        self.height = height
        
        # self.block_mgr.request_load_block(self.center_x, self.center_y)



