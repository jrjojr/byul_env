from grid.grid_cell import GridCell

import json
from pathlib import Path

from utils.log_to_panel import g_logger

from PySide6.QtCore import QThread, Signal

import time

import json
import time
from pathlib import Path

from PySide6.QtCore import QThread, Signal

from grid.grid_cell import GridCell, TerrainType

from utils.log_to_panel import g_logger

import random

class GridBlock:
    def __init__(self, x0: int, y0: int, block_size: int = 100,
                 cells: dict[tuple, GridCell] = None):
        self.x0 = x0
        self.y0 = y0
        self.block_size = block_size
        self.cells: dict[tuple, GridCell] = cells if cells is not None else {}

    def to_dict(self) -> dict:
        return {
            "x0": self.x0,
            "y0": self.y0,
            "block_size": self.block_size,
            "cells": [cell.to_dict() for cell in self.cells.values()]
        }

    def to_json(self, folder: str):
        Path(folder).mkdir(parents=True, exist_ok=True)
        data = self.to_dict()
        path = Path(folder) / f"block_{self.x0}_{self.y0}.json"
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=4, ensure_ascii=False)

    @classmethod
    def from_dict(cls, data: dict):
        x0 = data["x0"]
        y0 = data["y0"]
        block_size = data["block_size"]
        raw_cells = data["cells"]

        cell_dict: dict[tuple, GridCell] = {}
        for raw in raw_cells:
            cell = GridCell.from_dict(raw)
            cell_dict[(cell.x, cell.y)] = cell

        return cls(x0, y0, block_size, cell_dict)

    def close(self):
        for cell in self.cells.values():
            cell.close()
        self.cells.clear()

    def get_origin(self) -> tuple[int,int]:
        return (self.x0, self.y0)

    def __getitem__(self, pos: tuple[int,int]) -> GridCell:
        return self.cells[pos]

    def __setitem__(self, pos: tuple[int,int], cell: GridCell):
        self.cells[pos] = cell

    def __contains__(self, pos: tuple[int,int]) -> bool:
        return pos in self.cells

    def __len__(self) -> int:
        return len(self.cells)

    def __iter__(self):
        return iter(self.cells.items())

class BlockThread(QThread):
    succeeded = Signal(tuple)
    failed = Signal(tuple)
    loading_block_started = Signal(float)

    def __init__(self, block: GridBlock):
        super().__init__()
        self.block = block
        self.result_dict = {}

    def run(self):
        origin = self.block.get_origin()
        self.loading_block_started.emit(time.time())
        try:
            self.result_dict = self.block.cells
            self.succeeded.emit(origin)
        except Exception as e:
            g_logger.log_debug_threadsafe(f"[\u274c 블럭 생성 실패] {origin}: {e}")
            self.failed.emit(origin)

class BlockSaverThread(QThread):
    succeeded = Signal(tuple)
    failed = Signal(tuple)

    def __init__(self, block: GridBlock, folder: str):
        super().__init__()
        self.block = block
        self.folder = Path(folder)

    def run(self):
        origin = self.block.get_origin()
        try:
            self.folder.mkdir(parents=True, exist_ok=True)
            path = self.folder / f"block_{self.block.x0}_{self.block.y0}.json"
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self.block.to_dict(), f, indent=4, ensure_ascii=False)
            self.succeeded.emit(origin)
        except Exception as e:
            g_logger.log_debug_threadsafe(f"[\u274c 블럭 저장 실패] {origin}: {e}")
            self.failed.emit(origin)

class BlockLoaderThread(QThread):
    succeeded = Signal(GridBlock)
    failed = Signal(str)

    def __init__(self, path: Path):
        super().__init__()
        self.path = path

    def run(self):
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                data = json.load(f)
                block = GridBlock.from_dict(data)
                self.succeeded.emit(block)
        except Exception as e:
            g_logger.log_debug_threadsafe(f"[\u274c 블럭 로딩 실패] {self.path}: {e}")
            self.failed.emit(str(self.path))

class BlockMaker(GridBlock):
    def __init__(
        self,
        x0: int, y0: int,
        block_size: int = 100,
        make_cell_func: tuple[tuple[int, int], GridCell] = None,
        cells: dict = None,
    ):
        """
        make_cell_func: 셀을 생성하는 함수 (x, y) -> GridCell
        """
        self.block_size = block_size
        self.make_cell_func = make_cell_func or self._default_make_cell

        if cells is None:
            cells = self._generate_cells(x0, y0)

        super().__init__(x0, y0, block_size, cells)

    def _generate_cells(self, x0: int, y0: int) -> dict:
        result = {}
        for dy in range(self.block_size):
            for dx in range(self.block_size):
                x = x0 + dx
                y = y0 + dy
                result[(x, y)] = self.make_cell_func(x, y)
        return result

    def _default_make_cell(self, x: int, y: int) -> GridCell:
        # 기본 지형 랜덤 생성 (외부 지식 없이 내부적으로만 처리)
        # terrain = random.choices(
        #     population=[
        #         TerrainType.NORMAL,
        #         TerrainType.FOREST,
        #         TerrainType.MOUNTAIN,
        #         TerrainType.WATER,
        #         TerrainType.FORBIDDEN
        #     ],
        #     weights=[0.5, 0.2, 0.1, 0.1, 0.1]
        # )[0]
        # return GridCell(x=x, y=y, terrain=terrain)
        return GridCell.random(x=x, y=y, npc_chance=0.05)

class BlockMakerThread(QThread):
    succeeded = Signal(tuple, BlockMaker)
    failed = Signal(tuple)
    loading_block_started = Signal(float)

    def __init__(self,
                 x0: int, y0: int,
                 block_size: int = 100,
                 make_cell_func: tuple[tuple[int, int], GridCell] = None):
        super().__init__()
        self.x0 = x0
        self.y0 = y0
        self.block_size = block_size
        self.make_cell_func = make_cell_func
        self.result: BlockMaker | None = None

    def run(self):
        try:
            block = BlockMaker(
                x0=self.x0,
                y0=self.y0,
                block_size=self.block_size,
                make_cell_func=self.make_cell_func
            )
            self.result = block
            self.succeeded.emit((self.x0, self.y0), block)
        except Exception as e:
            g_logger.log_debug_threadsafe(
                f"[❌ BlockMakerThread 실패] ({self.x0},{self.y0}): {e}"
            )
            self.failed.emit((self.x0, self.y0))
