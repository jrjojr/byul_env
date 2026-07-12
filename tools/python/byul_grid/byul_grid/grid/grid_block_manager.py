from pathlib import Path
import json

from collections import OrderedDict, deque

from utils.log_to_panel import g_logger

from PySide6.QtCore import QObject, QRect, Signal, QTimer

from grid.grid_block import GridBlock, BlockMakerThread
from grid.grid_cell import GridCell

import time

from threading import Lock

class GridBlockManager(QObject):
    # loading_block_completed = Signal()
    
    load_block_succeeded = Signal(tuple)

    def __init__(self, block_size=100, max_blocks = 18, max_parallel = 2):
        super().__init__()

        self.block_size = block_size
        self.max_blocks = max_blocks
        self.max_parallel = max_parallel

        self.block_cache: OrderedDict[tuple, GridBlock] = OrderedDict()
        self._cache_lock = Lock()        
        
        self._active_threads: dict[tuple, BlockMakerThread] = {}

        self.loading_queue: deque[tuple] = deque()

        # 중복 제거용 : 큐는 같은 키도 추가한다.
        # 집합으로 중복 값을 확인한다.
        self.loading_set: set[tuple] = set()

        self._pending_timer = False

        self.on_after_block_loaded = None
        self.on_before_block_evicted = None

    def reset(self):
        """모든 블록 상태, 캐시, 쓰레드, 큐를 초기화한다."""
        with self._cache_lock:
            # 블록 캐시 정리
            for key, block in self.block_cache.items():
                try:
                    if self.on_before_block_evicted:
                        self.on_before_block_evicted(key, block)
                    block.close()
                except Exception as e:
                    g_logger.log_debug(f"[GridBlockManager] 블록({key}) close 중 예외: {e}")

            self.block_cache.clear()

        # 쓰레드 정리
        for key, thread in list(self._active_threads.items()):
            try:
                thread.stop()  # BlockMakerThread가 stop() 지원하는 경우
            except Exception as e:
                g_logger.log_debug(f"[GridBlockManager] 쓰레드({key}) 중단 중 예외: {e}")
        self._active_threads.clear()

        # 대기 큐 및 중복 확인 세트 초기화
        self.loading_queue.clear()
        self.loading_set.clear()

        # 타이머 플래그 초기화
        self._pending_timer = False

        g_logger.log_debug("[GridBlockManager] 상태 초기화 완료")

    def get_origin(self, coord:tuple) -> tuple[int,int]:
        return (
            (coord[0] // self.block_size) * self.block_size,
            (coord[1] // self.block_size) * self.block_size
        )

    def get_cell(self, coord:tuple) -> GridCell:
        """
        지정 좌표의 셀을 반환한다.
        블럭 캐시를 확인하여 해당 셀을 가져오고,
        없으면 None
        """
        key = self.get_origin(coord)

        block = self.block_cache.get(key)
        if block:
            dx = coord[0] - key[0]
            dy = coord[1] - key[1]            
            cell = block.cells.get((dx+key[0], dy+key[1]))
            if cell:
                return cell
            
        return None
    
    def set_cell(self, key:tuple, cell:GridCell):
        block = self.block_cache[key]
        block.cells[(cell.x, cell.y)] = cell

    def request_load_block(self, x: int, y: int, interval_msec=5):
        key = self.get_origin((x, y))

        if key in self.block_cache or key in self.loading_set:
            return

        g_logger.log_debug(f'블럭({key[0]}, {key[1]}) 로딩이 큐에 추가됨.')

        self.loading_queue.append(key)
        self.loading_set.add(key)

        if not self._pending_timer:
            self._pending_timer = True
            QTimer.singleShot(interval_msec, self._process_next_block)

    def _process_next_block(self, interval_msec=5):
        self._pending_timer = False

        while (self.loading_queue and
            len(self._active_threads) < self.max_parallel):

            key = self.loading_queue.popleft()
            thread = BlockMakerThread(key[0], key[1], self.block_size)

            thread.succeeded.connect(self._on_load_block_succeeded)
            thread.failed.connect(self._on_load_block_failed)
            thread.finished.connect(lambda key=key: self._finalize_thread(key))

            self._active_threads[key] = thread
            thread.start()

        if self.loading_queue and not self._pending_timer:
            self._pending_timer = True
            QTimer.singleShot(interval_msec, self._process_next_block)

    def after_block_loaded(self, key: tuple, block: GridBlock):
        if self.on_after_block_loaded:
            self.on_after_block_loaded(key)
        pass

    def before_block_evicted(self, key: tuple, block: GridBlock):
        if self.on_before_block_evicted:
            self.on_before_block_evicted(key)
        pass

    def after_block_evicted(self, key: tuple):
        pass
    
    def _on_block_ready(self, key: tuple):
        if not self._pending_timer:
            self._pending_timer = True
            QTimer.singleShot(0, self._process_next_block)

    def _on_load_block_succeeded(self, key: tuple):
        t0 = time.perf_counter()

        with self._cache_lock:
            if key in self.block_cache:
                g_logger.log_debug(f"[load_block] ⚠️ 이미 처리된 key (중복 signal?): {key}")
                return  # ❗ finalize_thread는 finished 시그널로 처리됨

            thread = self._active_threads.get(key)
            if not thread:
                g_logger.log_debug(f"[load_block] ❓ 쓰레드 누락: {key}")
                return

            self.__evict_if_needed(protect_key=key)
            self.block_cache[key] = thread.result
            self.after_block_loaded(key, thread.result)

        self.load_block_succeeded.emit(key)

        t1 = time.perf_counter()
        g_logger.log_debug(f"🎯 load_block_succeeded : {key} : 처리 시간: {(t1 - t0)*1000:.3f}ms")

    def __evict_if_needed(self, 
            protect_key: tuple | None = None, max_remove: int = 1):
        removed = 0
        while len(self.block_cache) > self.max_blocks and removed < max_remove:
            for key in self.block_cache:
                if key == protect_key:
                    continue

                old_block = self.block_cache.pop(key)
                self.before_block_evicted(key, old_block)
                old_block.close()
                self.after_block_evicted(key)

                removed += 1
                break  # 한 번에 하나만 제거하고 while 다시 검사
            else:
                g_logger.log_debug("[evict_block] 🚫 보호 대상 외에 제거할 key 없음")
                break

    def _finalize_thread(self, key: tuple, interval_msec=5):
        self.loading_set.discard(key)

        thread = self._active_threads.pop(key, None)
        if thread:
            thread.deleteLater()  # ❗ 메모리 안전 정리만

        # if not self._pending_timer:
        #     self._pending_timer = True
        #     QTimer.singleShot(interval_msec, self._process_next_block)

    def _on_load_block_failed(self, key: tuple):
        # 중복 처리 방어
        if key not in self._active_threads:
            g_logger.log_debug(
                f"[load_block] ⚠️ 실패 시그널 중복 또는 이미 finalize됨: {key}")
            return

        g_logger.log_debug(f"[load_block] ❌ 실패: {key}")
        self._finalize_thread(key)

    def clear_block_cache(self):
        self.block_cache.clear()

    def set_max_blocks(self, new_max):
        """
        블록 캐시의 최대 개수를 동적으로 조정하고, 초과분 제거
        """
        self.max_blocks = max(1, new_max)  # 최소 1 이상 보장

        while len(self.block_cache) > self.max_blocks:
            old_key, _ = self.block_cache.popitem(last=False)

    def is_inside_block(self, x, y, block_x, block_y):
        """좌표 (x, y)가 키가(block_x, block_y)인 블록의 영역 안에 있는지 확인"""
        return (
            block_x <= x < block_x + self.block_size and
            block_y <= y < block_y + self.block_size
        )

    def is_block_loaded_for(self, x, y):
        return self.get_origin((x, y)) in self.block_cache

    def is_blocks_loaded_for_rect(self, rect: QRect):
        keys = self.get_block_keys_in_rect(rect.left(), rect.top(),
                                       rect.width(), rect.height())
        for key in keys:
            if key not in self.block_cache:
                return False
        return True
    
    def is_blocks_loaded_forward_for_rect(
        self, rect, dx, dy, distance=1, offset=0):

        if dx == 0 and dy == 0:
            return True

        bs = self.block_size

        # 💡 offset으로 rect 확장
        expanded = QRect(
            rect.left() - offset,
            rect.top() - offset,
            rect.width() + offset * 2,
            rect.height() + offset * 2
        )

        for y in range(expanded.top(), expanded.bottom() + 1, bs):
            for x in range(expanded.left(), expanded.right() + 1, bs):
                for i in range(1, distance + 1):
                    fx = x + dx * i * bs
                    fy = y + dy * i * bs

                    if dx == 0:
                        targets = [(fx - bs, fy), (fx, fy), (fx + bs, fy)]
                    elif dy == 0:
                        targets = [(fx, fy - bs), (fx, fy), (fx, fy + bs)]
                    else:
                        targets = [
                            (fx, fy),
                            (fx - dx * bs, fy),
                            (fx, fy - dy * bs),
                            (fx - dx * bs, fy - dy * bs),
                        ]

                    for tx, ty in targets:
                        if not self.is_block_loaded_for(tx, ty):
                            return False

        return True

    def load_blocks_around_for_rect(self, rect: QRect, 
                                    around_range: int = 1, offset: int = 0):
        """
        지정된 rect 영역과 그 주변(around_range + offset) 블럭들 중,
        아직 로딩되지 않은 블럭을 로딩 요청한다.
        """
        # 확장된 영역 계산
        expanded_left = rect.left() - around_range - offset
        expanded_top = rect.top() - around_range - offset
        expanded_width = rect.width() + 2 * (around_range + offset)
        expanded_height = rect.height() + 2 * (around_range + offset)

        block_keys = self.get_block_keys_in_rect(
            expanded_left, expanded_top, expanded_width, expanded_height
        )

        for key in block_keys:
            if key not in self.block_cache:
                self.request_load_block(*key)

    def load_blocks_forward_for_rect(self, rect: QRect, 
                                     dx: int, dy: int, distance=1):
        """
        지정된 rect 범위를 기준으로 (dx, dy) 방향으로 
        1부터 distance 거리까지의 forward 블럭들을 모두 비동기로 로딩한다.
        """
        if dx == 0 and dy == 0:
            return

        bs = self.block_size
        visited = set()

        base_keys = self.get_block_keys_in_rect(
            rect.left(), rect.top(), rect.width(), rect.height())

        for key in base_keys:
            # i = distance
            # fx = bx + dx * i * bs
            # fy = by + dy * i * bs
            bx = key[0]
            by = key[1]

            for i in range(1, distance + 1): 
                fx = bx + dx * i * bs
                fy = by + dy * i * bs

                if dx == 0:
                    targets = [(fx - bs, fy), (fx, fy), (fx + bs, fy)]
                elif dy == 0:
                    targets = [(fx, fy - bs), (fx, fy), (fx, fy + bs)]
                else:
                    targets = [
                        (fx, fy),
                        (fx - dx * bs, fy),
                        (fx, fy - dy * bs),
                        (fx - dx * bs, fy - dy * bs),
                    ]

                for tx, ty in targets:
                    key = self.get_origin((tx, ty))
                    if key not in visited:
                        visited.add(key)
                        self.request_load_block(tx, ty)

    def get_block_keys_in_rect(self, start_x, start_y, width, height):
        """
        주어진 범위를 포함하는 모든 블럭의 키(bx, by)를 집합으로 반환한다.
        """
        end_x = start_x + width - 1
        end_y = start_y + height - 1

        start = self.get_origin((start_x, start_y))
        bx_start = start[0]
        by_start = start[1]

        end = self.get_origin((end_x, end_y))
        bx_end = end[0]
        by_end = end[1]

        result_keys = set()

        for by in range(by_start, by_end + 1, self.block_size):
            for bx in range(bx_start, bx_end + 1, self.block_size):
                result_keys.add((bx, by))

        return result_keys

    def get_block_keys_in_rect_only_loaded(self, start_x, start_y, width, height):
        """
        주어진 범위에 걸친 블럭들 중, block_cache에 로드된 블럭 키만 집합으로 반환한다.
        """
        end_x = start_x + width - 1
        end_y = start_y + height - 1

        start = self.get_origin((start_x, start_y))
        bx_start = start[0]
        by_start = start[1]

        end = self.get_origin((end_x, end_y))
        bx_end = end[0]
        by_end = end[1]

        result_keys = set()

        for by in range(by_start, by_end + 1, self.block_size):
            for bx in range(bx_start, bx_end + 1, self.block_size):
                key = (bx, by)
                if key in self.block_cache:
                    result_keys.add(key)

        return result_keys

    def save_cells_to_blocks(self, cells, grid_block_dir='./'):
        blocks = {}
        block_sizes = {}
        fixed_block_size = (self.block_size, self.block_size)
        for (x, y), cell in cells.items():
            key = self.get_origin((x, y))
            if key not in blocks:
                path = Path(grid_block_dir) / f"block_{key[0]}_{key[1]}.json"

                if path.exists():
                    with open(path, "r", encoding="utf-8") as f:
                        existing = json.load(f)
                    block_size_x = existing.get("width", self.block_size)
                    block_size_y = existing.get("height", self.block_size)

                    if (block_size_x, block_size_y) != fixed_block_size:
                        raise ValueError(
"Block size mismatch in {}: expected {}, got ({}, {})".format(
    path, fixed_block_size, block_size_x, block_size_y)
                    )
                    block_sizes[key] = fixed_block_size
                    cell_dict = {
                        (c["x"], c["y"]): GridCell.from_dict(c)
                        for c in existing["cells"]
                    }
                    blocks[key] = cell_dict

                else:
                    block_sizes[key] = fixed_block_size
                    blocks[key] = {}
                    
            blocks[key] = cell

        Path(self.grid_block_dir).mkdir(parents=True, exist_ok=True)
        for key, cell_dict in blocks.items():
            bx = key[0]
            by = key[1]
            bw, bh = block_sizes[(bx, by)]
            block_data = GridBlock(bx, by, bw, bh, 
                [cell.to_dict() for cell in cell_dict.values()])
            
            path = Path(self.grid_block_dir) / f"block_{bx}_{by}.json"
            with open(path, "w", encoding="utf-8") as f:
                json.dump(block_data.to_json(), f, indent=4)