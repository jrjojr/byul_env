from route import RouteDir
from route import c_route

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from world.npc.npc import NPC  # 순환 참조 방지용 타입 힌트
    from world.world import World

class DirectionalAnimator:
    def __init__(self, npc:'NPC',
        on_complete_cb=None, on_tick_cb=None, on_start_cb=None, 
        cell_size=100):

        self.on_complete_cb = on_complete_cb
        self.on_tick_cb = on_tick_cb
        self.on_start_cb = on_start_cb
        self.cell_size = cell_size
        self.total_elapsed_sec = 0.0
        self.is_running = False
        self.npc = npc
        self.direction = npc.direction
        self.next = c_route.direction_to_coord(self.direction)

    def is_anim_started(self):
        return self.is_running

    def get_cell_size(self):
        return self.cell_size

    def set_cell_size(self, cell_size: int):
        self.cell_size = cell_size

    def start(self, direction:RouteDir):
        """애니메이션 시작 (tick 호출 필요)"""
        self.is_running = True
        self.on_start_cb()
        self.direction = direction

    def tick(self, elapsed_sec: float):
        """tick 기반 애니메이션 동작"""
        if not self.is_running or not self.npc or not self.world:
            return

        npc = self.npc

        self.total_elapsed_sec += elapsed_sec
        if self.total_elapsed_sec < npc.start_delay_sec:
            return

        arrived = self._step_ratio(elapsed_sec)

        self.on_tick_cb()

        if arrived:
            self.is_running = False
            self.on_complete_cb()

    def _step_ratio(self, elapsed_sec: float, tolerance: float = 1e-4) -> bool:
        """
        NPC의 disp_dx/dy를 목표 값까지 직접 보간하여 이동한다.
        
        Returns:
            bool: x, y가 모두 목표 위치에 도달하면 True
        """
        grid_unit_m = self.world.grid_unit_m
        speed_cells_per_sec = self.npc.speed_kmh * 1000 / 3600.0 / grid_unit_m
        delta = speed_cells_per_sec * elapsed_sec

        # 현재 보간 상태
        rel_x = self.npc.pos.disp_dx
        rel_y = self.npc.pos.disp_dy

        # 목표값
        target_x = self.next[0]
        target_y = self.next[1]

        # X축 보간
        dx = target_x - rel_x
        if abs(dx) <= delta + tolerance:
            rel_x = target_x
            x_done = True
        else:
            rel_x += delta if dx > 0 else -delta
            x_done = False

        # Y축 보간
        dy = target_y - rel_y
        if abs(dy) <= delta + tolerance:
            rel_y = target_y
            y_done = True
        else:
            rel_y += delta if dy > 0 else -delta
            y_done = False

        self.npc.pos.update_disp(rel_x, rel_y)
        return x_done and y_done
