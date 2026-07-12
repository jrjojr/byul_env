from route import RouteDir, c_route
from coord import c_coord
from dataclasses import dataclass

@dataclass
class NpcPos:
    cur_coord: tuple  # 셀 단위 절대 좌표
    disp_dx: float = 0.0  # -1.0 ~ 1.0 비율 단위
    disp_dy: float = 0.0  # -1.0 ~ 1.0 비율 단위
    offset_x: int = 0
    offset_y: int = 0

    def get_pos(self) -> tuple[float, float]:
        """
        현재 셀 위치(cur_coord) + 보간값(disp_dx/dy)을 더해
        부드러운 위치(float)를 반환한다.
        offset_x/y는 픽셀 기반 좌표에서는 렌더링 시점에만 사용한다.
        """
        x = self.cur_coord[0] + self.disp_dx
        y = self.cur_coord[1] + self.disp_dy
        return (x, y)
    
    def get_coord(self, tolerance: float = 1e-4) -> tuple[int, int]:
        """
        disp_dx/dy 보간 상태가 ±1.0에 충분히 가까우면
        다음 셀로 이동한 것으로 판단하고 해당 좌표를 반환한다.

        Args:
            tolerance (float): 도달 판정 공차. 기본값은 0.0001.

        Returns:
            tuple[int, int]: 현재 좌표 또는 다음 셀 좌표
        """
        x, y = self.cur_coord

        # 보간값이 ±1.0에 가까우면 방향에 따라 셀 이동
        if self.disp_dx >= 1.0 - tolerance:
            x += 1
        elif self.disp_dx <= -1.0 + tolerance:
            x -= 1

        if self.disp_dy >= 1.0 - tolerance:
            y += 1
        elif self.disp_dy <= -1.0 + tolerance:
            y -= 1

        return (x, y)
    
    def set_disp_dx(self, val:float):
        self.disp_dx = val

    def set_disp_dy(self, val:float):
        self.disp_dy = val
        
    def to_pixel(self, cell_size: int) -> tuple:
        px = (self.cur_coord[0] + self.disp_dx) * cell_size + self.offset_x
        py = (self.cur_coord[1] + self.disp_dy) * cell_size + self.offset_y
        return px, py

    def update_disp(self, dx_ratio: float, dy_ratio: float):
        self.disp_dx = dx_ratio
        self.disp_dy = dy_ratio

    def move_to(self, new_coord: tuple):
        """
        위치를 새 좌표로 고정하고, 보간값도 0.0으로 초기화한다.
        즉시 위치 변경용 (강제 이동 포함).
        """
        self.cur_coord = new_coord
        self.disp_dx = 0.0
        self.disp_dy = 0.0

    def snap_to(self, next_coord: tuple, tolerance: float = 1e-4):
        """
        보간 상태가 ±1.0에 가까우면 cur_coord를 이동 방향으로 1칸 이동한다.
        next_coord는 방향 유추용으로만 사용된다.
        """
        start = c_coord.from_tuple(self.cur_coord)
        end = c_coord.from_tuple(next_coord)
        direction = c_route.calc_direction(start, end)
        delta = c_route.direction_to_coord(direction)  # 방향 벡터 (dx, dy)

        moved = False

        if abs(1.0 - abs(self.disp_dx)) < tolerance and delta.x != 0:
            self.cur_coord = (self.cur_coord[0] + delta.x, self.cur_coord[1])
            moved = True

        if abs(1.0 - abs(self.disp_dy)) < tolerance and delta.y != 0:
            self.cur_coord = (self.cur_coord[0], self.cur_coord[1] + delta.y)
            moved = True

        if moved:
            self.disp_dx = 0.0
            self.disp_dy = 0.0

    def mirrored_pos_from(self, direction: RouteDir) -> "NpcPos":
        """
        현재 위치와 보간값을 기준으로, 
        주어진 방향으로 이동했을 때의 
        다음 셀에서의 반대 보간값을 계산하여 NpcPos를 반환한다.
        disp_dx, disp_dy는 비율(-1.0 ~ 1.0) 단위로 처리된다.
        """
        direction_map = {
            RouteDir.LEFT: (-1, 0), RouteDir.RIGHT: (1, 0),
            RouteDir.UP: (0, -1), RouteDir.DOWN: (0, 1),
            RouteDir.UP_LEFT: (-1, -1), RouteDir.UP_RIGHT: (1, -1),
            RouteDir.DOWN_LEFT: (-1, 1), RouteDir.DOWN_RIGHT: (1, 1),
        }
        dx, dy = direction_map[direction]

        mirror_x = 1.0 - self.disp_dx if dx != 0 else self.disp_dx
        mirror_y = 1.0 - self.disp_dy if dy != 0 else self.disp_dy

        next_x = self.cur_coord[0] + dx
        next_y = self.cur_coord[1] + dy

        return NpcPos(
            cur_coord=(next_x, next_y),
            disp_dx=mirror_x,
            disp_dy=mirror_y
        )

    def mirrored_pos_to(self, next_coord: tuple) -> "NpcPos":
        """
        현재 보간 상태를 기준으로, 
        명시된 다음 좌표에서 보간 상태를 반영한 NpcPos를 생성한다.

        좌표 변화량이 있는 축(x 또는 y)에 대해서만 보간 비율을 반전하여 적용하며,
        disp_dx, disp_dy는 비율(-1.0 ~ 1.0) 단위로 저장된다.
        """
        dx = next_coord[0] - self.cur_coord[0]
        dy = next_coord[1] - self.cur_coord[1]

        mirror_x = 1.0 - self.disp_dx if dx != 0 else self.disp_dx
        mirror_y = 1.0 - self.disp_dy if dy != 0 else self.disp_dy

        return NpcPos(
            cur_coord=next_coord,
            disp_dx=mirror_x,
            disp_dy=mirror_y
        )

    def set_offset(self, x: int, y: int):
        self.offset_x = x
        self.offset_y = y
