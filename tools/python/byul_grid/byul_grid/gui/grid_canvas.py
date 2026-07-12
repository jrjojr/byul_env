# 리팩터링된 GridCanvas - View 전용, 오프스크린 렌더링 최적화
from PySide6.QtWidgets import QWidget, QToolTip
from PySide6.QtGui import (
    QPainter, QColor, QFont, QPixmap, QMouseEvent, QWheelEvent, 
    QCursor, QPen, QBrush
)
from PySide6.QtCore import (
    Qt, QPoint, Signal, Slot, QTimer, QRect, QEvent, QLine
)

from grid.grid_cell import GridCell, CellStatus, CellFlag, TerrainType
from world.village.village import Village
from world.world import World

from byul_wrapper.route import RouteDir
import time
from collections import deque
from utils.log_to_panel import g_logger
from utils.mouse_input_handler import MouseInputHandler

from world.world import World, FIRST_NPC_ID
from world.village.village import Village
from world.npc.npc import NPC

from pathlib import Path

from utils.image_manager import ImageManager

from utils.route_changing_detector import RouteChangingDetector

class GridCanvas(QWidget):
    '''GridCanvas는 사용자와의 상호 작용을 담당하며,
       오프스크린 렌더링을 통해 성능을 최적화한다.'''
    grid_changed = Signal(int, int)
    center_changed = Signal(int, int)

    cell_size_changed = Signal(int)
    key_pressed = Signal()
    key_released = Signal()

    draw_cells_elapsed = Signal(float)
    draw_cells_started = Signal(float)
    draw_cells_ended = Signal(float)

    click_mode_changed = Signal(str)

    interval_sec_changed = Signal(int)

    tick_elapsed = Signal(float)

    npc_selected = Signal(NPC)


    def __init__(self, world:World, interval_sec=0.03, min_px=30, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.world = world

        self.grid_width = 11
        self.grid_height = 11

        self.center_x = 0
        self.center_y = 0
        self.set_center(0,0)

        # self.cell_size = 80
        self.set_cell_size(80)
        self.min_px = min_px

        self.min_size_for_text = 50
        self.setMinimumSize(500, 500)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)

        self._pressed_keys = set()

        self.cached_pixmap = None
        self.needs_redraw = False

        self.default_empty_cell_color = QColor(30, 30, 30)

        self.interval_sec = interval_sec
        self.logic_timer = QTimer(self)
        self.logic_timer.timeout.connect(self._tick)
        self.set_interval_sec(interval_sec)
        self.logic_timer.start()

        self.wheel_timer = QTimer()
        self.wheel_timer.setSingleShot(True)
        self.wheel_timer.timeout.connect(self.change_grid_from_window)

        self.center_changed.connect(self.request_redraw)

        self.click_mode = "select_npc"

        # 예: 초기화 시 핸들러 연결
        self.mouse_handler = MouseInputHandler(self)
        self.mouse_handler.clicked.connect(self._on_clicked)
        self.mouse_handler.mouse_moved.connect(self._on_mouse_moved)
        self.mouse_handler.double_clicked.connect(self._on_dbl_clicked)
        self.last_mouse_pos = QPoint()

        # move_center에서 사용한다 현재 진행방향을 확인하기 위해
        self.route_detector = RouteChangingDetector()        

        self._move_queue = deque()
        self._move_timer = QTimer()
        self._move_timer.setInterval(100)  # 이동 주기: 100ms (필요시 조절)
        self._move_timer.timeout.connect(self._dequeue_move)
        self._move_timer.start()        

        if world.npc_mgr.has_npc(FIRST_NPC_ID):
            self.m_selected_npc = world.npc_mgr.get_npc(FIRST_NPC_ID)

    @property
    def selected_npc(self):
        return self.m_selected_npc
    
    @selected_npc.setter
    def selected_npc(self, npc:NPC):
        self.m_selected_npc = npc
        self.npc_selected.emit(npc)


    @Slot(int)
    def set_interval_sec(self, sec: int):
        sec = max(0, sec)  # 음수면 0으로 고정
        self._interval_sec = sec

        if self.logic_timer is not None:
            self.logic_timer.setInterval(sec/1000)

        self.interval_sec_changed.emit(sec)

    def request_redraw(self):
        self.needs_redraw = True

    def showEvent(self, event):
        super().showEvent(event)
        QTimer.singleShot(1000, self.change_grid_from_window)

    def spawn_npc_at(self, coord:tuple):
        npc_id = NPC.generate_random_npc_id()
        npc = self.world.spawn_npc(npc_id, coord)
        if npc is None:
            g_logger.log_debug(f'spawn_npc({npc_id}) 실패했다')

        if self.selected_npc is None:
            if npc.id == FIRST_NPC_ID:
                self.selected_npc = npc            

        g_logger.log_always(f"NPC 추가됨: at ({coord[0]},{coord[1]})")            

    def despawn_npc_at(self, coord:tuple):
        cell = self.world.block_mgr.get_cell(coord)
        if cell:
            if cell.npc_ids:
                npc_id = cell.npc_ids[0]
                npc = self.world.npc_mgr.get_npc(npc_id)
                if npc:
                    if npc == self.selected_npc:
                        self.selected_npc = None
                        
                    self.world.delete_npc(npc_id)
                    g_logger.log_always(
                        f"NPC 제거됨: {npc_id} at ({coord[0]},{coord[1]})")
       
    def enterEvent(self, event):
        if not self.hasFocus():
            self.setFocus(Qt.OtherFocusReason)
            g_logger.log_debug('GridCanvas가 잃었던 포커스를 다시 회복했다')
        super().enterEvent(event)

    def resizeEvent(self, event):
        self.change_grid_from_window()

    def paintEvent(self, event):
        painter = QPainter(self)
        if self.cached_pixmap:
            painter.drawPixmap(0, 0, self.cached_pixmap)

    def get_center(self)->tuple[int,int]:
        return (self.center_x, self.center_y)
    
    def set_center(self, gx, gy):
        # 좌표(gx, gy)를 센터로 이동한다.
        g_logger.log_always(
            f'좌표({gx}, {gy})를 센터로 설정한다.')
        
        self.center_x = gx
        self.center_y = gy

        # self.update_grid()

        self.center_changed.emit(gx, gy)

    def move_center(self, dx: int, dy: int, distance=1):
        if dx == 0 and dy == 0:
            return
        self._move_queue.append((dx, dy, distance))

    def _dequeue_move(self):
        if not self._move_queue:
            return

        dx, dy, distance = self._move_queue.popleft()
        self.real_move(dx, dy, distance)

    def real_move(self, dx: int, dy: int, distance=1):
        cx, cy = self.get_center()
        new_x = cx + dx
        new_y = cy + dy

        g_logger.log_debug(
            f'센터 이동 : 현재=({cx}, {cy}) '
            f'→ 이동량=({dx}, {dy}) → 목표=({new_x}, {new_y})'
        )

        if self.route_detector.has_changed((cx, cy), (new_x, new_y)):
            self._move_reason = "changed"
        else:
            self._move_reason = "continue"

        self._target_step = 1
        self.center_x = new_x
        self.center_y = new_y

        min_x = new_x - (self.grid_width // 2)
        min_y = new_y - (self.grid_height // 2)

        rect = QRect(min_x, min_y, self.grid_width, self.grid_height)

        if not self.world.block_mgr.is_blocks_loaded_forward_for_rect(rect, dx, dy, distance):
            self.world.block_mgr.load_blocks_forward_for_rect(rect, dx, dy, distance)

        self.center_changed.emit(new_x, new_y)

    def update_grid(self):
           
        x0 = self.center_x - (self.grid_width // 2)
        y0 = self.center_y - (self.grid_height // 2)
        rect = QRect(x0, y0, self.grid_width, self.grid_height)

        if not self.world.block_mgr.is_blocks_loaded_for_rect(rect):
            self.world.block_mgr.load_blocks_around_for_rect(rect)

    def _tick(self):
        now = time.time()
        if not hasattr(self, "_last_tick_time") or self._last_tick_time is None:
            self._last_tick_time = now
            return

        elapsed_sec = now - self._last_tick_time
        self._last_tick_time = now

        self.move_from_keys(self._pressed_keys)

        min_x = self.center_x - (self.grid_width // 2)
        min_y = self.center_y - (self.grid_height // 2)
        rect = QRect(min_x, min_y, self.grid_width, self.grid_height)
        npcs = self.world.get_npcs_in_rect(rect)
        for npc in npcs:
            npc.on_tick(elapsed_sec, self.cell_size)

        self.update_grid()

        # if self.needs_redraw:
        #     # self.draw_cells_and_npcs(npcs)
        #     self.draw_cells()
        #     self.needs_redraw = False

        self.draw_cells()        

        if g_logger.debug_mode:
            self.tick_elapsed.emit(elapsed_sec * 1000)
        
        self.update()

    def draw_cells(self):
        if g_logger.debug_mode:
            t0 = time.time()
            self.draw_cells_started.emit(t0)

        w, h = self.width(), self.height()
        self.cached_pixmap = QPixmap(w, h)
        self.cached_pixmap.fill(Qt.darkGray)

        painter = QPainter(self.cached_pixmap)
        painter.setFont(QFont("Courier", 8))

        font = QFont("Courier", 10)
        pen_black = QPen(Qt.black)
        brush_empty = QBrush(self.default_empty_cell_color)

        min_x = self.center_x - (self.grid_width // 2)
        min_y = self.center_y - (self.grid_height // 2)

        for y in range(self.grid_height):
            for x in range(self.grid_width):
                gx = min_x + x
                gy = min_y + y
                # key = self.world.block_mgr.get_origin(gx, gy)
                px, py = self.convert_pos_grid_to_win(x, y)

                # ox, oy = self.world.block_mgr.get_origin(gx, gy)
                cell = self.world.block_mgr.get_cell((gx, gy))

                if cell is None:
                    painter.setBrush(brush_empty)
                    painter.drawRect(px, py, self.cell_size, self.cell_size)
                    continue

                image = None
                npc = None
                if cell.status == CellStatus.NPC:
                    for npc_id in cell.npc_ids:
                        if self.world.npc_mgr.has_npc(npc_id):
                            npc = self.world.npc_mgr.get_npc(npc_id)
                        # else:
                        #     npc = self.world.spawn_npc(npc_id, (gx, gy))
                        #     pass
                        if npc:
                            if npc.anim_started:
                                image = ImageManager.get_empty_image()
                            else:
                                image = npc.get_image()

                            win_pos_x, win_pos_y = self.get_win_pos_at_coord(npc.start)

                            if win_pos_x and win_pos_y:
                                npc.draw( painter, win_pos_x, win_pos_y, self.cell_size)   

                elif cell.status == CellStatus.EMPTY:
                    image = ImageManager.get_empty_image()

                if self.selected_npc:
                    coord = (gx, gy)
                    
                    if not cell.terrain in self.selected_npc.movable_terrain:
                        image = ImageManager.get_obstacle_for_npc_image()

                    if cell.has_flag(CellFlag.ROUTE):
                        # index = self.selected_npc.cur_index
                        # image = self.selected_npc.get_proto_image(index)
                        image = self.selected_npc.get_proto_route_image(coord)

                    if cell.has_flag(CellFlag.GOAL):
                        image = ImageManager.get_goal_image()

                if image:
                    painter.drawPixmap(
                        px, py, self.cell_size, self.cell_size, image)

                if self.cell_size > self.min_size_for_text:
                    painter.setPen(pen_black)
                    painter.setFont(font)
                    painter.drawText(px, py, self.cell_size, self.cell_size, 
                                    Qt.AlignCenter, cell.text())

        if self.selected_npc:
            npc_start = self.selected_npc.start
            win_pos_x, win_pos_y = self.get_win_pos_at_coord(npc_start)
            if win_pos_x and win_pos_y:
                self.draw_selected_npc(painter, win_pos_x, win_pos_y)

        if self.last_mouse_pos:
            self.draw_hover_cell(painter, self.last_mouse_pos, 120)

        painter.end()

        if g_logger.debug_mode:
            t1 = time.time()
            elapsed = (t1 - t0) * 1000
            self.draw_cells_elapsed.emit(elapsed)       

    # 나머지: 입력 처리, hover 표시, 클릭 처리 등은 원래 코드 유지
    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_Escape:
            if self.window().isFullScreen():
                self.window().toggle_fullscreen()
                return
        elif key == Qt.Key_Space:
            cell = self.get_cell_at_win_pos(
                self.last_mouse_pos.x(), self.last_mouse_pos.y())
            if cell:
                c = (cell.x, cell.y)
                self.world.toggle_obstacle(c, self.selected_npc)

        # print(f"[KEY] {event.key()}, focus: {self.hasFocus()}")            

        # 나머지는 일반 키로 취급
        self._pressed_keys.add(key)
        self.key_pressed.emit()

        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        self._pressed_keys.discard(event.key())
        # self._move_queue.clear()  # 🎯 큐 전부 제거             
        self._preserve_queue_tail(1)  # 예: 마지막 2개만 남김           
        self.key_released.emit()        

    def _preserve_queue_tail(self, keep_count: int):
        # 현재 큐가 너무 작으면 그대로 둠
        if len(self._move_queue) <= keep_count:
            return
        # 큐의 마지막 N개만 유지
        self._move_queue = deque(list(self._move_queue)[-keep_count:])


    # 마우스 이동 시 위치 저장 후 업데이트
    def _on_mouse_moved(self, event: QMouseEvent):
        self.last_mouse_pos = event.position().toPoint()
        self.request_redraw()
        # self.update()

    def focusOutEvent(self, event):
        self._pressed_keys.clear()
        super().focusOutEvent(event)

    def event(self, event):
        if event.type() == QEvent.WindowDeactivate:
            self._pressed_keys.clear()
        return super().event(event)

    def _on_clicked(self, event: QMouseEvent):
        pos = event.position().toPoint()

        if event.button() == Qt.LeftButton:
            self._handle_left_click(pos)

        elif event.button() == Qt.RightButton:
            if event.modifiers() & Qt.ShiftModifier:
                self._handle_shift_right_click(pos)
            else:
                self._handle_right_click(pos)

        elif event.button() == Qt.MiddleButton:
            cell = self.get_cell_at_win_pos(pos.x(), pos.y())
            if cell:
                g_logger.log_always(
                    f"중간 클릭 at {pos.x()}, {pos.y()}, 셀 키 ({cell.x},{cell.y})")
                self.set_center(cell.x, cell.y)

    def _on_dbl_clicked(self, event: QMouseEvent):
        if event.button() == Qt.LeftButton:
            # 더블 클릭 위치
            pos = event.position().toPoint()
            cell = self.get_cell_at_win_pos(pos.x(), pos.y())
            if cell:
                g_logger.log_always(f"더블 클릭 at {pos.x()}, {pos.y()}, "
                                    f"셀 키 ({cell.x},{cell.y})")    
        # self.update()

    def wheelEvent(self, event: QWheelEvent):
        delta = event.angleDelta().y()
        scale = 1.1 if delta > 0 else 1 / 1.1
        new_size = max(self.min_px, min(self.cell_size * scale, 
                               min(self.width(), self.height())))
        
        self.set_cell_size(int(new_size))

        self.wheel_timer.start(50)

        # self.update()

    def convert_pos_grid_to_win(self, cx, cy):
        margin_x = (self.width() - self.grid_width * self.cell_size) // 2
        margin_y = (self.height() - self.grid_height * self.cell_size) // 2
        return margin_x + cx * self.cell_size, margin_y + cy * self.cell_size

    def convert_pos_win_to_grid(self, x, y):
        margin_x = (self.width() - self.grid_width * self.cell_size) // 2
        margin_y = (self.height() - self.grid_height * self.cell_size) // 2
        return (x - margin_x) // self.cell_size, \
            (y - margin_y) // self.cell_size

    def change_grid_from_window(self):
        w = self.width()
        h = self.height()
        self.grid_width = ((w - self.cell_size) // self.cell_size | 1)
        self.grid_height = ((h - self.cell_size) // self.cell_size | 1)

        self.grid_changed.emit(self.grid_width, self.grid_height)
        self.request_redraw()

    def draw_hover_cell(self, painter: QPainter, pos: QPoint, cell_size=80):

        # mouse_pos = self.mapFromGlobal(QCursor.pos())

        # x = mouse_pos.x()
        # y = mouse_pos.y()

        x, y = pos.x(), pos.y()

        canvas_width = self.width()
        canvas_height = self.height()

        cell = self.get_cell_at_win_pos(x, y)
        if not cell:
            return


        # 셀 사각형 기본 위치는 마우스 위치 기준
        # rect_x = (x // cell_size) * cell_size
        # rect_y = (y // cell_size) * cell_size

        rect_x = x
        rect_y = y
        

        # 오른쪽/아래쪽 경계에 가까우면 왼쪽/위쪽으로 이동
        if rect_x + cell_size > canvas_width:
            rect_x = max(0, canvas_width - cell_size)
        if rect_y + cell_size > canvas_height:
            rect_y = max(0, canvas_height - cell_size)

        rect = QRect(rect_x, rect_y, cell_size, cell_size)

        # 배경: 반투명 검정
        painter.setBrush(QColor(0, 0, 0, 127))
        painter.setPen(Qt.NoPen)
        painter.drawRect(rect)

        # 텍스트: 흰색
        painter.setPen(Qt.white)
        painter.setFont(QFont("Arial", 10))
        painter.drawText(rect, Qt.AlignCenter, cell.text())

    def get_cell_at_mouse(self):
        pos = self.last_mouse_pos
        cx, cy = self.convert_pos_win_to_grid(pos.x(), pos.y())
        center_x, center_y = self.get_center()

        if not (0 <= cx < self.grid_width and 0 <= cy < self.grid_height):
            return None  # 마우스가 그리드 바깥이면 None

        gx = center_x - self.grid_width // 2 + cx
        gy = center_y - self.grid_height // 2 + cy
        return self.world.block_mgr.get_cell((gx, gy))  # 셀이 존재하지 않으면 자동으로 None
   
    def get_cell_at_win_pos(self, x, y):
        cx, cy = self.convert_pos_win_to_grid(x, y)
        cx = min(max(cx, 0), self.grid_width - 1)
        cy = min(max(cy, 0), self.grid_height - 1)

        center_x, center_y = self.get_center()
        gx = center_x - self.grid_width // 2 + cx
        gy = center_y - self.grid_height // 2 + cy

        return self.world.block_mgr.get_cell((gx, gy))

    def get_cell_at_grid(self, grid_x:int, grid_y:int):
        center_x, center_y = self.get_center()
        if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
            gx = center_x - self.grid_width // 2 + grid_x
            gy = center_y - self.grid_height // 2 + grid_y
            return self.world.block_mgr.get_cell((gx, gy))
        return None
    
    def get_grid_at_cell(self, cell: GridCell):
        cx, cy = cell.x, cell.y
        center_x, center_y = self.get_center()

        grid_x = cx - (center_x - self.grid_width // 2)
        grid_y = cy - (center_y - self.grid_height // 2)

        # 바깥이면 가장 가까운 경계로 클램프
        grid_x = min(max(grid_x, 0), self.grid_width - 1)
        grid_y = min(max(grid_y, 0), self.grid_height - 1)

        return grid_x, grid_y

    def get_grid_at_coord(self, coord:tuple):
        cx, cy = coord[0], coord[1]
        center_x, center_y = self.get_center()

        grid_x = cx - (center_x - self.grid_width // 2)
        grid_y = cy - (center_y - self.grid_height // 2)

        if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
            return grid_x, grid_y
        return None, None
    
    def get_win_pos_at_grid(self, grid_x:int, grid_y:int):
        pos_x, pos_y = self.convert_pos_grid_to_win(grid_x, grid_y)
        return pos_x, pos_y
    
    def get_win_pos_at_coord(self, coord:tuple):
        gx, gy = self.get_grid_at_coord(coord)

        if gx == None or gy == None:
            return None, None
        
        return self.get_win_pos_at_grid(gx, gy)
    
    @Slot(int)
    def set_cell_size(self, cell_size:int):
        self.cell_size = cell_size
        # for npc in self.world.npc_mgr.npc_dict.values():
        #     npc.set_cell_size(cell_size)

        self.change_grid_from_window()

        self.cell_size_changed.emit(cell_size)


    def _handle_left_click(self, pos:QPoint):
        x = pos.x()
        y = pos.y()
        cell = self.get_cell_at_win_pos(x, y)
        if not cell:
            g_logger.log_always(f'현재 win위치({x}, {y})에 셀이 없다.')
            return 
        
        gx = cell.x
        gy = cell.y

        coord = (gx, gy)
        if self.click_mode == "select_npc":
            npc = self.get_first_npc_in_cell(cell)
            if npc:
                self.selected_npc = npc
                g_logger.log_always(f"✅ 선택된 NPC: {npc.id}")
            else:
                self.selected_npc = None                
                g_logger.log_always("⚠️ 해당 셀에 NPC가 없습니다.")
        
        elif self.click_mode == "spawn_npc_at":
            self.spawn_npc_at(coord)

        elif self.click_mode == "despawn_npc_at":
            self.despawn_npc_at(coord)

        elif self.click_mode == "obstacle":
            g_logger.log_debug(f"장애물 추가: ({gx}, {gy})")            
            self.world.add_obstacle(coord)

    def _handle_right_click(self, pos:QPoint):
        x = pos.x()
        y = pos.y()
        # gx, gy = self.convert_pos_win_to_grid(x, y)
        cell = self.get_cell_at_win_pos(x, y)
        if not cell:
            g_logger.log_always(f'현재 win위치({x}, {y})에 셀이 없다.')
            return 
                    
        gx = cell.x
        gy = cell.y
        
        coord = (gx, gy)

        if self.click_mode == "select_npc":
            if self.selected_npc:
                g_logger.log_always(f"🔴 목표 위치 설정: ({gx}, {gy})")
                self.world.set_goal(self.selected_npc, coord)
            
            else:
                g_logger.log_always("⚠️ 현재 선택된 NPC가 없습니다.")

        elif self.click_mode == "obstacle":
            g_logger.log_debug(f"장애물 제거: ({gx}, {gy})")
            self.world.remove_obstacle(coord)

    def _handle_shift_right_click(self, pos:QPoint):
        x = pos.x()
        y = pos.y()
        # gx, gy = self.convert_pos_win_to_grid(x, y)
        cell = self.get_cell_at_win_pos(x, y)
        if not cell:
            g_logger.log_always(f'현재 win위치({x}, {y})에 셀이 없다.')
            return 
                
        gx = cell.x
        gy = cell.y
        
        coord = (gx, gy)

        if self.click_mode == "select_npc":
            if self.selected_npc:
                g_logger.log_always(f"🔴 목표 위치 추가: ({gx}, {gy})")
                self.world.append_goal(self.selected_npc, coord)
            else:
                g_logger.log_always("⚠️ 현재 선택된 NPC가 없습니다.")

    @Slot(str)
    def set_click_mode(self, mode: str):
        g_logger.log_debug(f"🛠️ 클릭 모드 전환됨: {mode}")        
        self.click_mode = mode
        self.click_mode_changed.emit(mode)

    def move_from_keys(self, pressed_keys):
        # g_logger.log_debug(f'move_From_keys는 동작하나? : {pressed_keys}')
        dx = dy = 0
        if Qt.Key_Left in pressed_keys:
            dx -= 1
        if Qt.Key_Right in pressed_keys:
            dx += 1
        if Qt.Key_Up in pressed_keys:
            dy -= 1
        if Qt.Key_Down in pressed_keys:
            dy += 1
        self.move_center(dx, dy)
    
    def get_first_npc_in_cell(self, cell: GridCell) -> NPC | None:
        if not cell.npc_ids:
            return None
        first_id = cell.npc_ids[0]
        return self.world.npc_mgr.npc_dict.get(first_id)

    def draw_selected_npc(self, painter: QPainter, 
        win_pos_x:int, win_pos_y:int):
        '''실제 디바이스에 이미지를 그린다.
        '''
        x = win_pos_x + self.selected_npc.draw_offset_x + int(self.selected_npc.disp_dx)
        y = win_pos_y + self.selected_npc.draw_offset_y + int(self.selected_npc.disp_dy)

        # 배경: 반투명 검정
        # rect = QRect(x, y, self.m_cell_size, self.m_cell_size)
        # painter.setBrush(QColor(0, 0, 0, 127))
        # painter.setPen(Qt.NoPen)
        # painter.drawRect(rect)

        image = self.selected_npc.get_selected_npc_image()

        painter.drawPixmap(
                x, y, self.cell_size, self.cell_size, image)

    def clear_proto_flags(self):
        for block in self.world.block_mgr.block_cache.values():
            for cell in block.cells.values():
                cell.remove_flag(CellFlag.ROUTE)
