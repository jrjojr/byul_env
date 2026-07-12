# npc.py
#
# Copyright (c) 2025 별이아빠 (byuldev@outlook.kr)
# This file is part of the Byul World project.
# Licensed under the Byul World 공개 라이선스 v1.0.
# See LICENSE file for details.

from ffi_core import ffi, C
from coord import c_coord
from navgrid import c_navgrid
from dstar_lite import c_dstar_lite

from pathlib import Path

from PySide6.QtGui import QPixmap, QPainter, QColor
from PySide6.QtCore import QPoint, QRect, Qt, QTimer, QObject, Signal, Slot

from route import c_route, RouteDir
from coord_list import c_coord_list

from world.village.village import Village
from grid.grid_cell import TerrainType, GridCell, CellStatus

from utils.log_to_panel import g_logger

from queue import Queue, Empty

import random

import uuid
import math

from utils.image_manager import ImageManager

from threading import Thread

import copy

class NPC(QObject):
    anim_to_started_sig = Signal(tuple)
    anim_to_arrived_sig = Signal(tuple)
    disp_dx_changed = Signal(float)
    disp_dy_changed = Signal(float)

    start_changed_sig = Signal(tuple)
    goal_changed_sig = Signal(tuple)
    speed_kmh_changed = Signal(float)
    start_delay_sec_changed = Signal(float)
    max_retry_changed = Signal(int)
    route_capacity_changed = Signal(int)

    def __init__(self, npc_id: str, world, start:tuple=None, 
                 speed_kmh:float=4.0, start_delay_sec=0.5, 
                 route_capacity=100, 
                 max_retry = 1000, 
                 influence_range = 0,
                 max_range = 10,
                 image_path:Path=None, route_image_path:Path=None, 
                 parent=None):
        
        '''start_delay_sec는 0.5 밑으로는 설정하지 마라.
        여러번 클릭시에 경로 찾기가 잠깐 멈춘다 다시 클릭해야 npc가 움직인다.
        '''
        super().__init__()
        self.parent = parent
        self.world = world
        self.id = npc_id
        self.native_terrain = TerrainType.NORMAL
        self.influence_range = influence_range
        self.max_range = max_range

        self.m_start = start
        self.m_goal = self.m_start
        start_coord = c_coord.from_tuple(start)
        self.finder = c_dstar_lite(self.world.navgrid, start_coord)
        
        self.max_retry = max_retry
        self.set_max_retry(max_retry)

        self.route_capacity = route_capacity
        self.set_route_capacity(route_capacity)

        self.loop_once = False

        self.movable_terrain = [TerrainType.NORMAL]

        # 경로 및 이미지 캐시에서 로딩
        self.image_paths = ImageManager.get_npc_image_paths(image_path)
        self.images = ImageManager.get_npc_image_set(image_path)        

        self.route_images = ImageManager.get_route_image_set(route_image_path)

        self.direction = random.randint(
            RouteDir.RIGHT.value, RouteDir.DOWN_RIGHT.value)
        
        self.disp_dx = 0.0
        self.set_disp_dx(0.0)
        self.disp_dy = 0.0
        self.set_disp_dy(0.0)

        self.draw_offset_x = 0
        self.draw_offset_y = 0
        self.anim_dx_arrived = False
        self.anim_dy_arrived = False

        self.goal_list:list[tuple[int,int]] = list()

        self.real_list = list()
        self.proto_list = list()

        self.phantom_start = self.m_start
        self.anim_started = False

        self.next = None
        self.next_history = list()
        
        self.speed_kmh = speed_kmh  # default speed
        self.start_delay_sec = start_delay_sec
        self.set_start_delay_sec(start_delay_sec)

        self.total_elapsed_sec = 0.0

        self.finder.set_move_func(self._move_cb)

        self.finder.set_changed_coords_func(self.world.changed_coords_cb)

        self.finder.set_cost_func(self._cost_cb)

        self.finding_thread = None
        
        self.finding_active = False

        # self._real_q = Queue()
        # self._proto_q = Queue()

    @Slot(float)
    def set_disp_dx(self, dx:float):
        self.disp_dx = dx
        self.disp_dx_changed.emit(dx)

    @Slot(float)
    def set_disp_dy(self, dy:float):
        self.disp_dy = dy
        self.disp_dy_changed.emit(dy)

    @Slot(int)
    def set_route_capacity(self, capacity:int):
        self.route_capacity = capacity
        self.route_capacity_changed.emit(capacity)

    @Slot(int)
    def set_max_retry(self, max_retry:int):
        self.max_retry = max_retry
        self.finder.max_retry = max_retry
        self.max_retry_changed.emit(max_retry)

    @Slot(float)
    def set_speed_kmh(self, speed_kmh:float):
        self.speed_kmh = speed_kmh

    @Slot(float)
    def set_start_delay_sec(self, delay_sec:float):
        self.start_delay_sec = delay_sec
        self.start_delay_sec_changed.emit(delay_sec)

    def reset(self):
        """NPC 상태를 초기화한다. (경로, 애니메이션, 큐 등)"""
        # 탐색기 재초기화
        self.finder.reset()

        self.goal_list.clear()

        self.next = None

        # 애니메이션 상태
        self.anim_started = False
        self.anim_dx_arrived = False
        self.anim_dy_arrived = False
        self.disp_dx = 0.0
        self.disp_dy = 0.0
        self.disp_dx_changed.emit(0.0)
        self.disp_dy_changed.emit(0.0)

        # 위치 추적 및 타이머
        self.phantom_start = self.m_start
        self.total_elapsed_sec = 0.0

        # 스레드 상태
        self.finding_thread = None
        self.finding_active = False

    def close(self):
        '''NPC 종료 시 리소스를 정리한다'''
        # 🔸 탐색 쓰레드 정지
        self.stop_finding()

        self.phantom_start = None

        self.finder.close()

        # 🔸 로깅
        g_logger.log_debug(f"[NPC.close] npc({self.id}) 종료 완료")

    def __del__(self):
        self.close()

    @property
    def start(self):
        # return self.finder.start.to_tuple()
        c = self.finder.get_start()
        return c.to_tuple()
        
    @start.setter
    def start(self, coord:tuple):
        # self.finder.start = c_coord.from_tuple(coord)
        self.m_start = coord
        c = c_coord.from_tuple(coord)
        self.finder.set_start(c)
        # self.world.add_changed_coord(coord)
        self.start_changed_sig.emit(coord)

    @property
    def goal(self):
        # return self.finder.goal.to_tuple()
        c = self.finder.get_goal()
        return c.to_tuple()
    
    @goal.setter
    def goal(self, coord: tuple):
        # self.finder.goal = c_coord.from_tuple(coord)
        self.m_goal = coord
        self.finder.set_goal(c_coord.from_tuple(coord))
        # self.world.add_changed_coord(coord)
        self.goal_changed_sig.emit(coord)

    @property
    def speed_kmh(self):
        return self.m_speed_kmh
    
    @speed_kmh.setter
    def speed_kmh(self, kmh: float):
        self.m_speed_kmh = kmh
        self.finder.set_interval_sec(self.interval_sec)
        self.speed_kmh_changed.emit(kmh)

    @property
    def interval_sec(self):
        """속도(km/h)와 셀 단위(m)를 기준으로 한 이동 간격(sec) 반환"""
        speed_mps = self.m_speed_kmh * 1000 / 3600.0
        if speed_mps == 0:
            return float('inf')  # 속도가 0이면 무한대 시간 필요
        return int((self.world.grid_unit_m / speed_mps))

    @Slot(int, int)
    def set_start_from_int(self, x:int, y:int):
        s = (x, y)
        self.m_start = s

    def append_goal(self, coord:tuple):
        # self._goal_q.put(coord)
        self.goal_list.append(coord)
        
    def move_to(self, coord: tuple):
        if self.finding_thread and self.finding_thread.is_alive():
            g_logger.log_debug("🔁 현재 find 루프 종료 요청 중...")
            # 바로 요청하면 마우스 클릭으로 여러번 함수 호출할때
            # force_quit중에 또 force_quit이 실행되어서 
            # 다음 finder 루프를 돌수 없다.
            self.finder.force_quit()
            # 이제 요청했다. 루프 멈추라고...
            # 즉각적으로 멈추는게 아니다.
            # 아마도 시속에 따라 계산된 interval_sec 이후에 멈춘다.
            self.finding_active = False

        self.goal = coord
        # c = c_coord.from_tuple(coord)
        # self.finder.update_vertex(c)

        # 목표 모드가 한번만 설정하는것이다.
        # append_goal은 기본적으로 클릭할때마다 목표를 추가한다
        # 하지만 모드가 loop_once 이므로 가장 마지막에 설정된 목표만 
        # 실제 이동할때 사용된다.
        self.loop_once = True

        # 바로 목표로 이동하는게 아니다.
        # 시작 지연 msec에 따라 약간 지연 후에 on_tick에서 이동 시작한다.        
        self.append_goal(coord)

    def anim_moving_to(self, next: tuple, elapsed_sec: float, cell_size:int):
        speed_mps = self.speed_kmh * 1000 / 3600.0
        speed_pixel_per_sec = speed_mps * (cell_size / self.world.grid_unit_m)
        delta = speed_pixel_per_sec * elapsed_sec
        epsilon = 1e-3

        target_dx = (next[0] - self.phantom_start[0]) * cell_size
        target_dy = (next[1] - self.phantom_start[1]) * cell_size

        delta_x = target_dx - self.disp_dx
        if abs(delta_x) <= delta + epsilon:
            self.disp_dx = target_dx
            self.anim_dx_arrived = True
            self.disp_dx_changed.emit(target_dx)
        else:
            self.disp_dx += delta if delta_x > 0 else -delta
            self.anim_dx_arrived = False
            self.disp_dx_changed.emit(self.disp_dx)

        delta_y = target_dy - self.disp_dy
        if abs(delta_y) <= delta + epsilon:
            self.disp_dy = target_dy
            self.anim_dy_arrived = True
            self.disp_dy_changed.emit(target_dy)
        else:
            self.disp_dy += delta if delta_y > 0 else -delta
            self.anim_dy_arrived = False
            self.disp_dy_changed.emit(self.disp_dy)

    def is_anim_arrived(self) -> bool:
        return self.anim_dx_arrived and self.anim_dy_arrived
    
    def on_tick(self, elapsed_sec: float, cell_size:int):
        if self.total_elapsed_sec >= self.start_delay_sec:
            # if not self._goal_q.empty():
            if len(self.goal_list) > 0:
                if not self.finding_thread or not self.finding_thread.is_alive():
                    g_logger.log_debug(f'''지금 find()가 실행되었다
가장 중요한 finder.is_quit_forced는 {self.finder.is_quit_forced()}
elapsed_sec : {elapsed_sec}, 
self.total_elapsed_sec : {self.total_elapsed_sec},
start_delay_sec : {self.start_delay_sec}''')
                    self.find()
            self.total_elapsed_sec = 0.0
        else:
            self.total_elapsed_sec += elapsed_sec

        # 현재 목표가 없으면 큐에서 꺼내서 대기 목표 설정
        if not self.next and not self.anim_started:
            if len(self.next_history) > 0:
                next = self.next_history.pop(0)
                self.next = next
                self.anim_started = True
                self.anim_to_started_sig.emit(self.next)
            else:
                next = None

        if self.next:
            ps = c_coord.from_tuple(self.phantom_start)
            sn = c_coord.from_tuple(self.next)
            new_dir = c_route.calc_direction(ps, sn)
            if new_dir != RouteDir.UNKNOWN:
                self.direction = new_dir

            self.anim_moving_to(self.next, elapsed_sec, cell_size)

            if self.is_anim_arrived():
                self.anim_to_arrived_sig.emit(self.next)
                self.m_start = self.next
                self.world.add_changed_coord(self.next)
                self.phantom_start = self.m_start

                self.disp_dx = 0
                self.disp_dy = 0
                self.disp_dx_changed.emit(0)
                self.disp_dy_changed.emit(0)
                self.anim_dx_arrived = False
                self.anim_dy_arrived = False
                self.anim_started = False
                self.next = None  # 다음 tick에서 새 목표 pop

    def find_loop(self):
        '''쓰레드에서 실행된다.'''
        try:
            prev_goal = None
            while self.finding_active:
                try:
                    if self.loop_once:
                        # 가장 마지막에 추가된 목표만 사용한다.
                        g = self.goal_list.pop()
                        
                        self.goal = g
                        # self.world.add_changed_coord(g)

                        self.loop_once = False

                        self.goal_list.clear()

                    else:
                        if prev_goal is None:
                            prev_goal = self.m_start

                        if prev_goal == self.m_start:
                            if len(self.goal_list) <= 0:
                                break

                            g = self.goal_list.pop(0)
                            self.goal = g
                            # self.world.add_changed_coord(g)
                            self.m_start = prev_goal
                            self.world.add_changed_coord(prev_goal)
                        else:
                            if prev_goal != self.goal:
                                prev_goal = self.goal

                    self.finder.find_proto()
                    route = self.finder.get_proto_route()
                    l = route.coords().to_list()
                    for c in l:
                        self.proto_list.append(c)

                    # if route.success:
                    if route.is_success():
                        g_logger.log_debug_threadsafe(f'proto route 찾기가 성공했다')
                    else:
                        g_logger.log_debug_threadsafe(f'proto route 찾기가 실패했다')
                        '''실패시 행동요령
                        1. 실패하면 목표를 찾아야 한다.
                        2. 장애물을 제거해야 한다.
                        3. 가만 있는다.
                        '''

                    g_logger.log_debug_threadsafe(f'''초기 경로 찾기 로그:
        self.finder.proto_compute_retry_count : {self.finder.proto_compute_retry_count}, 
        self.finder.reconstruct_retry_count : {self.finder.reconstruct_retry_count}, 
        ''')

                    self.finder.find_loop()
                    route1 = self.finder.get_real_route()
                    self.real_list.append(route1.coords().to_list())
                    # if route1.success:
                    if route1.is_success():
                        g_logger.log_debug_threadsafe(f'real route 찾기가 성공했다')
                    else:
                        g_logger.log_debug_threadsafe(f'real route 찾기가 실패했다')

                    g_logger.log_debug_threadsafe(f'''실제 경로 찾기 로그
        self.finder.real_compute_retry_count : {self.finder.real_compute_retry_count}, 
        self.finder.real_loop_retry_count : {self.finder.real_loop_retry_count}
        ''')
                    g_logger.log_debug_threadsafe(
                                '✅ find_loop가 정상 종료되었습니다')
                except Empty:
                    g_logger.log_debug_threadsafe(
                        '✅ find_loop가 self._goal_q.get_nowait()가 '
                        'Empty라서 종료되었습니다')
                    break

        except Exception as e:
            g_logger.log_debug_threadsafe(f'🚨 경로 탐색 중 예외 발생: {e}')

        finally:
            # 🔸 종료 처리 (외부에서 self.finding_active = False 호출할 수 
            # 있으므로 여기선 굳이 다시 False 안 해도 됨)
            self.finding_thread = None

    def find(self):
        if self.finding_thread and self.finding_thread.is_alive():
            return  # 이미 실행 중이면 무시
        
        self.finding_active = True
        self.finding_thread = Thread(
            target=self.find_loop, daemon=True)
        
        self.finding_thread.start()

    def stop_finding(self):
        self.finding_active = False

        if self.finding_thread:
            if self.finding_thread.is_alive():
                # finder() 내부에서 while루프에 forece_quite = True해서 루프 종료
                # finder()가 종료되면 자연스럽게 쓰레드 종료
                self.finder.force_quit()

                self.finding_thread.join(timeout=1.0)
                if self.finding_thread and self.finding_thread.is_alive():
                    g_logger.log_debug_threadsafe(
                        "⏰ 타임아웃: 쓰레드 아직 종료되지 않음!")
                    # 필요하면 여기서 강제종료 로직 또는 경고 처리
                    return  # 또는 그냥 남겨둠
            self.finding_thread = None

    def _move_cb(self, coord_c:'c_coord', userdata):
        try:
            g_logger.log_debug_threadsafe(f"[MOVE_CB] 받은 이동 좌표: {coord_c}")
            c = coord_c.to_tuple()
            self.next_history.append(c)

        except Exception as e:
            g_logger.log_debug_threadsafe(f"[MOVE_CB] 예외 발생: {e}")

    def _cost_cb(self, map_obj, start, goal, userdata):
        """
        기본 비용 함수 예시. set_cost_func()에 전달 가능한 형태.
        
        - 장애물 셀은 inf 반환
        - 아닌 경우 유클리드 거리 반환
        """
        tg = goal.to_tuple()
        cell = self.world.block_mgr.get_cell(tg)

        if cell is None or self.is_obstacle(cell):
            return float('inf')

        dx = start.x - goal.x
        dy = start.y - goal.y
        return math.hypot(dx, dy)

    @staticmethod
    def _is_blocked_cb(navgrid:c_navgrid, x, y, userdata):
        c = (x, y)
        cell = userdata.world.block_mgr.get_cell(c)
        return userdata.is_obstacle(cell)

    def draw(self, painter: QPainter,
                 start_win_pos_x:int, start_win_pos_y:int, cell_size):
        '''실제 디바이스에 이미지를 그린다.
        '''
        x = start_win_pos_x + self.draw_offset_x + int(self.disp_dx)
        y = start_win_pos_y + self.draw_offset_y + int(self.disp_dy)

        # 배경: 반투명 검정
        # rect = QRect(x, y, cell_size, cell_size)
        # painter.setBrush(QColor(0, 0, 0, 127))
        # painter.setPen(Qt.NoPen)
        # painter.drawRect(rect)

        image = self.get_image()

        painter.drawPixmap(
                x, y, cell_size, cell_size, image)
        pass

    def get_image(self):
        return self.images[self.direction]
    
    def get_image_path(self):
        return self.image_paths[self.direction]
    
    def get_proto_route_image(self, coord):
        c = c_coord.from_tuple(coord)
        direction = c_route.get_direction_by_dir_coord(c)
        return self.route_images[direction]
    
        # for sublist in self.proto_list:  # sublist: list of c_coord
        #     try:
        #         cur_idx = sublist.index(c)
        #     except ValueError:
        #         continue  # 현재 coord_list에는 없음, 다음 coord_list로

        #     if cur_idx + 1 < len(sublist):
        #         start = c
        #         end = sublist[cur_idx + 1]
        #     elif cur_idx - 1 >= 0:
        #         start = sublist[cur_idx - 1]
        #         end = c
        #     else:
        #         return None  # 단일 좌표인 경우

        #     # direction = c_route.calc_direction(start, end)
        #     direction = c_route.get_direction_by_dir_coord(c)
        #     return self.route_images[direction]

        # return None  # 모든 coord_list에 coord가 없을 경우
        
    def get_real_route_image(self, coord):
        c = c_coord.from_tuple(coord)

        for sublist in self.real_list:  # sublist: list of c_coord
            try:
                cur_idx = sublist.index(c)
            except ValueError:
                continue  # 현재 coord_list에는 없음, 다음 coord_list로

            if cur_idx + 1 < len(sublist):
                start = c
                end = sublist[cur_idx + 1]
            elif cur_idx - 1 >= 0:
                start = sublist[cur_idx - 1]
                end = c
            else:
                return None  # 단일 좌표인 경우

            direction = c_route.calc_direction(start, end)
            return self.route_images[direction]

        return None  # 모든 coord_list에 coord가 없을 경우    

    def load_image_paths(self, image_path:Path):
        self.image_paths = ImageManager.get_npc_image_paths(image_path)

    def load_images(self, image_path:Path):
        self.images = ImageManager.get_npc_image_set(image_path)                

    def on_proto_route_found(self):
        new_items: list = self.proto_list

        if not new_items:
            g_logger.log_debug('proto_list is empty')
            return

        # 중복 없이 추가
        for item in new_items:
            if item not in self.proto_list:
                self.proto_list.append(item)

        # 길이 초과 시 마지막 N개만 유지
        if len(self.proto_list) > self.route_capacity:
            self.proto_list[:] = self.proto_list[-self.route_capacity:]

        g_logger.log_debug(f'len(proto_list): {len(self.proto_list)}')

    def on_real_route_found(self):
        new_items: list = self.real_list

        if not new_items:
            g_logger.log_debug('real_list is empty')
            return

        # 중복 없이 추가
        for item in new_items:
            if item not in self.real_list:
                self.real_list.append(item)

        # 길이 초과 시 마지막 N개만 유지
        if len(self.real_list) > self.route_capacity:
            self.real_list[:] = self.real_list[-self.route_capacity:]

        g_logger.log_debug(f'len(real_list): {len(self.real_list)}')


    def clear_proto_route(self):
        self.proto_list.clear()

    def clear_real_route(self):
        self.real_list.clear()

    def is_movable(self, cell:GridCell):
        return not self.is_obstacle(cell)
    
    def is_obstacle(self, cell:GridCell):
        if cell.status == CellStatus.NPC:
            return True
        
        if cell.terrain == TerrainType.FORBIDDEN:
            return True

        if not cell.terrain in self.movable_terrain:            
            return True
        
        return False

    def get_image(self):
        return self.images[self.direction]
    
    def get_selected_npc_image(self, path:Path=None):
        return ImageManager.get_selected_npc_image(path)
    
    @staticmethod
    def generate_random_npc_id() -> str:
        return str(uuid.uuid4())[:8]

