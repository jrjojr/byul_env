from PySide6.QtCore import Qt, QRect, QObject, Slot, Signal, QTimer

from collections import deque, OrderedDict

from grid.grid_cell import GridCell, CellFlag, TerrainType
from world.village.village import Village
from grid.grid_block_manager import GridBlockManager

from byul_wrapper.coord import c_coord
from byul_wrapper.navgrid import c_navgrid
from world.route_engine.route_finder_engine import RouteFinderEngine
# from world.npc.npc_animator_engine import AnimatorEngine

from world.npc.npc import NPC
from world.npc.npc_manager import NPCManager

from utils.log_to_panel import g_logger
import time

from queue import Queue, Empty
from byul_wrapper.coord_list import c_coord_list

FIRST_NPC_ID = 'first_npc'
FIRST_VILLAGE_ID = 'first_village'

class World(QObject):
    npc_created = Signal(str)
    npc_deleted = Signal(str)

    grid_unit_m_changed = Signal(float)

    def __init__(self, block_size=100, grid_unit_m=1.0, parent=None):
        super().__init__()
        self.parent = parent
        self.selected_village = None

        self.navgrid = c_navgrid()

        self.finder_engine = RouteFinderEngine()

        self.grid_unit_m = grid_unit_m
        self.set_grid_unit_m(grid_unit_m)

        self.block_mgr = GridBlockManager(block_size)
        self.npc_mgr = NPCManager(self)
        self.block_mgr.on_after_block_loaded = self.on_after_block_loaded
        self.block_mgr.on_before_block_evicted = self.on_before_block_evicted
        
        self.villages: dict[str, Village] = {}

        # 기본 마을 생성
        village = self.create_village(FIRST_VILLAGE_ID, 0, 0, 4000, 4000)

        # 기본 NPC 생성

        npc = self.spawn_npc(FIRST_NPC_ID, (0, 0))

        self.selected_village = village
        
        self.pending_spawn_batches: deque[list[tuple[str, tuple]]] = deque()
        self._block_load_queue: deque[tuple] = deque()
        self._loading_scheduled = False
        self._spawning_scheduled = False

        self.queued_despawn_ids: OrderedDict[str, None] = OrderedDict()
        self._block_evict_queue: deque[tuple] = deque()
        self._evicting_scheduled = False
        self._despawning_scheduled = False

        self._changed_q = Queue()        

    @Slot(float)
    def set_grid_unit_m(self, grid_unit_m:float):
        self.grid_unit_m = grid_unit_m
        self.grid_unit_m_changed.emit(grid_unit_m)

    def reset(self):
        self.navgrid.clear()
        self.block_mgr.reset()
        self.npc_mgr.reset()

    def close(self):
        self.finder_engine.shutdown()
        # self.animator_engine.shutdown()
        self.navgrid.close()

    def create_village(self, name: str, 
                    center_x:int, center_y:int, width: int, height:int):
        v = Village(self.block_mgr, center_x, center_y, width, height)
        self.villages[name] = v
        return v

    def add_obstacle(self, coord: tuple, npc:NPC):
        cell = self.block_mgr.get_cell(coord)
        if cell and npc:
            if not npc.movable_terrain:
                cell.terrain = TerrainType.FORBIDDEN
                self.add_changed_coord((cell.x, cell.y))
                g_logger.log_always(
                    f"[SET OBSTACLE] {coord} : {npc.id}가 "
                    f"{cell.terrain.name}으로 설정했다."
                )
                return

            # 2. 현재 terrain이 NPC 기준 이동 가능하다면 → 바꿔야 함
            if cell.terrain in npc.movable_terrain:
                for terrain in TerrainType:
                    if terrain not in npc.movable_terrain:
                        cell.terrain = terrain
                        self.add_changed_coord((cell.x, cell.y))
                        g_logger.log_always(
                            f"[SET OBSTACLE] {coord} → terrain = "
                            f"{terrain.name} (not movable by {npc.id})"
                        )
                        return
            else:
                g_logger.log_always(
                    f"[SKIP SET OBSTACLE] {coord} → {npc.id} 기준으로 "
                    f"이미 이동 불가 terrain ({cell.terrain.name})"
                )

    def remove_obstacle(self, coord: tuple, npc:NPC):
        cell = self.block_mgr.get_cell(coord)
        if cell and npc:
            old_terrain = cell.terrain
            new_terrain = npc.native_terrain
            cell.terrain = new_terrain
            self.add_changed_coord((cell.x, cell.y))            
            g_logger.log_always(
                f"[REMOVE OBSTACLE] {coord} → {npc.id} 기준 장애물 제거 "
                f"({old_terrain.name} → {new_terrain.name})"
            )

    def toggle_obstacle(self, coord: tuple, npc:NPC):
        cell = self.block_mgr.get_cell(coord)
        if not cell or not npc:
            return

        if cell.terrain not in npc.movable_terrain:
            # 현재 terrain은 NPC 기준 장애물이다 → 해제
            self.remove_obstacle(coord, npc)
        else:
            # 현재 terrain은 NPC가 통과 가능 → 새 장애물 생성
            self.add_obstacle(coord, npc)

    def set_start(self, npc: NPC, coord: tuple):
        new_cell = self.block_mgr.get_cell(coord)
        if new_cell and npc.is_movable(new_cell):
            new_cell.add_flag(CellFlag.START)
            npc.start = coord
            npc.goal = coord
            self.place_npc_to_cell(npc)
        else:
            g_logger.log_always(f'{coord}는 npc가 이동할 수 없는 테란타입이다.')

    def set_goal(self, npc: NPC, coord: tuple):
        new_cell = self.block_mgr.get_cell(coord)
        if new_cell and npc.is_movable(new_cell):
            old_cell = self.block_mgr.get_cell(npc.goal)
            old_cell.remove_flag(CellFlag.GOAL)
            for c in npc.goal_list:
                old_cell = self.block_mgr.get_cell(c)
                if old_cell:
                    old_cell.remove_flag(CellFlag.GOAL)

            new_cell.add_flag(CellFlag.GOAL)
            npc.move_to(coord)
        else:
            g_logger.log_always(f'{coord}는 장애물 좌표이다.')

    def append_goal(self, npc: NPC, coord: tuple):
        new_cell = self.block_mgr.get_cell(coord)
        if new_cell:
            new_cell.add_flag(CellFlag.GOAL)
        npc.append_goal(coord)
        self.find_proto(npc)

    def find_proto(self, npc: NPC):
        if g_logger.debug_mode:
            t0 = time.time()
        npc.find()
        if g_logger.debug_mode:
            t1 = time.time()
            elapsed = t1 - t0
            g_logger.log_debug(f'elapsed : {elapsed:.3f} msec')

    @Slot(NPC)
    def apply_proto_to_cells(self, npc: NPC):
        for coord in npc.proto_list:
            ct = coord.to_tuple()
            if (cell := self.block_mgr.get_cell(ct)):
                cell.add_flag(CellFlag.ROUTE)

    def clear_proto_flags(self, npc: NPC):
        """
        NPC의 proto_list에 따라 설정된 셀들의 ROUTE 플래그를 제거한다.
        """
        for coord in npc.proto_list:
            ct = coord.to_tuple()
            cell = self.block_mgr.get_cell(ct)
            if cell:
                cell.remove_flag(CellFlag.ROUTE)

        g_logger.log_debug(f"[clear_proto_flags] npc({npc.id})의 경로 깃발 제거 완료")


    def place_npc_to_cell(self, npc: NPC, coord:tuple):
        # npc가 기존에 있던 셀에서 제거한다.
        key = self.block_mgr.get_origin(npc.start)
        cell = self.block_mgr.get_cell(npc.start)
        cell.remove_npc_id(npc.id)
        cell.remove_flag(CellFlag.START)

        self.block_mgr.set_cell(key, cell)

        # 새로운 위체의 셀에 npc를 추가한다.
        npc.start = coord
        cell = self.block_mgr.get_cell(coord)
        if cell:
            if cell.terrain not in npc.movable_terrain:
                if cell.terrain != TerrainType.FORBIDDEN:
                    npc.movable_terrain.append(cell.terrain)
                    npc.native_terrain = cell.terrain

            cell.add_npc_id(npc.id)
            cell.add_flag(CellFlag.START)            

            # 경로 제거
            if cell.has_flag(CellFlag.ROUTE):
                cell.remove_flag(CellFlag.ROUTE)

            # 목표 제거
            if cell.has_flag(CellFlag.GOAL):
                cell.remove_flag(CellFlag.GOAL)

            key = self.block_mgr.get_origin(coord)
            self.block_mgr.set_cell(key, cell)

    @Slot(tuple)
    def on_anim_to_arrived(self, npc: NPC, coord: tuple):
        self.place_npc_to_cell(npc, coord)

        pass

    def get_npcs_in_rect(self, rect: QRect) -> set[NPC]:
        """
        셀 구조를 기반으로 rect 범위 내의 NPC 객체들을 반환한다.
        None은 포함하지 않는다.
        """
        result = set()

        x0, x1 = rect.left(), rect.right()
        y0, y1 = rect.top(), rect.bottom()

        for x in range(x0, x1):
            for y in range(y0, y1):
                cell = self.block_mgr.get_cell((x, y))
                if not cell or not cell.npc_ids:
                    continue

                for npc_id in cell.npc_ids:
                    npc = self.npc_mgr.get_npc(npc_id)
                    if npc:
                        result.add(npc)

        return result

    def spawn_npc(self, npc_id, start:tuple):
        if not self.npc_mgr.has_npc(npc_id):
            self.npc_mgr.create_npc(npc_id, start)
        
        npc = self.npc_mgr.get_npc(npc_id)

        npc.anim_to_arrived_sig.connect(lambda coord, n=npc: 
            self.place_npc_to_cell(n, coord))

        cell = self.block_mgr.get_cell(start)
        if cell:
            self.place_npc_to_cell(npc, start)        
        else:
            g_logger.log_debug(f'''왜 셀이 없지? 
아직 생성되지 않았거나 제거되었겠지, 
셀에 위치할때까지 기다려야 한다.''')

            # 블록 요청
            self.block_mgr.request_load_block(start[0], start[1])

            # 로딩 완료되면 다시 셀 배치 시도
            def on_block_loaded(key):
                if key == self.block_mgr.get_origin(start):
                    self.place_npc_to_cell(npc, start)
                    self.block_mgr.load_block_succeeded.disconnect(on_block_loaded)

            self.block_mgr.load_block_succeeded.connect(on_block_loaded)

        # 5. 생성 완료 알림
        self.npc_created.emit(npc_id)
        return npc
    
    def delete_npc(self, npc_id):
        # npc 존재 여부 확인
        if not self.npc_mgr.has_npc(npc_id):
            g_logger.log_debug(f'npc({npc_id})가 존재하지 않아서 종료한다.')
            return

        # npc 객체 추출
        npc: NPC = self.npc_mgr.npc_dict.pop(npc_id)

        # 현재 위치 기준 셀에서 npc 제거
        key = self.block_mgr.get_origin(npc.start)
        cell = self.block_mgr.get_cell(npc.start)
        if cell:
            cell.remove_npc_id(npc_id)
            self.block_mgr.set_cell(key, cell)
        # else:
        #       # 이미 블락이 제거되었으면 셀도 존재하지 않는다.
        #     g_logger.log_debug(
        #         f'npc({npc_id})의 위치 셀을 찾을 수 없음: {npc.start}')

        # 관련 리소스 해제 (비동기 스레드 종료 등 추가 처리 필요 시 여기에)
        npc.close()

        # 시그널 전파
        self.npc_deleted.emit(npc_id)

    def on_after_block_loaded(self, block_key: tuple):
        if block_key not in self._block_load_queue:
            self._block_load_queue.append(block_key)
        if not self._loading_scheduled:
            self._loading_scheduled = True
            self._process_block_load()

    def _process_block_load(self, batch_size=100):
        self._loading_scheduled = False

        while self._block_load_queue:
            block_key = self._block_load_queue.popleft()
            block = self.block_mgr.block_cache.get(block_key)
            if not block:
                continue

            pending_npcs: list[tuple[str, tuple]] = []
            for cell in block.cells.values():
                for npc_id in cell.npc_ids:
                    if npc_id in self.queued_despawn_ids:
                        del self.queued_despawn_ids[npc_id]
                        g_logger.log_debug(f"[Spawn:{block_key}] 디스폰 예약 취소됨: {npc_id}")
                    pending_npcs.append((npc_id, (cell.x, cell.y)))
                    if len(pending_npcs) >= batch_size:
                        self.pending_spawn_batches.append(pending_npcs)
                        pending_npcs = []

            if pending_npcs:
                self.pending_spawn_batches.append(pending_npcs)

        self._schedule_spawn()

    def _schedule_spawn(self, interval_msec: int = 1):
        if not self._spawning_scheduled and self.pending_spawn_batches:
            self._spawning_scheduled = True
            QTimer.singleShot(interval_msec, self._spawn_next_batch)

    def _spawn_next_batch(self, batch_size: int = 10, interval_msec: int = 1):
        self._spawning_scheduled = False

        if not self.pending_spawn_batches:
            return

        pending_npcs = self.pending_spawn_batches.popleft()

        def run_batch():
            nonlocal pending_npcs
            count = 0
            while pending_npcs and count < batch_size:
                npc_id, coord = pending_npcs.pop(0)
                self.spawn_npc(npc_id, coord)
                count += 1

            if pending_npcs:
                QTimer.singleShot(interval_msec, run_batch)
            else:
                self._schedule_spawn(interval_msec)

        run_batch()


    def on_before_block_evicted(self, block_key: tuple, interval_msec=50):
        if block_key not in self._block_evict_queue:
            self._block_evict_queue.append(block_key)
        if not self._evicting_scheduled:
            self._evicting_scheduled = True
            QTimer.singleShot(interval_msec, self._process_block_evict)

    def _process_block_evict(self):
        self._evicting_scheduled = False

        while self._block_evict_queue:
            block_key = self._block_evict_queue.popleft()
            for npc_id, npc in self.npc_mgr.npc_dict.items():
                npc_block_key = self.block_mgr.get_origin(npc.start)
                if npc_block_key == block_key:
                    self.queued_despawn_ids[npc_id] = None

        self._schedule_despawn()

    def _schedule_despawn(self, interval_msec: int = 1):
        if not self._despawning_scheduled and self.queued_despawn_ids:
            self._despawning_scheduled = True
            QTimer.singleShot(interval_msec, self._despawn_next_batch)

    def _despawn_next_batch(self, batch_size: int = 1, interval_msec: int = 50):
        self._despawning_scheduled = False

        if not self.queued_despawn_ids:
            return

        npc_ids = list(self.queued_despawn_ids.keys())[:batch_size]

        def run_batch():
            nonlocal npc_ids
            for npc_id in npc_ids:
                self.delete_npc(npc_id)
                self.queued_despawn_ids.pop(npc_id, None)
                g_logger.log_debug(f"[Despawn] NPC 제거됨: {npc_id}")

            self._schedule_despawn(interval_msec)

        run_batch()

    def add_changed_coord(self, coord_c: tuple):
        self._changed_q.put(coord_c)

    def clear_changed_coords(self):
        try:
            while not self._changed_q.empty():
                dummy = self._changed_q.get_nowait()
        except Empty:
            g_logger.log_debug('모두 비웠다 changed_coords를...')
            pass

    def changed_coords_cb(self, userdata):
        g_logger.log_debug_threadsafe('_changed_coords_cb 호출됨')

        c_list_obj = c_coord_list()

        while not self._changed_q.empty():
            tu = self._changed_q.get_nowait()
            c = c_coord.from_tuple(tu)
            c._own = False
            c_list_obj.append(c)
        return c_list_obj