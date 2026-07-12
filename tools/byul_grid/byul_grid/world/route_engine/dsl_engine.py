from concurrent.futures import ThreadPoolExecutor
from queue import Queue
import threading
from typing import Callable
from coord import c_coord
from navgrid import c_navgrid
from dstar_lite import c_dstar_lite
from utils.log_to_panel import g_logger

from route_finder_common import g_RouteFuncReg

from .common import RouteRequest, RouteResult

class DslEngine:
    def __init__(self, max_workers: int = 8):
        self.executor = ThreadPoolExecutor(max_workers=max_workers)
        self.task_queue = Queue()
        self.running = True
        self.dispatcher = threading.Thread(
            target=self._dispatcher_loop, daemon=True)
        self.dispatcher.start()

    def submit(self,
               navgrid: c_navgrid,
               npc_id: str,
               start: tuple,
               goal: tuple,
               on_route_found_cb: Callable,
               move_cb: Callable,
               interval_sec: int = 0.1,
               max_retry: int = 10000,
               cost_func_name: str = "dstar_lite",
               heuristic_func_name: str = "dstar_lite",
               userdata: any = None,
               on_real_route_found_cb: Callable = None):
        map_ptr = navgrid.ptr()
        request = RouteRequest(
            map_ptr=map_ptr,
            npc_id=npc_id,
            start=start,
            goal=goal,
            on_route_found_cb=on_route_found_cb,
            move_cb=move_cb,
            interval_sec=interval_sec,
            max_retry=max_retry,
            cost_func_name=cost_func_name,
            heuristic_func_name=heuristic_func_name,
            userdata=userdata,
            on_real_route_found_cb=on_real_route_found_cb
        )
        self.task_queue.put(request)

    def shutdown(self):
        self.running = False
        self.task_queue.put(None)
        self.executor.shutdown(wait=True)

    def _dispatcher_loop(self):
        while self.running:
            request = self.task_queue.get()
            if request is None:
                break
            self.executor.submit(self._process_request, request)

    def _process_request(self, request: RouteRequest):
        try:
            g_logger.log_debug_threadsafe(f"[{request.npc_id}] 경로 요청 시작")

            map_obj = c_navgrid(raw_ptr=request.map_ptr, own=False)

            finder = c_dstar_lite(map_obj, c_coord.from_tuple(request.start))
            finder.set_goal(c_coord.from_tuple(request.goal))
            finder.set_route_capacity(100)
            finder.set_max_retry(request.max_retry)
            cost_fn = g_RouteFuncReg.get_cost_func(request.cost_func_name)
            heuristic_fn = g_RouteFuncReg.get_heuristic_func(
                request.heuristic_func_name)
            
            finder.set_cost_func(cost_fn)
            finder.set_heuristic_func(heuristic_fn)

            # if isinstance(request.userdata, (int, float, str)):
            #     finder.set_userdata(request.userdata)

            # 핵심 콜백 + 타이밍
            # finder.set_move_func(lambda coord_c: 
            #                          self._handle_move_cb(request, coord_c))
            finder.set_move_func(request.move_cb)
            
            finder.set_interval_sec(request.interval_sec)

            # D* Lite 내부적으로 move_cb 호출됨
            finder.find_loop()

            real_route = finder.get_real_route()

            if request.on_real_route_found_cb:
                request.on_real_route_found_cb(request.npc_id, real_route)

            if request.on_route_found_cb:
                result = RouteResult(request.npc_id, real_route)
                request.on_route_found_cb(result)

            g_logger.log_debug_threadsafe(f"[{request.npc_id}] 경로 처리 완료")

        except Exception as e:
            g_logger.log_debug_threadsafe(f"[{request.npc_id}] 오류 발생: {e}")

