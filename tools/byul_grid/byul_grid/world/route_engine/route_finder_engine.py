from concurrent.futures import ThreadPoolExecutor
from queue import Queue
from typing import Callable
import threading

from route import c_route
from navgrid import c_navgrid
from route_finder import c_route_finder, RouteFinderType
from route_finder_common import g_RouteFuncReg
from coord import c_coord

from utils.log_to_panel import g_logger

from .common import RouteRequest, RouteResult

class RouteFinderEngine:
    def __init__(self, max_workers: int = 4):
        self.executor = ThreadPoolExecutor(max_workers=max_workers)
        self.task_queue = Queue()
        self.running = True
        threading.Thread(target=self._dispatcher_loop, daemon=True).start()

    def _dispatcher_loop(self):
        while self.running:
            request: RouteRequest = self.task_queue.get()
            if request is None:
                break
            self.executor.submit(self._process_request, request)

    def _process_request(self, request: RouteRequest):
        g_logger.log_debug_threadsafe('before 길찾기')

        # userdata는 C 쪽에서 직접 쓰지 않고 복제해서 넘겨라
        safe_userdata = request.userdata if isinstance(
            request.userdata, (int, float, str)) else None
        
        cost_func = g_RouteFuncReg.get_cost_func(request.cost_func_name)
        heuristic_func = g_RouteFuncReg.get_heuristic_func(
            request.heuristic_func_name)
        
        self.route_finder = c_route_finder(
            navgrid=request.navgrid,
            type=request.type,
            start=c_coord.from_tuple(request.start),
            goal=c_coord.from_tuple(request.goal),
            cost_fn=cost_func,
            heuristic_fn=heuristic_func,
            max_retry=request.max_retry,
            debug_mode=request.debug_mode,
            userdata=safe_userdata
        )

        route: 'c_route' = self.route_finder.find()
        g_logger.log_debug_threadsafe('after 길찾기')
        result = RouteResult(request.npc_id, route)
        request.on_route_found_cb(result)

    def submit(self,
               navgrid: c_navgrid,
               npc_id: str,
               type: RouteFinderType,
               start: tuple,
               goal: tuple,
               on_route_found_cb: Callable,
               max_retry: int = 10000,
               debug_mode: bool = False,
               cost_func_name: str = "default",
               heuristic_func_name: str = "euclidean",
               userdata: any = None):
        request = RouteRequest(
            navgrid=navgrid,
            npc_id=npc_id,
            type=type,
            start=start,
            goal=goal,
            on_route_found_cb=on_route_found_cb,
            max_retry=max_retry,
            debug_mode=debug_mode,
            cost_func_name=cost_func_name,
            heuristic_func_name=heuristic_func_name,
            userdata=userdata
        )
        self.task_queue.put(request)

    def shutdown(self):
        self.running = False
        self.task_queue.put(None)
        self.executor.shutdown(wait=True)

