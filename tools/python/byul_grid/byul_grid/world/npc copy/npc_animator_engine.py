# animator_engine.py

from concurrent.futures import ThreadPoolExecutor
from queue import Queue
import threading

class AnimatorTask:
    def __init__(self, animator, elapsed_sec):
        self.animator = animator
        self.elapsed_sec = elapsed_sec

class AnimatorEngine:
    def __init__(self, max_workers=8):
        self.executor = ThreadPoolExecutor(max_workers=max_workers)
        self.queue = Queue()
        self.running = True
        self.thread = threading.Thread(
            target=self._dispatcher_loop, daemon=True)
        self.thread.start()

    def submit(self, animator, elapsed_sec):
        """애니메이터를 큐에 제출한다. (NPC가 아니라 Animator 기준)"""
        if animator and animator.is_anim_started():
            self.queue.put(AnimatorTask(animator, elapsed_sec))

    def _dispatcher_loop(self):
        while self.running:
            task = self.queue.get()
            if task is None:
                break
            self.executor.submit(self._process_task, task)

    def _process_task(self, task: AnimatorTask):
        task.animator.tick(task.elapsed_sec)
        # tick() 내부에서 on_anim_complete() 호출됨
        # 필요시 이후 프레임처리는 NPC가 직접 구현 가능

    def shutdown(self):
        self.running = False
        self.queue.put(None)
        self.executor.shutdown(wait=True)
