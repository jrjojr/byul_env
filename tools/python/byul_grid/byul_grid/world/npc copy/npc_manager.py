from PySide6.QtCore import QObject, Signal

from PySide6.QtCore import QObject
from world.npc.npc import NPC

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from world.world import World  # 순환 참조 방지용 타입 힌트

from utils.log_to_panel import g_logger

class NPCManager(QObject):
    npc_created = Signal(str)

    def __init__(self, world: "World"):
        super().__init__()
        self.npc_dict: dict[str, NPC] = {}
        self.world = world

    def has_npc(self, npc_id: str) -> bool:
        return npc_id in self.npc_dict

    def get_npc(self, npc_id: str) -> NPC | None:
        return self.npc_dict.get(npc_id)

    def create_npc(self, npc_id: str, coord: tuple[int, int]):
        """
        새로운 NPC를 생성하여 등록하고 좌표에 배치한다.
        동일 ID의 NPC가 이미 존재하면 경고 후 무시.
        """
        if self.has_npc(npc_id):
            return None

        npc = NPC(npc_id, self.world, coord)
        self.npc_dict[npc_id] = npc

        self.npc_created.emit(npc_id)
    
    def delete_npc(self, npc_id: str):
        """NPC를 삭제하고 모든 인덱스를 제거한다."""
        npc = self.npc_dict.pop(npc_id, None)
        if not npc:
            return

        npc.start = None
        npc.close()  # 또는 필요 시 npc.reset()
        g_logger.log_debug(f"[NPCManager] npc({npc_id}) 제거 완료")

    def reset(self):
        """모든 NPC의 상태를 초기화하고 좌표 인덱스를 정리한다."""
        for npc_id, npc in self.npc_dict.items():
            try:
                npc.reset()
                npc.start = None
            except Exception as e:
                g_logger.log_debug(f"[NPCManager] npc({npc_id}) 초기화 실패: {e}")

    def attach_npc(self, npc: NPC):
        """
        외부에서 생성된 NPC를 이 매니저에 등록하고 위치 정보를 설정한다.
        이미 같은 ID가 존재하면 덮어쓴다.
        """
        npc_id = npc.id

        self.npc_dict[npc_id] = npc

        g_logger.log_debug(f"[NPCManager] npc({npc_id}) 등록 완료 at {npc.start}")

    def detach_npc(self, npc: NPC):
        """
        NPC를 매니저에서 제거하지만, 실제 객체는 소멸시키지 않는다.
        위치 정보도 정리한다.
        """
        npc_id = npc.id

        self.npc_dict.pop(npc_id, None)

        npc.start = None  # 위치 초기화
        g_logger.log_debug(f"[NPCManager] npc({npc_id}) 연결 해제 완료")
