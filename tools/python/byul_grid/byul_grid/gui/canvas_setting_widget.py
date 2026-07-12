from PySide6.QtWidgets import ( 
    QWidget, QFormLayout, QSpinBox, QLabel, QComboBox, QVBoxLayout,
    QFrame, QLineEdit
)
from PySide6.QtGui import QIntValidator
from PySide6.QtCore import Signal, Slot

from gui.grid_canvas import GridCanvas

from world.world import World
from world.npc.npc import NPC

from utils.memory_usage import get_memory_usage_mb

class CanvasSettingWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.canvas = None  # 외부에서 set_canvas(canvas) 호출 필요

        layout = QVBoxLayout(self)

        # --- 기본 속성 폼 ---
        form = QFormLayout()

        # 현재 그리드 상태 (읽기 전용)
        self.label_block_size = QLabel("-")
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setFrameShadow(QFrame.Sunken)

        self.label_grid_width = QLabel("-")
        self.label_grid_height = QLabel("-")

        form.addRow("Block Size", self.label_block_size)
        form.addRow(separator)

        form.addRow("Grid Width", self.label_grid_width)
        form.addRow("Grid Height", self.label_grid_height)
        form.addRow(separator)
        
        # 중심 좌표
        INT_MAX = 2_147_483_647
        INT_MIN = -2_147_483_648

        self.field_center_x = QLineEdit()
        self.field_center_x.setValidator(QIntValidator(INT_MIN, INT_MAX))
        # self.field_center_x.setText("0")  # 초기값은 필요시 동기화

        self.field_center_y = QLineEdit()
        self.field_center_y.setValidator(QIntValidator(INT_MIN, INT_MAX))
        # self.field_center_y.setText("0")  # 초기값은 필요시 동기화

        form.addRow("Center X", self.field_center_x)
        form.addRow("Center Y", self.field_center_y)

        # 셀 크기
        self.field_cell_size = QLineEdit()
        self.field_cell_size.setValidator(QIntValidator(10, 1000))
        # self.field_cell_size.setText("80")  # 초기값은 필요시 동기화
        form.addRow("Cell Size", self.field_cell_size)

        form.addRow(separator)

        layout.addLayout(form)

        # --- 클릭 모드 ---
        self.label_click_mode = QLabel("Click Mode")
        self.combo_click_mode = QComboBox()
        self.combo_click_mode.addItems([
            "select_npc", "spawn_npc_at", "despawn_npc_at", "obstacle"
        ])
        layout.addWidget(self.label_click_mode)
        layout.addWidget(self.combo_click_mode)

        # --- 속도/애니메이션 ---
        self.label_interval_sec = QLabel("interval msec")
        layout.addWidget(self.label_interval_sec)

        self.combo_interval_sec = QComboBox()
        self.combo_interval_sec.addItems([
            '15', '30', '45', '60', '120', '240', '360'
        ])
        layout.addWidget(self.combo_interval_sec)

        self.label_selected_npc = QLabel("-")
        layout.addWidget(self.label_selected_npc)

        self.label_memory_usage = QLabel("_")
        layout.addWidget(self.label_memory_usage)

        self.label_total_npc_len = QLabel("-")
        layout.addWidget(self.label_total_npc_len)

        # 추후 버튼이나 리셋 등 추가 가능
        layout.addStretch()
    
    def bind_canvas(self, canvas:GridCanvas):
        """GridCanvas 객체를 설정 위젯에 연결"""
        self.canvas = canvas

        # 앱 실행중에 절대 바뀌면 안되는거
        self.label_block_size.setText(
            str(self.canvas.world.block_mgr.block_size))

        # ⬇️ 초기 상태 동기화
        self.on_grid_changed(canvas.grid_width, canvas.grid_height)
        self.on_center_changed(
            canvas.center_x, canvas.center_y)
        self.field_center_x.setText(str(self.canvas.center_x))
        self.field_center_y.setText(str(self.canvas.center_y))        
        
        self.field_cell_size.setText(str(canvas.cell_size))
        self.on_interval_sec_changed(canvas.interval_sec)

        selected = canvas.selected_npc
        if selected:
            self.on_npc_selected(selected)
        else:
            self.label_selected_npc.setText("-")

        self.label_total_npc_len.setText(
            f"총 NPC 수: {len(canvas.world.npc_mgr.npc_dict)}")
        
        mem_mb = get_memory_usage_mb()
        mem_usage = f"메모리 사용량: {mem_mb:.1f} MB"
        self.label_memory_usage.setText(mem_usage)        


        self.canvas.grid_changed.connect(self.on_grid_changed)
        self.canvas.center_changed.connect(self.on_center_changed)

        self.canvas.cell_size_changed.connect(lambda val:
            self.field_cell_size.setText(str(val)))
        
        self.canvas.world.npc_created.connect(self.on_npc_created)
        self.canvas.world.npc_deleted.connect(self.on_npc_deleted)

        self.canvas.npc_selected.connect(self.on_npc_selected)
        self.canvas.interval_sec_changed.connect(
            self.on_interval_sec_changed)

        self.field_center_x.editingFinished.connect(lambda:
            self.canvas.set_center(
                int(self.field_center_x.text()), 
                self.canvas.center_y
            )
        )

        self.field_center_y.editingFinished.connect(lambda:
            self.canvas.set_center(
                self.canvas.center_x, 
                int(self.field_center_y.text())
            )
        )        

        # 엔터나 포커스 이동 시만 반영
        self.field_cell_size.editingFinished.connect(lambda:
            self.canvas.set_cell_size(int(self.field_cell_size.text())))
        
        self.combo_click_mode.currentTextChanged.connect(
            canvas.set_click_mode
        )

        self.combo_interval_sec.currentTextChanged.connect(lambda:
            canvas.set_interval_sec(
                int(self.combo_interval_sec.currentText()))
        )
        
    @Slot(int, int)
    def on_grid_changed(self, grid_width:int, grid_height:int):
        self.label_grid_width.setText(str(grid_width))
        self.label_grid_height.setText(str(grid_height))

    @Slot(int, int)
    def on_center_changed(self, x:int, y:int):
        self.field_center_x.setText(str(self.canvas.center_x))
        self.field_center_y.setText(str(self.canvas.center_y))

    @Slot(str)
    def set_combo_click_mode(self, mode_text: str):
        idx = self.combo_click_mode.findText(mode_text)
        if idx >= 0:
            self.combo_click_mode.setCurrentIndex(idx)
        else:
            # 리스트에 없는 경우 강제로 텍스트 설정 (주의: 선택 상태 아님)
            self.combo_click_mode.setEditText(mode_text)  # editable=False면 표시만

    @Slot(NPC)
    def on_npc_selected(self, npc: NPC):
        if npc:
            full_id = npc.id
            short_id = full_id if len(full_id) <= 15 else full_id[:11] + "..."
        else:
            short_id = ''
            full_id = ''

        self.label_selected_npc.setText(f"현재 선택된 npc : {short_id}")
        self.label_selected_npc.setToolTip(f"NPC ID: {full_id}")


    @Slot(int)
    def on_interval_sec_changed(self, msec:int):
        idx = self.combo_interval_sec.findText(str(msec))
        if idx >= 0:
            self.combo_interval_sec.setCurrentIndex(idx)
        else:
            # 리스트에 없는 경우 강제로 텍스트 설정 (주의: 선택 상태 아님)
            self.combo_interval_sec.setEditText(str(msec))  # editable=False면 표시만

    @Slot(str)
    def on_npc_created(self, npc_id:str):
        # 총 NPC 수 표시
        npc_count = len(self.canvas.world.npc_mgr.npc_dict)
        full_id = npc_id
        short_id = full_id if len(full_id) <= 15 else full_id[:11] + "..."

        self.label_total_npc_len.setText(f"현재 선택된 npc : {short_id}")
        self.label_total_npc_len.setToolTip(f"NPC ID: {full_id}")        

        msg = f"""{short_id} 추가됨 
총 NPC 수: {npc_count}
"""
        self.label_total_npc_len.setText(msg)

        # 메모리 사용량 측정
        mem_mb = get_memory_usage_mb()
        mem_usage = f"메모리 사용량: {mem_mb:.1f} MB"
        self.label_memory_usage.setText(mem_usage)

    @Slot(str)
    def on_npc_deleted(self, npc_id: str):
        # 총 NPC 수 표시
        full_id = npc_id
        short_id = full_id if len(full_id) <= 15 else full_id[:11] + "..."

        self.label_total_npc_len.setText(f"현재 선택된 npc : {short_id}")
        self.label_total_npc_len.setToolTip(f"NPC ID: {full_id}")           

        npc_count = len(self.canvas.world.npc_mgr.npc_dict)
        msg = f"""{short_id} 제거됨 
총 NPC 수: {npc_count}
"""
        self.label_total_npc_len.setText(msg)

        # 메모리 사용량 측정
        mem_mb = get_memory_usage_mb()
        mem_usage = f"메모리 사용량: {mem_mb:.1f} MB"
        self.label_memory_usage.setText(mem_usage)


