from PySide6.QtWidgets import ( QWidget, 
    QVBoxLayout, QLabel, QFormLayout, QDoubleSpinBox, QSpinBox, QCheckBox
    )

from PySide6.QtCore import Qt

from gui.grid_canvas import GridCanvas
from world.npc.npc import NPC
from grid.grid_cell import TerrainType

class NpcPropertyWidget(QWidget):
    def __init__(self, npc: NPC | None, parent=None):
        super().__init__(parent)
        self.npc = npc

        self.layout = QVBoxLayout()
        self.form = QFormLayout()
        self.layout.addLayout(self.form)
        self.setLayout(self.layout)

        self._build_or_empty()

    def _build_or_empty(self):
        while self.form.rowCount():
            self.form.removeRow(0)

        if self.npc is None:
            self.form.addRow(QLabel("⚠️ 선택된 NPC가 없습니다."))
            return
        
        # ── 기본 정보 ──
        self.id_label = QLabel(self.npc.id)
        self.form.addRow("🆔 NPC ID:", self.id_label)

        self.start_label = QLabel(f"({self.npc.start[0]}, {self.npc.start[1]})")
        self.form.addRow("📍 시작 위치:", self.start_label)
        
        self.goal_label = QLabel(f"({self.npc.goal[0]}, {self.npc.goal[1]})")
        self.form.addRow("📍 목표 위치:", self.goal_label)        
        
        self.next_label = QLabel(f"(0, 0)")
        self.form.addRow("📍 다음 위치:", self.next_label)

        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(0.1, 100)
        self.speed_spin.setValue(self.npc.speed_kmh)
        self.speed_spin.setSuffix(" km/h")
        self.form.addRow("🚶 이동 속도:", self.speed_spin)

        self.delay_spin = QDoubleSpinBox()
        self.delay_spin.setRange(0.0, 10.0)
        self.delay_spin.setValue(self.npc.start_delay_sec)
        self.delay_spin.setSuffix(" sec")
        self.form.addRow("⏱️ 시작 지연:", self.delay_spin)

        self.capacity_spin = QSpinBox()
        self.capacity_spin.setRange(0, 10000)
        self.capacity_spin.setValue(self.npc.route_capacity)
        self.form.addRow("🗺️ 경로 용량:", self.capacity_spin)

        self.unit_spin = QDoubleSpinBox()
        self.unit_spin.setRange(0.01, 100.0)
        self.unit_spin.setValue(self.npc.world.grid_unit_m)
        self.form.addRow("📐 그리드 단위(m):", self.unit_spin)

        self.retry_spin = QSpinBox()
        self.retry_spin.setRange(0, 10000)
        self.retry_spin.setValue(self.npc.max_retry)
        self.form.addRow("🔁 최대 재시도:", self.retry_spin)

        # ── 그래픽 설정 ──
        self.disp_dx_spin = QDoubleSpinBox()
        self.disp_dx_spin.setRange(-1000.0, 1000.0)
        self.disp_dx_spin.setValue(self.npc.disp_dx)
        self.form.addRow("disp_dx:", self.disp_dx_spin)

        self.disp_dy_spin = QDoubleSpinBox()
        self.disp_dy_spin.setRange(-1000.0, 1000.0)
        self.disp_dy_spin.setValue(self.npc.disp_dy)
        self.form.addRow("disp_dy:", self.disp_dy_spin)

        self.form.addRow(QLabel("<b>🏞️ 적합한 지형</b>"), QLabel(""))
        self.native_terrain_label = QLabel(f'{self.npc.native_terrain.name}')
        self.form.addRow(self.native_terrain_label)

        self.form.addRow(QLabel("<b>🏞️ 이동 가능 지형</b>"), QLabel(""))

        self.terrain_checkboxes = {}
        for terrain in TerrainType:
            if terrain == self.npc.native_terrain:
                continue
            if terrain == TerrainType.FORBIDDEN:
                continue
            cb = QCheckBox(terrain.name)
            cb.setChecked(terrain in self.npc.movable_terrain)
            self.form.addRow(cb)
            self.terrain_checkboxes[terrain] = cb

        # self.layout.addLayout(self.form)
        # self.setLayout(self.layout)

    def set_start_label(self):
        if not self.npc:
            return
        self.start_label.setText(f"({self.npc.start[0]}, {self.npc.start[1]})")

    def set_goal_label(self):
        self.goal_label.setText(f"({self.npc.goal[0]}, {self.npc.goal[1]})")
        
    def set_next_label(self):
        if not self.npc:
            return
        if self.npc.next:
            self.next_label.setText(
                f"({self.npc.next[0]}, {self.npc.next[1]})")

    def set_npc(self, npc: NPC):
        self.npc = npc
        self._build_or_empty()
        if npc:
            npc.start_changed_sig.connect(self.set_start_label)
            npc.goal_changed_sig.connect(self.set_goal_label)
            npc.anim_to_started_sig.connect(self.set_next_label)            

            npc.speed_kmh_changed.connect(self.speed_spin.setValue)
            self.speed_spin.valueChanged.connect(npc.set_speed_kmh)
            self.speed_spin.editingFinished.connect(
                lambda: npc.set_speed_kmh(self.speed_spin.value())
            )

            npc.start_delay_sec_changed.connect(self.delay_spin.setValue)
            self.delay_spin.valueChanged.connect(npc.set_start_delay_sec)
            self.delay_spin.editingFinished.connect(lambda:
                npc.set_start_delay_sec(self.delay_spin.value())
            )

            npc.world.grid_unit_m_changed.connect(self.unit_spin.setValue)
            self.unit_spin.valueChanged.connect(npc.world.set_grid_unit_m)
            self.unit_spin.editingFinished.connect(lambda:
                npc.world.set_grid_unit_m(self.unit_spin.value())
            )

            npc.route_capacity_changed.connect(self.capacity_spin.setValue)
            self.capacity_spin.valueChanged.connect(npc.set_route_capacity)
            self.capacity_spin.editingFinished.connect(lambda:
                npc.set_route_capacity(self.capacity_spin.value())
            )

            npc.max_retry_changed.connect(self.retry_spin.setValue)
            self.retry_spin.valueChanged.connect(npc.set_max_retry)
            self.retry_spin.editingFinished.connect(lambda:
                npc.set_max_retry(self.retry_spin.value())
            )

            npc.disp_dx_changed.connect(self.disp_dx_spin.setValue)
            self.disp_dx_spin.valueChanged.connect(npc.set_disp_dx)
            self.disp_dx_spin.editingFinished.connect(lambda:
                npc.set_disp_dx(self.disp_dx_spin.value())
            )

            npc.disp_dy_changed.connect(self.disp_dy_spin.setValue)
            self.disp_dy_spin.valueChanged.connect(npc.set_disp_dy)
            self.disp_dy_spin.editingFinished.connect(lambda:
                npc.set_disp_dy(self.disp_dy_spin.value())
            )            

            for terrain in TerrainType:
                if terrain == self.npc.native_terrain:
                    continue
                if terrain == TerrainType.FORBIDDEN:
                    continue

                cb: QCheckBox = self.terrain_checkboxes[terrain]
                
                def on_checked(state, t=terrain):  # 기본 인수로 terrain을 캡처
                    if state == Qt.Checked:
                        if t not in npc.movable_terrain:
                            npc.movable_terrain.append(t)
                    else:
                        if t in npc.movable_terrain:
                            npc.movable_terrain.remove(t)

                cb.checkStateChanged.connect(on_checked)



    def bind_canvas(self, canvas: GridCanvas):
        self.set_npc(canvas.selected_npc)
