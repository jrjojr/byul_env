from PySide6.QtWidgets import QDockWidget, QTabWidget
from PySide6.QtCore import Qt, Slot

from gui.canvas_setting_widget import CanvasSettingWidget  # 외부 정의된 설정 위젯
from gui.grid_canvas import GridCanvas
from gui.npc_property_widget import NpcPropertyWidget

from world.npc.npc import NPC

class SideDockingPanel(QDockWidget):
    def __init__(self, parent=None):
        super().__init__("설정 패널", parent)
        self.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea)
        self.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetClosable)

        self.parent = parent
        self.tabs = QTabWidget()
        self.setWidget(self.tabs)
        self.setFixedWidth(320)  # ✅ 창 크기 고정 (원하시는 값으로 조절)


        # ▶️ 항상 존재하는 탭: GridCanvas 설정
        self.canvas_setting_widget = CanvasSettingWidget(parent=self.tabs)
        self.tabs.addTab(self.canvas_setting_widget, "GridCanvas")

        # NPC 속성 탭 (항상 생성, 내용만 바뀜)
        self.npc_prop_widget = NpcPropertyWidget(None, parent=self.tabs)
        self.tabs.addTab(self.npc_prop_widget, "NpcProperty")

        self.tabs.setTabsClosable(True)
        self.tabs.tabCloseRequested.connect(self.on_tab_close_requested)

    def bind_canvas(self, canvas: GridCanvas):
        if self.npc_prop_widget:
            self.npc_prop_widget.bind_canvas(canvas)

        if self.canvas_setting_widget:
            self.canvas_setting_widget.bind_canvas(canvas)            

        self.check_auto_hide()

        canvas.npc_selected.connect(self.on_npc_selected)

    # def on_tab_close_requested(self, index: int):
    #     widget = self.tabs.widget(index)

    #     # 내부 참조 제거
    #     if widget == self.canvas_setting_widget:
    #         self.canvas_setting_widget = None
    #     elif widget == self.npc_prop_widget:
    #         self.npc_prop_widget = None

    #     self.tabs.removeTab(index)
    #     self.check_auto_hide()

    def on_tab_close_requested(self, index: int):
        widget = self.tabs.widget(index)

        if widget == self.canvas_setting_widget:
            self.canvas_setting_widget = None
        elif widget == self.npc_prop_widget:
            # ❗️실제로 닫지 말고 비활성화만 고려할 수도 있음
            return  # 또는 self.tabs.setTabEnabled(index, False)       

        self.tabs.removeTab(index)
        self.check_auto_hide()         

    def check_auto_hide(self):
        if self.tabs.count() == 0:
            self.setVisible(False)

    # @Slot(NPC)
    # def on_npc_selected(self, npc: NPC | None):
    #     if self.npc_prop_widget:
    #         index = self.tabs.indexOf(self.npc_prop_widget)
    #         if index >= 0:
    #             self.tabs.removeTab(index)
    #         self.npc_prop_widget = None

    #     self.npc_prop_widget = NpcPropertyWidget(
    #         npc, parent=self.tabs)
    #     self.tabs.addTab(self.npc_prop_widget, "NpcProperty")        

    @Slot(NPC)
    def on_npc_selected(self, npc: NPC | None):
        if self.npc_prop_widget:
            self.npc_prop_widget.set_npc(npc)