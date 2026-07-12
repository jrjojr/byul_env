import sys

from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtGui import QCursor, QKeyEvent
from PySide6.QtCore import QTimer, Qt, QEvent

from grid_canvas import GridCanvas
from menu_bar import MenuBar
from side_panel import SideDockingPanel
from bottom_panel import BottomDockingPanel
from toolbar_panel import ToolbarPanel
from actions import Actions

from world.world import World

class GridViewer(QMainWindow):
    def __init__(self, world:World):
        super().__init__()
        self.setWindowTitle("BYUL Grid")

        self.world = world

        QApplication.instance().focusWindowChanged.connect(
            self._on_focus_window_changed)

        # === Core Components ===
        self.grid_canvas = GridCanvas(world, parent=self, min_px=10)
        self.grid_canvas.full_redraw = True

        self.setCentralWidget(self.grid_canvas)

        # 바톰 패널
        self.bottom_panel = BottomDockingPanel(self)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.bottom_panel)
        
        # 사이드 패널
        self.side_panel = SideDockingPanel(self)
        self.addDockWidget(Qt.RightDockWidgetArea, self.side_panel)
        self.side_panel.bind_canvas(self.grid_canvas)

        self.actions = Actions(world, self)

        self.menu_bar = MenuBar(self.actions, self)
        self.setMenuBar(self.menu_bar)

        self.toolbar_panel = ToolbarPanel(self.actions, self)
        self.addToolBar(Qt.TopToolBarArea, self.toolbar_panel)

        # === UI Finalization ===
        self.bottom_panel.console_widget.log("✅ Console log test")
        QTimer.singleShot(100, self.center_window)
        QTimer.singleShot(0, self.grid_canvas.setFocus)

    def center_window(self):
        screen = QApplication.screenAt(QCursor.pos()) or \
                 QApplication.primaryScreen()
        screen_geometry = screen.geometry()
        size = self.frameGeometry()
        self.move(
(screen_geometry.width() - size.width()) // 2 + screen_geometry.x(),
(screen_geometry.height() - size.height()) // 2 + screen_geometry.y()
        )

    def toggle_fullscreen(self):
        if self.isFullScreen():
            # ⇨ 일반 모드로 복귀
            self.side_panel.show()
            self.bottom_panel.show()
            self.toolbar_panel.show()
            self.menuBar().show()
            self.showNormal()
            self.menuBar()._action_fullscreen.setChecked(False)
        else:
            # ⇨ 풀스크린 진입
            self.side_panel.hide()
            self.bottom_panel.hide()
            # self.toolbar_panel.hide()
            self.menuBar().hide()
            self.showFullScreen()
            self.grid_canvas.setFocus()
            self.menuBar()._action_fullscreen.setChecked(True)

    def _on_focus_window_changed(self, window):
        if window != self.window():
            # 다른 창으로 전환됨
            self.grid_canvas._pressed_keys.clear()

    def _on_focus_window_changed(self, window):
        if window != self.window():
            for key in list(self.grid_canvas._pressed_keys):
                fake_release = QKeyEvent(
                    QEvent.KeyRelease, key, Qt.NoModifier)
                self.keyReleaseEvent(fake_release)
            self.grid_canvas._pressed_keys.clear()            
