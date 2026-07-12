from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPlainTextEdit, QPushButton, QLabel
)
from PySide6.QtCore import Slot


class ConsoleOutputWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        main_layout = QHBoxLayout(self)

        # 왼쪽: 출력 영역 + Clear 버튼
        left_layout = QVBoxLayout()
        self.output_box = QPlainTextEdit()
        self.output_box.setReadOnly(True)

        self.clear_button = QPushButton("Clear")
        self.clear_button.clicked.connect(self.output_box.clear)
        self.clear_button.clicked.connect(self._update_status_label)

        left_layout.addWidget(self.output_box)
        left_layout.addWidget(self.clear_button)

        # 오른쪽: 스크롤 제어 버튼들 + 상태 표시
        right_layout = QVBoxLayout()
        self.scroll_top_btn = QPushButton("⬆ 맨 위로")
        self.scroll_bottom_btn = QPushButton("⬇ 맨 아래로")
        self.status_label = QLabel("0 lines, 0 bytes")

        self.scroll_top_btn.clicked.connect(self.scroll_to_top)
        self.scroll_bottom_btn.clicked.connect(self.scroll_to_bottom)

        right_layout.addWidget(self.scroll_top_btn)
        right_layout.addWidget(self.scroll_bottom_btn)
        right_layout.addStretch()
        right_layout.addWidget(self.status_label)

        # 전체 배치
        main_layout.addLayout(left_layout)
        main_layout.addLayout(right_layout)

        # 상태 변수
        self._auto_scroll_enabled = True
        self.output_box.verticalScrollBar().valueChanged.connect(self._check_user_scroll)

        # 초기 상태 업데이트
        self._update_status_label()

    def _check_user_scroll(self, value):
        scrollbar = self.output_box.verticalScrollBar()
        self._auto_scroll_enabled = (value == scrollbar.maximum())

    def _update_status_label(self):
        text = self.output_box.toPlainText()
        num_lines = text.count('\n') + (1 if text else 0)
        num_bytes = len(text.encode('utf-8'))

        # 크기 단위 변환
        if num_bytes < 1_000:
            size_str = f"{num_bytes} bytes"
        elif num_bytes < 1_000_000:
            size_str = f"{num_bytes / 1_000:.2f} KB"
        elif num_bytes < 1_000_000_000:
            size_str = f"{num_bytes / 1_000_000:.2f} MB"
        else:
            size_str = f"{num_bytes / 1_000_000_000:.2f} GB"

        self.status_label.setText(f"{num_lines} lines, {size_str}")


    @Slot(str)
    def log(self, message):
        self.output_box.appendPlainText(message)
        self._update_status_label()
        if self._auto_scroll_enabled:
            self.scroll_to_bottom()

    def scroll_to_top(self):
        self._auto_scroll_enabled = False
        scrollbar = self.output_box.verticalScrollBar()
        scrollbar.setValue(scrollbar.minimum())

    def scroll_to_bottom(self):
        self._auto_scroll_enabled = True
        scrollbar = self.output_box.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
