import sys
import os
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QGraphicsDropShadowEffect
)
from PyQt6.QtCore import Qt, QTimer, QPropertyAnimation, pyqtProperty
from PyQt6.QtGui import QPixmap, QFont, QPainter, QColor, QPainterPath, QFontDatabase

BG_COLOR = "#F7F7F7"
CARD_COLOR = "#FFFFFF"
CARD_RADIUS = 16

class CardWidget(QWidget):
    def __init__(self, x, y, w, h, parent=None):
        super().__init__(parent)
        self.setGeometry(x, y, w, h)
        self.setStyleSheet(f"background: {CARD_COLOR}; border-radius: {CARD_RADIUS}px;")
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)

class ChatMessage(QWidget):
    def __init__(self, is_bot, text, parent=None):
        super().__init__(parent)
        self.setFixedHeight(56)
        self.setFixedWidth(373-16)
        # 加载Silkscreen字体并打印家族名
        font_path = os.path.join(os.path.dirname(__file__), "fonts/Silkscreen-Regular.ttf")
        font_id = QFontDatabase.addApplicationFont(font_path)
        font_families = QFontDatabase.applicationFontFamilies(font_id)
        print('Loaded Silkscreen font families:', font_families)
        if font_families:
            font = QFont(font_families[0], 14)
        else:
            font = QFont("Silkscreen", 14)
        icon = QLabel(self)
        icon.setFixedSize(24, 24)
        msg = QLabel(text, self)
        msg.setFont(font)
        msg.setWordWrap(True)
        msg.setFixedWidth(289)
        msg.setStyleSheet("color: #111;")
        if is_bot:
            icon.setPixmap(QPixmap("assets/robot.png").scaled(24, 24, Qt.AspectRatioMode.KeepAspectRatio))
            icon.move(24, 16)
            msg.move(56, 16)
            msg.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        else:
            icon.setPixmap(QPixmap("assets/human.png").scaled(24, 24, Qt.AspectRatioMode.KeepAspectRatio))
            icon.move(325, 16)
            msg.move(24, 16)
            msg.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)

class VoiceButtonWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(385, 80)  # 比按钮大，留出阴影
        base_x = (385-345)//2
        base_y = (80-40)//2
        self.base = QLabel(self)
        self.base.setGeometry(base_x, base_y, 345, 40)
        self.base.setStyleSheet("background: #fff; border-radius: 20px;")
        shadow = QGraphicsDropShadowEffect(self.base)
        shadow.setBlurRadius(24)
        shadow.setColor(QColor(0, 0, 0, int(255*0.2)))
        shadow.setOffset(0, 0)
        self.base.setGraphicsEffect(shadow)
        # 让底座可点击（初始状态下）
        self.base.setCursor(Qt.CursorShape.PointingHandCursor)
        self.base.mousePressEvent = self.start_wave_anim
        self.base.installEventFilter(self)
        # micro图标（20x20）
        self.micro_label = QLabel(self)
        self.micro_label.setPixmap(QPixmap("assets/micro.png").scaled(20, 20, Qt.AspectRatioMode.KeepAspectRatio))
        self.micro_label.setFixedSize(20, 20)
        self.micro_label.move((385-20)//2, (80-20)//2)
        self.micro_label.setCursor(Qt.CursorShape.PointingHandCursor)
        self.micro_label.mousePressEvent = self.start_wave_anim
        # wave
        self.wave_left = QLabel(self)
        self.wave_right = QLabel(self)
        self.wave_left.hide()
        self.wave_right.hide()
        # check按钮（16x16）
        self.check_btn = QLabel(self)
        self.check_btn.setPixmap(QPixmap("assets/check.png").scaled(16, 16, Qt.AspectRatioMode.KeepAspectRatio))
        self.check_btn.setFixedSize(16, 16)
        self.check_btn.move(base_x+345-16-15, base_y+12)
        self.check_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.check_btn.installEventFilter(self)
        self.check_btn.mousePressEvent = self.check_clicked
        self.check_btn.hide()  # 初始隐藏
        # 状态
        self.wave_state = 0
        self.wave_timer = QTimer(self)
        self.wave_timer.timeout.connect(self.toggle_wave)
        self.wave_animating = False
        self.micro_muted = False
        # 整个控件也可点击（兼容性）
        self.mousePressEvent = self.start_wave_anim

    def eventFilter(self, obj, event):
        if obj == self.base and not self.wave_animating and not self.micro_muted:
            if event.type() == event.Type.Enter:
                self.base.setStyleSheet("background: #F7F7F7; border-radius: 20px;")
            elif event.type() == event.Type.Leave:
                self.base.setStyleSheet("background: #fff; border-radius: 20px;")
        if obj == self.check_btn:
            if event.type() == event.Type.Enter:
                self.check_btn.setPixmap(QPixmap("assets/check_enter.png").scaled(16, 16, Qt.AspectRatioMode.KeepAspectRatio))
            elif event.type() == event.Type.Leave:
                self.check_btn.setPixmap(QPixmap("assets/check.png").scaled(16, 16, Qt.AspectRatioMode.KeepAspectRatio))
        return super().eventFilter(obj, event)

    def start_wave_anim(self, event):
        if not self.wave_animating and not self.micro_muted:
            self.wave_animating = True
            self.wave_state = 0
            self.show_wave()
            self.wave_timer.start(800)
            self.check_btn.show()  # 显示check
            # 录音时底座不可再点
            self.base.mousePressEvent = lambda e: None
            self.micro_label.mousePressEvent = None
            self.base.setStyleSheet("background: #fff; border-radius: 20px;")

    def show_wave(self):
        # wave_L1/wave_R1 or wave_L2/wave_R2，宽116高16
        if self.wave_state % 2 == 0:
            l_img = "assets/wave_L1.png"
            r_img = "assets/wave_R1.png"
        else:
            l_img = "assets/wave_L2.png"
            r_img = "assets/wave_R2.png"
        self.wave_left.setPixmap(QPixmap(l_img).scaled(116, 16, Qt.AspectRatioMode.KeepAspectRatio))
        self.wave_right.setPixmap(QPixmap(r_img).scaled(116, 16, Qt.AspectRatioMode.KeepAspectRatio))
        self.wave_left.setFixedSize(116, 16)
        self.wave_right.setFixedSize(116, 16)
        # micro正中，wave左右4px
        center_x = (385-20)//2
        center_y = (80-20)//2
        self.wave_left.move(center_x-116-4, center_y+2)
        self.wave_right.move(center_x+20+4, center_y+2)
        self.wave_left.show()
        self.wave_right.show()

    def toggle_wave(self):
        self.wave_state += 1
        self.show_wave()

    def check_clicked(self, event):
        # 停止wave动画，check消失，micro不再变mute，始终可重新录音
        self.wave_timer.stop()
        self.wave_left.hide()
        self.wave_right.hide()
        # self.micro_label.setPixmap(QPixmap("assets/micro_mute.png").scaled(20, 20, Qt.AspectRatioMode.KeepAspectRatio))
        # self.micro_muted = True
        self.check_btn.hide()
        # 允许点击micro重新录音
        self.micro_label.mousePressEvent = self.start_wave_anim
        self.wave_animating = False

    def reset_to_initial(self, event):
        self.micro_label.setPixmap(QPixmap("assets/micro.png").scaled(20, 20, Qt.AspectRatioMode.KeepAspectRatio))
        self.wave_left.hide()
        self.wave_right.hide()
        self.wave_animating = False
        self.micro_muted = False
        self.check_btn.hide()
        # 恢复底座和micro可点击
        self.base.mousePressEvent = self.start_wave_anim
        self.micro_label.mousePressEvent = self.start_wave_anim
        self.base.setStyleSheet("background: #fff; border-radius: 20px;")

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control Interface")
        self.setFixedSize(1280, 750)
        self.setStyleSheet(f"background: {BG_COLOR};")
        self.setup_ui()

    def setup_ui(self):
        # 保留左侧和右侧背景板
        self.left_card = CardWidget(60, 60, 373, 630, self)
        self.right_card = CardWidget(1280-60-766, 60, 766, 630, self)
        # 右侧框内居中放置farm.png，不做任何缩放
        map_label = QLabel(self.right_card)
        map_pix = QPixmap("assets/farm.png")
        print("farm.png isNull:", map_pix.isNull())
        if not map_pix.isNull():
            map_label.setPixmap(map_pix)
            map_label.resize(map_pix.width(), map_pix.height())
            # 上下左右居中，并整体向下移动65px，向右移动20px
            map_label.move((766 - map_pix.width()) // 2 + 20, (630 - map_pix.height()) // 2 + 65)
        else:
            map_label.setText("图片未找到")
            map_label.move((766 - 100) // 2, (630 - 30) // 2)
        # 右侧白框顶部居中显示标题
        title_label = QLabel("GIX Community Farm", self.right_card)
        title_font = QFont("Press Start 2P", 18)
        title_label.setFont(title_font)
        # 设置行高和字间距（部分属性QLabel不支持，但letter-spacing可用px近似）
        title_label.setStyleSheet("color: #111; letter-spacing: -0.54px; line-height: 20px;")
        title_label.adjustSize()
        # 居中放置，距离顶部30px
        title_label.move((766 - title_label.width()) // 2, 30 + 30)
        # 左侧对话区
        bot_msg = ChatMessage(True, "Hi, How can I help you today?", self.left_card)
        bot_msg.move(0, 32)
        human_msg = ChatMessage(False, "Please get a pen from Jack's desk.", self.left_card)
        human_msg.move(0, 104)
        # 底部语音按钮
        self.voice_btn = VoiceButtonWidget(self.left_card)
        self.voice_btn.move((373-385)//2, 630-80-8)


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
