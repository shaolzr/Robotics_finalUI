import sys
import os
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QGraphicsDropShadowEffect
)
from PyQt6.QtCore import Qt, QTimer, QPropertyAnimation, pyqtProperty
from PyQt6.QtGui import QPixmap, QFont, QPainter, QColor, QPainterPath

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
        font = QFont("Press Start 2P", 12)
        font.setStyleStrategy(QFont.StyleStrategy.PreferAntialias)
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
        # 停止wave动画，micro变mute，check消失
        self.wave_timer.stop()
        self.wave_left.hide()
        self.wave_right.hide()
        self.micro_label.setPixmap(QPixmap("assets/micro_mute.png").scaled(20, 20, Qt.AspectRatioMode.KeepAspectRatio))
        self.micro_muted = True
        self.check_btn.hide()
        # 点击micro_mute可回到初始状态
        self.micro_label.mousePressEvent = self.reset_to_initial

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

class RobotStatusWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(766, 630)
        self.status_list = [
            "Waiting to Start...",
            "Running to Kinova",
            "Kinova is working",
            "Delivering",
            "Delivered"
        ]
        self.status_idx = 0
        self.status_time = 0
        self.dot_state = 0
        self.dot_timer = QTimer(self)
        self.dot_timer.timeout.connect(self.update_dot)
        self.dot_timer.start(400)
        self.anim_timer = QTimer(self)
        self.anim_timer.timeout.connect(self.update_anim)
        self.anim_timer.start(800)
        self.state_timer = QTimer(self)
        self.state_timer.timeout.connect(self.next_status)
        self.state_timer.start(10000)
        self.turtle_x = 36
        self.turtle_carry_x = 426
        self.turtle_run_state = False
        self.kinova_run_state = False
        self.deliver_run_state = False
        self.turtle_move_step = 267 // 12
        self.carry_move_step = (672 - 426) // 12
        self.setStyleSheet("background: transparent;")

    def next_status(self):
        self.status_idx = (self.status_idx + 1) % len(self.status_list)
        self.status_time = 0
        self.turtle_x = 36
        self.turtle_carry_x = 426
        self.turtle_run_state = False
        self.kinova_run_state = False
        self.deliver_run_state = False
        self.update()

    def update_dot(self):
        if self.status_list[self.status_idx] == "Waiting to Start...":
            self.dot_state = (self.dot_state + 1) % 3
            self.update()

    def update_anim(self):
        status = self.status_list[self.status_idx]
        if status == "Running to Kinova":
            self.turtle_run_state = not self.turtle_run_state
            if self.turtle_x < 36+267:
                self.turtle_x += self.turtle_move_step
        elif status == "Kinova is working":
            self.kinova_run_state = not self.kinova_run_state
        elif status == "Delivering":
            self.deliver_run_state = not self.deliver_run_state
            if self.turtle_carry_x < 672:
                self.turtle_carry_x += self.carry_move_step
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        status = self.status_list[self.status_idx]
        painter.setFont(QFont("Press Start 2P", 30))
        painter.setPen(QColor(0,0,0))
        y_status = int(180)  # 从220改为180
        if status == "Waiting to Start...":
            dots = "." * (self.dot_state+1) + " " * (2-self.dot_state)
            painter.drawText(0, y_status, self.width(), 60, Qt.AlignmentFlag.AlignCenter, f"Waiting to Start{dots}")
        else:
            painter.drawText(0, y_status, self.width(), 60, Qt.AlignmentFlag.AlignCenter, status)
        process_pix = QPixmap("assets/process.png")
        if not process_pix.isNull():
            process_pix = process_pix.scaled(664, 44, Qt.AspectRatioMode.KeepAspectRatio)
            painter.drawPixmap((self.width()-664)//2, 380, process_pix)  # 从420改为380
        anim_y = 380 + 44 + 33  # 从420改为380
        if status == "Waiting to Start...":
            turtle_pix = QPixmap("assets/turtle.png")
            if not turtle_pix.isNull():
                turtle_pix = turtle_pix.scaled(58, 58, Qt.AspectRatioMode.KeepAspectRatio)
                painter.drawPixmap(36, anim_y, turtle_pix)
        elif status == "Running to Kinova":
            turtle_img = "assets/turtle_run.png" if self.turtle_run_state else "assets/turtle.png"
            turtle_pix = QPixmap(turtle_img)
            if not turtle_pix.isNull():
                turtle_pix = turtle_pix.scaled(58, 58, Qt.AspectRatioMode.KeepAspectRatio)
                painter.drawPixmap(self.turtle_x, anim_y, turtle_pix)
        elif status == "Kinova is working":
            kino_img = "assets/kino_put.png" if self.kinova_run_state else "assets/kino.png"
            kino_pix = QPixmap(kino_img)
            if not kino_pix.isNull():
                kino_pix = kino_pix.scaled(58, 58, Qt.AspectRatioMode.KeepAspectRatio)
                kino_x = (self.width() - 58) // 2  # process正中
                painter.drawPixmap(kino_x, anim_y, kino_pix)
                turtle_pix = QPixmap("assets/turtle.png")
                if not turtle_pix.isNull():
                    turtle_pix = turtle_pix.scaled(58, 58, Qt.AspectRatioMode.KeepAspectRatio)
                    painter.drawPixmap(kino_x - 58 - 8, anim_y, turtle_pix)
        elif status == "Delivering":
            carry_img = "assets/turtle_carryrun.png" if self.deliver_run_state else "assets/turtle_carry.png"
            carry_pix = QPixmap(carry_img)
            if not carry_pix.isNull():
                carry_pix = carry_pix.scaled(58, 70, Qt.AspectRatioMode.KeepAspectRatio)
                painter.drawPixmap(self.turtle_carry_x, anim_y, carry_pix)
        elif status == "Delivered":
            turtle_pix = QPixmap("assets/turtle.png")
            if not turtle_pix.isNull():
                turtle_pix = turtle_pix.scaled(58, 58, Qt.AspectRatioMode.KeepAspectRatio)
                painter.drawPixmap(672, anim_y, turtle_pix)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control Interface")
        self.setFixedSize(1280, 750)
        self.setStyleSheet(f"background: {BG_COLOR};")
        self.setup_ui()

    def setup_ui(self):
        # 左侧对话区
        self.left_card = CardWidget(60, 60, 373, 630, self)  # 750-120
        # 聊天内容
        bot_msg = ChatMessage(True, "Hi, How can I help you today?", self.left_card)
        bot_msg.move(0, 32)
        human_msg = ChatMessage(False, "Please get a pen from Jack's desk.", self.left_card)
        human_msg.move(0, 104)
        # 底部语音按钮
        self.voice_btn = VoiceButtonWidget(self.left_card)
        self.voice_btn.move((373-385)//2, 630-80-8)
        # 右侧进度区
        self.right_card = CardWidget(1280-60-766, 60, 766, 630, self)
        self.status_widget = RobotStatusWidget(self.right_card)
        self.status_widget.move(0, 0)


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
