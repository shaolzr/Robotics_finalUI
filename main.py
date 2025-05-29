import sys
import os
import signal
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QGraphicsDropShadowEffect, QTextEdit, QMessageBox, QScrollArea, QVBoxLayout, QTextBrowser, QStackedLayout
)
from PyQt6.QtCore import Qt, QTimer, QPropertyAnimation, pyqtProperty, pyqtSignal
from PyQt6.QtGui import QPixmap, QFont, QPainter, QColor, QPainterPath, QFontDatabase
import voice_to_json
from pathlib import Path
from openai import OpenAI
import pygame  # 用于播放音频
import numpy as np # Import numpy for matrix operations
# ROS2 imports
import rclpy
from rclpy.node import Node
from threading import Thread
from c8nav.msg import Nav2Status
from std_msgs.msg import String
import json
import datetime
from rclpy.executors import MultiThreadedExecutor
import random

# 初始化OpenAI客户端
client = OpenAI()

# 初始化pygame音频
pygame.mixer.init()

def text_to_speech(text):
    """将文本转换为语音并播放"""
    try:
        # 创建语音文件路径
        speech_file_path = Path(__file__).parent / "robot_speech.mp3"
        
        # 生成语音文件
        with client.audio.speech.with_streaming_response.create(
            model="gpt-4o-mini-tts",
            voice="coral",
            input=text,
            instructions="Speak in a cheerful and positive tone.",
        ) as response:
            response.stream_to_file(speech_file_path)
        
        # 播放语音
        pygame.mixer.music.load(str(speech_file_path))
        pygame.mixer.music.play()
        
        # 等待音频播放完成
        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)
            
    except Exception as e:
        print(f"TTS错误: {str(e)}")

print("=== 程序开始运行 ===")

# 全局变量用于存储应用程序实例
app = None
window = None



# 定义UI常量
BG_COLOR = "#F7F7F7"  # 背景颜色
CARD_COLOR = "#FFFFFF"  # 卡片颜色
CARD_RADIUS = 16  # 卡片圆角半径

# 卡片组件类 - 用于创建圆角矩形容器
class CardWidget(QWidget):
    def __init__(self, x, y, w, h, parent=None):
        super().__init__(parent)
        self.setGeometry(x, y, w, h)
        self.setStyleSheet(f"background: {CARD_COLOR}; border-radius: {CARD_RADIUS}px;")
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)

# 聊天消息组件类 - 用于显示用户和机器人的对话
class ChatMessage(QWidget):
    def __init__(self, is_bot, text, parent=None, json_data=None):
        super().__init__(parent)
        self.setFixedWidth(373-16)
        icon = QLabel(self)
        icon.setFixedSize(24, 24)
        # 用QTextBrowser显示消息内容，支持行距
        msg = QTextBrowser(self)
        msg.setHtml(f"<div style='color:#111; font-family:\"{MainWindow.PIXEL_FONT_FAMILY}\"; font-size:8pt; line-height:1.3'>{text}</div>")
        msg.setFixedWidth(289)
        msg.setStyleSheet("background: transparent; border: none;")
        msg.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        msg.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        msg.setMaximumHeight(100)
        msg.setMinimumHeight(24)
        msg.setReadOnly(True)
        self.json_data = json_data
        if is_bot:
            # 机器人消息样式
            icon.setPixmap(QPixmap("assets/robot.png").scaled(24, 24, Qt.AspectRatioMode.KeepAspectRatio))
            icon.move(24, 16)
            msg.move(56, 16)
            msg.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
            if json_data:
                self.show_json_btn = QPushButton("Show JSON", self)
                self.show_json_btn.setStyleSheet("font-size:10px;padding:2px 8px;")
                self.show_json_btn.move(56, 16+msg.sizeHint().height()+8)
                self.show_json_btn.clicked.connect(self.show_json)
                self.setFixedHeight(56 + 32)
            else:
                self.setFixedHeight(56)
        else:
            # 用户消息样式
            icon.setPixmap(QPixmap("assets/human.png").scaled(24, 24, Qt.AspectRatioMode.KeepAspectRatio))
            icon.move(325, 16)
            msg.move(24, 16)
            msg.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            self.setFixedHeight(56)
        self.msg = msg
    def show_json(self):
        dlg = QMessageBox(self)
        dlg.setWindowTitle("Command JSON")
        dlg.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        dlg.setText(f"<pre>{self.json_data}</pre>")
        dlg.setStandardButtons(QMessageBox.StandardButton.Ok)
        dlg.setStyleSheet(f"QLabel {{ font-family: '{MainWindow.PIXEL_FONT_FAMILY}', monospace; font-size: 12px; }}")
        dlg.exec()

# 语音按钮组件类 - 用于处理语音输入
class VoiceButtonWidget(QWidget):
    # 定义信号，用于传递语音识别结果
    resultReady = pyqtSignal(str, str)
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(385, 80)  # 比按钮大，留出阴影
        base_x = (385-345)//2
        base_y = (80-40)//2
        # 创建按钮底座
        self.base = QLabel(self)
        self.base.setGeometry(base_x, base_y, 345, 40)
        self.base.setStyleSheet("background: #fff; border-radius: 20px;")
        # 添加阴影效果
        shadow = QGraphicsDropShadowEffect(self.base)
        shadow.setBlurRadius(24)
        shadow.setColor(QColor(0, 0, 0, int(255*0.2)))
        shadow.setOffset(0, 0)
        self.base.setGraphicsEffect(shadow)
        # 设置鼠标事件
        self.base.setCursor(Qt.CursorShape.PointingHandCursor)
        self.base.mousePressEvent = self.start_wave_anim
        self.base.installEventFilter(self)
        
        # 麦克风图标
        self.micro_label = QLabel(self)
        self.micro_label.setPixmap(QPixmap("assets/micro.png").scaled(20, 20, Qt.AspectRatioMode.KeepAspectRatio))
        self.micro_label.setFixedSize(20, 20)
        self.micro_label.move((385-20)//2, (80-20)//2)
        self.micro_label.setCursor(Qt.CursorShape.PointingHandCursor)
        self.micro_label.mousePressEvent = self.start_wave_anim
        
        # 波形动画组件
        self.wave_left = QLabel(self)
        self.wave_right = QLabel(self)
        self.wave_left.hide()
        self.wave_right.hide()
        
        # 确认按钮
        self.check_btn = QLabel(self)
        self.check_btn.setPixmap(QPixmap("assets/check.png").scaled(16, 16, Qt.AspectRatioMode.KeepAspectRatio))
        self.check_btn.setFixedSize(16, 16)
        self.check_btn.move(base_x+345-16-15, base_y+12)
        self.check_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.check_btn.installEventFilter(self)
        self.check_btn.mousePressEvent = self.check_clicked
        self.check_btn.hide()
        
        # 状态变量
        self.wave_state = 0
        self.wave_timer = QTimer(self)
        self.wave_timer.timeout.connect(self.toggle_wave)
        self.wave_animating = False
        self.micro_muted = False
        
        # 录音相关变量
        self.audio_path = None
        self.timestamp = None

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
            # 开始录音
            voice_to_json.start_recording()

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
        # 停止录音并处理
        audio_path, timestamp = voice_to_json.stop_recording()
        self.audio_path = audio_path
        self.timestamp = timestamp
        if audio_path and timestamp:
            def on_result(transcript, json_result, error=None):
                if error:
                    transcript = f"Error: {error}"
                    json_result = ""
                self.resultReady.emit(transcript, json_result)
            voice_to_json.threaded_process(audio_path, timestamp, on_result)
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

# 机器人状态组件类 - 用于显示机器人的当前状态和动画
class RobotStatusWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(766, 630)
        # 定义所有可能的状态
        self.status_list = [
            "Waiting to Start...",
            "Running to Kinova",
            "Kinova is working",
            "Delivering",
            "Delivered"
        ]
        # 状态映射表
        self.status_mapping = {
            "starting": "Waiting to Start...",
            "navigating": "Running to Kinova",
            "manipulating": "Kinova is working",
            "deliverying": "Delivering",
            "done": "Delivered"
        }
        
        # 初始化状态变量
        self.status_idx = 0
        self.status_time = 0
        self.dot_state = 0
        
        # 设置动画定时器
        self.dot_timer = QTimer(self)
        self.dot_timer.timeout.connect(self.update_dot)
        self.dot_timer.start(400)
        
        self.anim_timer = QTimer(self)
        self.anim_timer.timeout.connect(self.update_anim)
        self.anim_timer.start(800)
        
        # 动画位置变量
        self.turtle_x = 36
        self.turtle_carry_x = 426
        self.turtle_run_state = False
        self.kinova_run_state = False
        self.deliver_run_state = False
        self.turtle_move_step = 267 // 12
        self.carry_move_step = (672 - 426) // 12
        
        self.setStyleSheet("background: transparent;")
        
        # 初始化ROS订阅者，用于接收机器人状态更新
        # Commenting out ROS1 subscriber
        # self.status_subscriber = rospy.Subscriber("/robot_status", String, self.status_callback)

    def status_callback(self, msg):
        try:
            status = msg.data
            if status in self.status_mapping:
                self.status_idx = list(self.status_mapping.keys()).index(status)
                self.status_time = 0
                self.turtle_x = 36
                self.turtle_carry_x = 426
                self.turtle_run_state = False
                self.kinova_run_state = False
                self.deliver_run_state = False
                self.update()
        except Exception as e:
            # Replace rospy.logerr with print
            print(f"Error processing status message: {str(e)}")

    def next_status(self):
        # 不再需要自动切换状态，由ROS消息控制
        pass

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

# New Map Widget Class
class MapWidget(QWidget):
    update_robot_signal = pyqtSignal(int, int)
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(1000, 740) # Use the size set in MainWindow
        self.setStyleSheet("background: transparent;")
        self.update_robot_signal.connect(self.update_robot_position)

        # 右侧框内居中放置farm.png，不做任何缩放
        map_label = QLabel(self)
        map_pix = QPixmap("assets/farm2.png")
        print("farm.png isNull:", map_pix.isNull())
        if not map_pix.isNull():
            # 使用 scaled 方法将地图放大到原来的1.2倍
            scaled_pix = map_pix.scaled(
                int(map_pix.width() * 1.1),
                int(map_pix.height() * 1.1),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation
            )
            map_label.setPixmap(scaled_pix)
            map_label.resize(scaled_pix.width(), scaled_pix.height())
            # Adjust position based on the new widget size (1000x740) to keep it centered.
            # The original image was 766x630, new container is 1000x740
            # Center X: (1000 - 766) // 2 = 117
            # Center Y: (740 - 630) // 2 = 55
            # Original offset: +20, +65
            # New position: 117 + 20 = 137, 55 + 65 = 120
            map_label.move(120, 40)  # 将 y 坐标从 120 改为 80，使图片位置更高
        else:
            map_label.setText("图片未找到")
            map_label.move((1000 - 100) // 2, (740 - 30) // 2)

        # # 右侧白框顶部居中显示标题
        # title_label = QLabel("GIX Community Farm", self)
        # # Use the pixel font defined in MainWindow
        # title_font = QFont(MainWindow.PIXEL_FONT_FAMILY, 18)
        # title_label.setFont(title_font)
        # # 设置行高和字间距（部分属性QLabel不支持，但letter-spacing可用px近似）
        # title_label.setStyleSheet("color: #111; letter-spacing: -0.54px; line-height: 20px;")
        # title_label.adjustSize()
        # # 居中放置，距离顶部30px
        # # Center X based on new widget size: (1000 - title_label.width()) // 2
        # title_label.move((1000 - title_label.width()) // 2, 30 + 30)

        # Calculate the transformation matrix (Actual -> Pixel)
        # 直接用线性系数
        self.x_coef = 23.0517
        self.x_bias = 444.2938
        self.y_coef =-26.1760
        self.y_bias = 617.8261

        # Add robot image
        self.robot_label = QLabel(self)
        robot_pixmap = QPixmap("./assets/turtle1.png")
        if not robot_pixmap.isNull():
            # Scale the robot image if needed, e.g., to 30x30
            scaled_robot_pixmap = robot_pixmap.scaled(30, 30, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
            self.robot_label.setPixmap(scaled_robot_pixmap)
            self.robot_label.resize(scaled_robot_pixmap.size())
            # Set initial position (example coordinates)
            self.update_robot_position(400, 400) # Example pixel coordinates
            self.robot_label.adjustSize()
        else:
            self.robot_label.setText("Robot image not found")
            self.robot_label.adjustSize()

        # Add input fields and button for manual position update (using real coordinates now)
        # self.x_input = QTextEdit(self)
        # self.x_input.setFixedSize(60, 30)
        # # Position below the title, adjusted for the new larger map widget size
        # self.x_input.move(10, 100) # Adjusted position
        # self.x_input.setPlaceholderText("Real X") # Indicate real coordinate input
        # self.x_input.setText("1.961") # Set initial real X value (example: pick_up x)

        # self.y_input = QTextEdit(self)
        # self.y_input.setFixedSize(60, 30)
        # # Position next to x_input, below the title
        # self.y_input.move(80, 100) # Adjusted position
        # self.y_input.setPlaceholderText("Real Y") # Indicate real coordinate input
        # self.y_input.setText("-0.349") # Set initial real Y value (example: pick_up y)

        # self.update_button = QPushButton("Update Robot Position", self) # Changed button text
        # Position next to y_input, below the title
        # self.update_button.move(150, 100) # Adjusted position
        # self.update_button.clicked.connect(self.update_position_button_clicked)

        # Update initial robot position using an example real coordinate
        example_real_pos = np.array([1.961, -0.349]) # Example: pick_up real coordinate
        pixel_x = self.x_coef * example_real_pos[0] + self.x_bias
        pixel_y = self.y_coef * example_real_pos[1] + self.y_bias
        self.update_robot_position(int(pixel_x), int(pixel_y))
        print(f"Initial robot pixel position calculated from real ({example_real_pos[0]}, {example_real_pos[1]}): ({int(pixel_x)}, {int(pixel_y)})")

        # Enable mouse tracking to get click coordinates
        self.setMouseTracking(True)

    def update_robot_position(self, x, y):
        """Updates the robot's position on the map based on pixel coordinates."""
        # Adjust position to account for the center of the robot image if needed
        # self.robot_label.move(x - self.robot_label.width() // 2, y - self.robot_label.height() // 2)
        self.robot_label.move(x, y)

    def update_position_button_clicked(self):
        """Reads real coordinates from input fields, calculates pixel coordinates, and updates robot position."""
        try:
            # Read real coordinates as floats
            real_x = float(self.x_input.toPlainText())
            real_y = float(self.y_input.toPlainText())
            
            # 直接用线性系数计算像素坐标
            pixel_x = self.x_coef * real_x + self.x_bias
            pixel_y = self.y_coef * real_y + self.y_bias

            # Update the robot's position on the map using the calculated pixel coordinates
            self.update_robot_position(int(pixel_x), int(pixel_y))
            print(f"Updated robot pixel position to: ({int(pixel_x)}, {int(pixel_y)}) from real ({real_x}, {real_y})")
        except ValueError:
            print("Invalid input. Please enter numerical values for real x and y.")
        except Exception as e:
            print(f"Error calculating or updating position: {str(e)}")

    def mousePressEvent(self, event):
        """Handles mouse clicks on the map to get pixel coordinates."""
        if event.button() == Qt.MouseButton.LeftButton:
            # Get the pixel coordinates of the click relative to the widget
            pixel_x = event.position().x()
            pixel_y = event.position().y()
            print(f"Clicked map at pixel coordinates: ({pixel_x:.2f}, {pixel_y:.2f})")

    def invoke_in_main_thread(self, func):
        # Helper to safely call a function in the Qt main thread
        QTimer.singleShot(0, func)

# 主窗口类 - 应用程序的主要界面
class MainWindow(QMainWindow):
    PIXEL_FONT_FAMILY = "Press Start 2P"  # 定义像素风格字体
    # 目的地映射表，供全局使用
    DESTINATION_MAP = {
        "Mickey's House": "sink",
        "Minnie's Bontique": "elevator",
        "Pluto's Den": "wall"
    }
    PERCEPTION_TIMEOUT = 3.0  # seconds
    def __init__(self):
        super().__init__()
        print("=== 初始化 MainWindow ===")
        self.manipulation_status = None
        self.latest_nav_status = None  # 新增：维护最新导航状态
        self.latest_manip_status = None  # 新增：维护最新机械臂状态
        self.task_id = 0
        self.available_objects = ["apple", "banana", "orange"]
        # 感知物体相关
        self._detected_object_times = {}  # {object_name: last_update_time}
        self.perception_timer = QTimer(self)
        self.perception_timer.timeout.connect(self._cleanup_detected_objects)
        self.perception_timer.start(1000)  # 每秒检查一次
        try:
            # Initialize ROS2 node instead
            print("Initializing ROS2 node...")
            # ROS2 node initialization will go here later
            print("ROS2 node initialized (placeholder)")
            
            # 设置窗口基本属性
            self.setWindowTitle("Robot Control Interface")
            # Increase window size
            self.setFixedSize(1600, 900)
            self.setStyleSheet(f"background: {BG_COLOR};")
            
            # Initialize ROS2 service client instead
            print("Waiting for ROS2 service...")
            # ROS2 service client creation will go here later
            print("ROS2 service connected (placeholder)")
            
            self.setup_ui()
            print("UI 设置完成")
        except Exception as e:
            print(f"Initialization error: {str(e)}")
            raise e

    def setup_ui(self):
        print("=== 开始设置 UI ===")
        try:
            # 左侧对话区 - Adjust size and position
            self.left_card = CardWidget(80, 80, 450, 740, self) # Increased size and adjusted position
            print("左侧卡片创建成功")
            
            # 聊天内容区 - Adjust size
            self.scroll_area = QScrollArea(self.left_card)
            self.scroll_area.setGeometry(0, 0, 450, 740-100) # Adjusted size
            self.scroll_area.setWidgetResizable(True)
            self.scroll_area.setStyleSheet("background:transparent; border:none;")
            print("滚动区域创建成功")
            
            # 聊天消息布局
            self.chat_area = QWidget()
            self.chat_layout = QVBoxLayout(self.chat_area)
            self.chat_layout.setContentsMargins(0, 24, 0, 24)
            self.chat_layout.setSpacing(16)
            self.scroll_area.setWidget(self.chat_area)
            print("聊天区域创建成功")
            
            # 添加初始机器人消息
            self.add_bot_message("Hi, How can I help you today?")
            print("初始消息添加成功")
            
            # 底部语音按钮 - Adjust horizontal position to center
            self.voice_btn = VoiceButtonWidget(self.left_card)
            self.voice_btn.move((450-385)//2, 740-80-8) # Adjusted horizontal position and vertical based on new card height
            self.voice_btn.resultReady.connect(self.on_voice_result)
            print("语音按钮创建成功")
            
            # 右侧状态和地图区容器 - Adjust size and position
            self.right_card = CardWidget(1600-80-1000, 80, 1000, 740, self) # Increased size and adjusted position
            print("右侧卡片创建成功")

            # Create a container widget for the stacked layout - Adjust size
            self.right_view_container = QWidget(self.right_card)
            self.right_view_container.setGeometry(0, 0, 1000, 740) # Adjusted size

            # 只显示地图页面
            self.map_widget = MapWidget(self.right_view_container)
            self.map_widget.setFixedSize(1000, 740)
            self.map_widget.show()

            # # Create the stacked layout and widgets
            # self.right_stacked_layout = QStackedLayout(self.right_view_container)
            # self.status_widget = RobotStatusWidget(self.right_view_container)
            # self.map_widget = MapWidget(self.right_view_container)
            # self.status_widget.setFixedSize(1000, 740)
            # self.map_widget.setFixedSize(1000, 740)
            # self.right_stacked_layout.addWidget(self.status_widget)
            # self.right_stacked_layout.addWidget(self.map_widget)
            # self.right_stacked_layout.setCurrentIndex(1) # 只显示地图
            # print("右侧状态区域创建成功")

            # # Add buttons to switch views - Adjust vertical position
            # self.status_button = QPushButton("Show Status", self.right_card)
            # self.map_button = QPushButton("Show Map", self.right_card)
            # button_y = 10 # Example vertical position
            # self.status_button.move(10, button_y)
            # self.map_button.move(120, button_y)
            # self.status_button.clicked.connect(self.show_status_view)
            # self.map_button.clicked.connect(self.show_map_view)
            # print("View switching buttons created and connected")

        except Exception as e:
            print(f"UI Setup error: {str(e)}")
            raise e

    # Slot to show the status view
    def show_status_view(self):
        self.right_stacked_layout.setCurrentIndex(0)
        print("Switched to status view")

    # Slot to show the map view
    def show_map_view(self):
        self.right_stacked_layout.setCurrentIndex(1)
        print("Switched to map view")

    # 添加用户消息到聊天区
    def add_user_message(self, text):
        msg = ChatMessage(False, text, self.chat_area)
        self.chat_layout.addWidget(msg)
        self._scroll_to_bottom()

    # 添加机器人消息到聊天区
    def add_bot_message(self, text, json_data=None):
        msg = ChatMessage(True, text, self.chat_area, json_data=json_data)
        self.chat_layout.addWidget(msg)
        self._scroll_to_bottom()
        
        # 使用TTS播放机器人回复
        try:
            # 在新线程中运行TTS，避免阻塞UI
            import threading
            threading.Thread(target=text_to_speech, args=(text,), daemon=True).start()
        except Exception as e:
            print(f"TTS线程启动失败: {str(e)}")

    # 滚动到聊天区底部
    def _scroll_to_bottom(self):
        bar = self.scroll_area.verticalScrollBar()
        bar.setValue(bar.maximum())

    # 处理语音识别结果
    def on_voice_result(self, transcript, _):
        # 先显示用户说的话
        self.add_user_message(transcript)
        # 获取最新状态，构造 context
        nav = self.latest_nav_status
        manip = self.latest_manip_status
        context = ""
        if nav:
            # 设置 ETA 默认值
            eta = getattr(nav, 'estimated_time_to_goal', None)
            if eta is None or eta == 0:
                eta = random.uniform(0, 20)
            context += f"Navigation status: position=({nav.position.x:.2f}, {nav.position.y:.2f}), distance_to_goal={nav.distance_to_goal:.2f} meters , eta={eta:.2f} seconds\n"
        if manip:
            context += f"Manipulation status: {manip}\n"
        # 让 voice_to_json 直接调用 LLM，带 context
        structured_json = voice_to_json.parse_command_with_llm(transcript, context=context)
        print(f"structured_json: {structured_json}")
        try:
            # Try to parse as JSON (may be a string or dict)
            if isinstance(structured_json, dict):
                result = structured_json
            else:
                result = json.loads(structured_json)
        except Exception:
            # If not valid JSON, show the raw output
            self.add_bot_message("Sorry, I could not understand your request.", json_data=structured_json)
            return
        # 对 destination 字段做别名映射（无论是 command 还是 query）
        if "destination" in result:
            dest = result["destination"]
            if dest in self.DESTINATION_MAP:
                result["destination"] = self.DESTINATION_MAP[dest]
        if result.get("type") == "command":
            is_valid, checked = self.validate_command(json.dumps(result, ensure_ascii=False))
            if is_valid:
                self.command_publisher.publish_command(checked["object"], checked["destination"], checked["id"])
                # 显示别名（如有），否则显示内部名
                display_dest = self.get_destination_alias(checked['destination'])
                bot_reply = f"I will fetch {checked['object']} and deliver it to the {display_dest}."
            else:
                bot_reply = f"Error: {checked}"
        elif result.get("type") == "query":
            # Robustly handle the answer field, fallback to showing the whole result if missing
            answer = result.get("answer")
            if answer:
                bot_reply = answer
            else:
                # Show the whole result as fallback
                bot_reply = f"Query result: {json.dumps(result, ensure_ascii=False)}"
        else:
            bot_reply = "Sorry, I could not understand your request."
        self.add_bot_message(bot_reply, json_data=structured_json)

    # 生成机器人回复
    def generate_bot_reply(self, data):
        # 该函数已不再用于 LLM query 场景，仅保留兼容性
        import json as _json
        try:
            if isinstance(data, str):
                if data.strip() == "Start":
                    return "The robot starts working"
                elif data.strip() == "Invalid":
                    return "Your destination is not available now. Please choose one from [\"sofa\", \"sink\", \"elevator\", \"lab\", \"wall\"]"
                else:
                    return data
            if isinstance(data, dict):
                obj = data.get("object", "item")
                loc = data.get("location", "somewhere")
                rec = data.get("recipient", "you")
                if isinstance(rec, str) and rec.strip().lower() == "me":
                    rec_disp = "you"
                else:
                    rec_disp = rec
                return f"OK, I will fetch {obj} from {loc} for {rec_disp}."
            else:
                return str(data)
        except Exception:
            if "not a clear instruction" in str(data):
                return "Sorry, I could not understand your command."
            return str(data)

    def validate_command(self, command_json):
        try:
            command = json.loads(command_json)
            # 检查是否有错误
            if "error" in command:
                return False, command["error"]
            # 检查必要字段 Must be one of: {valid_destinations}"
            # 验证对象是否在可用列表中
            if command["object"] not in self.available_objects:
                return False, f"Object '{command['object']}' is not available"
            # 新增：每次收到有效任务，id+1
            self.task_id += 1
            command["id"] = self.task_id
            return True, command
        except json.JSONDecodeError:
            return False, "Invalid JSON format"
        except Exception as e:
            return False, str(e)

    def update_nav_status(self, nav_status):
        self.latest_nav_status = nav_status
        print(f'[MainWindow] Updated nav status: position=({nav_status.position.x:.2f}, {nav_status.position.y:.2f}), distance_to_goal={nav_status.distance_to_goal:.2f}, eta={nav_status.estimated_time_to_goal:.2f}')

    def update_manipulation_status(self, status):
        self.manipulation_status = status
        self.latest_manip_status = status
        print(f'[MainWindow] Manipulation status updated: {status}')
        # 你可以在这里加UI刷新逻辑，比如显示在界面某个label上

    def update_available_objects(self, objects):
        self.available_objects = objects
        print(f'[MainWindow] Updated available objects: {objects}')

    # Helper: internal destination to alias (for display)
    def get_destination_alias(self, internal_name):
        for alias, code in self.DESTINATION_MAP.items():
            if code == internal_name:
                return alias
        return internal_name  # fallback to code if no alias

    def update_detected_object(self, object_name):
        import time
        now = time.time()
        self._detected_object_times[object_name] = now
        print(f'[Perception] Current objects: {self.detected_object_set}')

    def _cleanup_detected_objects(self):
        import time
        now = time.time()
        to_remove = [obj for obj, t in self._detected_object_times.items() if now - t > self.PERCEPTION_TIMEOUT]
        for obj in to_remove:
            del self._detected_object_times[obj]

    @property
    def detected_object_set(self):
        # 只保留3秒内有效的唯一物体名
        return set(self._detected_object_times.keys())

# 信号处理函数，用于优雅地关闭程序
def signal_handler(signum, frame):
    print("\nReceived Ctrl+C, shutting down...")
    try:
        # 停止所有音频播放
        pygame.mixer.music.stop()
        pygame.mixer.quit()
        
        # remove close ROS node block
        
        # 关闭 Qt 应用程序
        if app:
            print("Quitting Qt application...")
            app.quit()
            print("Qt application quit signal sent.")
        
        # 清理其他资源
        if window and hasattr(window, 'status_widget'):
            print("Cleanup complete.")
            window.status_widget.dot_timer.stop()
            window.status_widget.anim_timer.stop()
            print("状态组件已清理")
        
        print("程序已完全关闭")
    except Exception as e:
        print(f"Error during shutdown: {str(e)}")
    finally:
        sys.exit(0)

# 主函数
def main():
    global app, window
    print("=== 进入主函数 ===")
    try:
        rclpy.init()  # 只在这里调用一次
        app = QApplication(sys.argv)
        print("QApplication 创建成功")
        window = MainWindow()
        print("主窗口创建成功")
        print("正在显示窗口...")
        window.show()
        print("窗口显示成功")
        
        # 创建所有 ROS2 node
        nav_node = ROS2NavListener(window.map_widget, window.update_nav_status)
        manipulation_node = ManipulationStatusListener(window.update_manipulation_status)
        window.command_publisher = CommandPublisher()
        object_list_node = ObjectListListener(window.update_available_objects)
        perception_node = PerceptionListener(window.update_detected_object)

        # 创建 executor 并添加所有 node
        executor = MultiThreadedExecutor()
        executor.add_node(nav_node)
        executor.add_node(manipulation_node)
        executor.add_node(window.command_publisher)
        executor.add_node(object_list_node)
        executor.add_node(perception_node)

        # 启动 ROS2 executor 在后台线程
        from threading import Thread
        Thread(target=executor.spin, daemon=True).start()

        print("进入事件循环...")
        sys.exit(app.exec())
    except Exception as e:
        print(f"主函数执行出错: {str(e)}")
        raise e

class ROS2NavListener(Node):
    def __init__(self, map_widget, update_nav_status_callback=None):
        super().__init__('nav2_status_listener')
        self.map_widget = map_widget
        self.update_nav_status_callback = update_nav_status_callback
        self.subscription = self.create_subscription(
            Nav2Status,
            'nav2_status',
            self.listener_callback,
            10
        )
        print('[ROS2NavListener] Successfully subscribed to nav2_status topic.')

    def listener_callback(self, msg):
        real_x = msg.position.x
        real_y = msg.position.y
        print(f'[ROS2NavListener] Received real position: ({real_x}, {real_y})')
        pixel_x = self.map_widget.x_coef * real_x + self.map_widget.x_bias
        pixel_y = self.map_widget.y_coef * real_y + self.map_widget.y_bias
        print(f'[ROS2NavListener] Calculated pixel position: ({int(pixel_x)}, {int(pixel_y)})')
        self.map_widget.update_robot_signal.emit(int(pixel_x), int(pixel_y))
        if self.update_nav_status_callback:
            self.update_nav_status_callback(msg)

class ObjectListListener(Node):
    def __init__(self, callback):
        super().__init__('object_list_listener')
        self.callback = callback
        # 暂时注释掉订阅，等待确认正确的消息类型
        # self.subscription = self.create_subscription(
        #     ObjectList,
        #     'object_list',
        #     self.listener_callback,
        #     10
        # )
        print('[ObjectListListener] Waiting for object_list message type confirmation.')

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher = self.create_publisher(
            String,  # 使用标准消息类型
            'robot_command',
            10
        )
        print('[CommandPublisher] Successfully created command publisher.')

    def publish_command(self, object_name, destination, task_id=None):
        msg = String()
        # 将命令信息序列化为 JSON 字符串
        command_data = {
            "id": task_id,
            "object": object_name,
            "destination": destination
        }
        msg.data = json.dumps(command_data)
        self.publisher.publish(msg)
        print(f'[CommandPublisher] Published command: fetch {object_name} to {destination} (id={task_id})')

class ManipulationStatusListener(Node):
    def __init__(self, update_status_callback):
        super().__init__('manipulation_status_listener')
        self.update_status_callback = update_status_callback
        self.last_status = None
        self.subscription = self.create_subscription(
            String,
            '/robot_status',
            self.status_callback,
            10
        )
        print('[ManipulationStatusListener] Listening on /robot_status')

    def status_callback(self, msg):
        status = msg.data
        if status == self.last_status:
            return
        ts = datetime.datetime.now().strftime('%H:%M:%S')
        print(f'[ManipulationStatusListener] [{ts}] status = {status}')
        self.last_status = status
        self.update_status_callback(status)

class PerceptionListener(Node):
    def __init__(self, update_callback):
        super().__init__('perception_listener')
        from trans_msg.msg import FinalDetection
        self.subscription = self.create_subscription(
            FinalDetection,
            'perception_topic',  # TODO: 替换为实际topic名
            self.listener_callback,
            10
        )
        self.update_callback = update_callback
        print('[PerceptionListener] Subscribed to perception_topic')

    def listener_callback(self, msg):
        object_name = msg.object_name
        print(f'[PerceptionListener] Detected object: {object_name}')
        self.update_callback(object_name)

if __name__ == '__main__':
    main()
