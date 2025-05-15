import sys
import os
import rospy
import signal
from std_msgs.msg import String
from string_service_demo.srv import StringService
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QGraphicsDropShadowEffect, QTextEdit, QMessageBox, QScrollArea, QVBoxLayout, QTextBrowser
)
from PyQt6.QtCore import Qt, QTimer, QPropertyAnimation, pyqtProperty, pyqtSignal
from PyQt6.QtGui import QPixmap, QFont, QPainter, QColor, QPainterPath, QFontDatabase
import voice_to_json

print("=== 程序开始运行 ===")

# 全局变量用于存储应用程序实例
app = None
window = None

# 信号处理函数，用于优雅地关闭程序
def signal_handler(signum, frame):
    print("\n正在关闭程序...")
    try:
        # 关闭 ROS 节点
        if rospy.is_initialized():
            print("正在关闭 ROS 节点...")
            rospy.signal_shutdown("User requested shutdown")
            print("ROS 节点已关闭")
        
        # 关闭 Qt 应用程序
        if app:
            print("正在关闭 Qt 应用程序...")
            app.quit()
            print("Qt 应用程序已关闭")
        
        # 清理其他资源
        if window and hasattr(window, 'status_widget'):
            print("正在清理状态组件...")
            window.status_widget.dot_timer.stop()
            window.status_widget.anim_timer.stop()
            print("状态组件已清理")
        
        print("程序已完全关闭")
    except Exception as e:
        print(f"关闭过程中出错: {str(e)}")
    finally:
        sys.exit(0)

# 注册信号处理器
signal.signal(signal.SIGINT, signal_handler)

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
        self.status_subscriber = rospy.Subscriber("/robot_status", String, self.status_callback)

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
            rospy.logerr(f"Error processing status message: {str(e)}")

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

# 主窗口类 - 应用程序的主要界面
class MainWindow(QMainWindow):
    PIXEL_FONT_FAMILY = "Press Start 2P"  # 定义像素风格字体
    def __init__(self):
        super().__init__()
        print("=== 初始化 MainWindow ===")
        try:
            # 初始化ROS节点
            print("正在初始化 ROS 节点...")
            rospy.init_node('robot_control_interface', anonymous=True)
            print("ROS 节点初始化成功")
            
            # 设置窗口基本属性
            self.setWindowTitle("Robot Control Interface")
            self.setFixedSize(1280, 750)
            self.setStyleSheet(f"background: {BG_COLOR};")
            
            # 初始化ROS服务客户端
            print("正在等待 ROS 服务...")
            rospy.wait_for_service("/send_string")
            self.string_service = rospy.ServiceProxy("/send_string", StringService)
            print("ROS 服务连接成功")
            
            self.setup_ui()
            print("UI 设置完成")
        except Exception as e:
            print(f"初始化过程中出错: {str(e)}")
            raise e

    def setup_ui(self):
        print("=== 开始设置 UI ===")
        try:
            # 左侧对话区
            self.left_card = CardWidget(60, 60, 373, 630, self)
            print("左侧卡片创建成功")
            
            # 聊天内容区
            self.scroll_area = QScrollArea(self.left_card)
            self.scroll_area.setGeometry(0, 0, 373, 630-100)
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
            
            # 底部语音按钮
            self.voice_btn = VoiceButtonWidget(self.left_card)
            self.voice_btn.move((373-385)//2, 630-80-8)
            self.voice_btn.resultReady.connect(self.on_voice_result)
            print("语音按钮创建成功")
            
            # 右侧状态区
            self.right_card = CardWidget(1280-60-766, 60, 766, 630, self)
            self.status_widget = RobotStatusWidget(self.right_card)
            self.status_widget.move(0, 0)
            print("右侧状态区域创建成功")
        except Exception as e:
            print(f"UI 设置过程中出错: {str(e)}")
            raise e

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

    # 滚动到聊天区底部
    def _scroll_to_bottom(self):
        bar = self.scroll_area.verticalScrollBar()
        bar.setValue(bar.maximum())

    # 处理语音识别结果
    def on_voice_result(self, transcript, json_result):
        print(f"=== 收到语音结果 ===")
        print(f"转录文本: {transcript}")
        print(f"JSON结果: {json_result}")
        
        # 添加用户消息
        self.add_user_message(transcript)
        
        # 发送服务请求
        try:
            print("正在发送服务请求...")
            response = self.string_service(json_result)
            print(f"服务响应: {response.result}")
            rospy.loginfo(f"Service Response: {response.result}")
        except Exception as e:
            print(f"服务调用失败: {str(e)}")
            rospy.logerr(f"Service call failed: {str(e)}")
        
        # 生成自然语言回复
        print(f"正在生成回复...")
        bot_reply = self.generate_bot_reply(response.result)
        print(f"生成的回复: {bot_reply}")
        self.add_bot_message(bot_reply, json_data=json_result)

    # 生成机器人回复
    def generate_bot_reply(self, data):
        import json as _json
        try:
            # 处理特殊响应
            if isinstance(data, str):
                if data.strip() == "This input seems not to be a clear instruction. Please try again.":
                    return data
                elif data.strip() == "Start":
                    return "The robot starts working"
                elif data.strip() == "Invalid":
                    return "Your destination is not available now. Please choose one from [\"sofa\", \"sink\", \"elevator\", \"lab\", \"wall\"]"
            
            # 处理JSON响应
            if isinstance(data, dict):
                obj = data.get("object", "item")
                loc = data.get("location", "somewhere")
                rec = data.get("recipient", "you")
                # 处理接收者显示
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

# 主函数
def main():
    global app, window
    print("=== 进入主函数 ===")
    try:
        # 创建Qt应用程序实例
        app = QApplication(sys.argv)
        print("QApplication 创建成功")
        
        # 加载像素风格字体
        font_path = os.path.join(os.path.dirname(__file__), "fonts", "PressStart2P-Regular.ttf")
        print("Font path exists:", os.path.exists(font_path))
        font_id = QFontDatabase.addApplicationFont(font_path)
        families = QFontDatabase.applicationFontFamilies(font_id)
        print("Loaded font families:", families)
        if families:
            MainWindow.PIXEL_FONT_FAMILY = families[0]
        else:
            print("❌ Failed to load pixel font, falling back to Courier")
            MainWindow.PIXEL_FONT_FAMILY = "Courier"
        
        # 创建并显示主窗口
        print("正在创建主窗口...")
        window = MainWindow()
        print("主窗口创建成功")
        
        print("正在显示窗口...")
        window.show()
        print("窗口显示成功")
        
        # 进入事件循环
        print("进入事件循环...")
        sys.exit(app.exec())
    except Exception as e:
        print(f"主函数执行出错: {str(e)}")
        raise e

if __name__ == '__main__':
    main()
