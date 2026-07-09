"""
吸烟检测上位机 — PySide6 Qt 客户端
  - RTSP 视频拉流 + 实时 AI 标注框叠加
  - TCP 接收 AI 坐标 (32字节协议)
  - 报警日志 + 截图保存 + 配置记忆
"""
import sys
import os
import json
import cv2
import time
import struct
from pathlib import Path
from collections import deque

# 修复 OpenCV 自带 Qt 插件与 PySide6 冲突
import PySide6
os.environ['QT_PLUGIN_PATH'] = os.path.join(
    os.path.dirname(PySide6.__file__), 'Qt', 'plugins')
os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'rtsp_transport;tcp'

import numpy as np
import socket as sock_mod  # UDP 音频推流

from PySide6.QtCore import QTimer, Qt, QSettings, QRect, QUrl, QProcess, QPointF, QThread, Signal
from PySide6.QtGui import (
    QImage, QPixmap, QPainter, QPen, QColor, QFont,
    QAction, QKeySequence, QMouseEvent, QDesktopServices,
    QPolygonF, QLinearGradient
)
from PySide6.QtNetwork import QTcpSocket
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton,
    QTextEdit, QVBoxLayout, QHBoxLayout, QMessageBox,
    QDialog, QFormLayout, QLineEdit, QDialogButtonBox,
    QGroupBox, QSpinBox, QStatusBar, QFrame,
    QTableWidget, QTableWidgetItem, QHeaderView, QSplitter, QSlider
)

# ================================================================
#  DVR 帧缓存 — 最近10分钟录像, 暂停时可回放
# ================================================================
import glob as _glob
class FrameBuffer:
    """内存帧缓存: 保存最近N分钟的视频帧(JPEG), 支持时间轴回放"""

    BUFFER_SECS  = 600    # 缓存时长 (秒) = 10分钟
    CAPTURE_EVERY = 5     # 每5帧采1帧 (30fps÷5=6fps)
    JPEG_QUALITY = 50     # JPEG压缩质量

    def __init__(self):
        self.frames = deque(maxlen=6000)  # (timestamp_sec, jpeg_bytes)
        self._last_capture_ts = 0

    def feed(self, frame_bgr, timestamp_sec):
        """喂入一帧BGR图像, 按采样率决定是否缓存"""
        if timestamp_sec - self._last_capture_ts < 1.0 / (30.0 / self.CAPTURE_EVERY):
            return  # 未到采样间隔
        self._last_capture_ts = timestamp_sec
        # 压缩为JPEG
        ok, jpg = cv2.imencode('.jpg', frame_bgr,
                               [cv2.IMWRITE_JPEG_QUALITY, self.JPEG_QUALITY])
        if ok:
            self.frames.append((timestamp_sec, jpg.tobytes()))
            self._cleanup(timestamp_sec)

    def _cleanup(self, now_sec):
        """清理超过BUFFER_SECS的旧帧"""
        cutoff = now_sec - self.BUFFER_SECS
        while self.frames and self.frames[0][0] < cutoff:
            self.frames.popleft()

    def nearest(self, target_sec):
        """查找最接近target_sec的帧, 返回(ts, bgr_frame)或None"""
        if not self.frames:
            return None
        best = min(self.frames, key=lambda f: abs(f[0] - target_sec))
        img = cv2.imdecode(np.frombuffer(best[1], np.uint8), cv2.IMREAD_COLOR)
        return (best[0], img) if img is not None else None

    def time_range(self):
        """返回缓冲区时间范围 (earliest, latest)"""
        if not self.frames:
            return (0, 0)
        return (self.frames[0][0], self.frames[-1][0])

    def frame_count(self):
        return len(self.frames)

# ---- 配置路径 ----
CONFIG_PATH = Path(__file__).parent / "qt_monitor_config.json"
HISTORY_PATH = Path(__file__).parent / "alarm_history.json"

# ---- 协议 (68 字节: 4B slave_id + 64B 原格式) ----
# slave_id: 0=AI检测, 1=从机1传感器, 2=从机2传感器
MSG_SIZE = 68
MSG_FMT = '<iiiiiiifiiffiifff'  # i(slave_id) + 6i+f+2i+2f+2i+f+2f = 17 fields

# ---- 类别颜色 ----
CLASS_NAMES = {0: 'no_smoking', 1: 'smoking', 2: 'fire', 3: 'smoke'}
CLASS_COLORS = {0: (128, 128, 128), 1: (0, 255, 0), 2: (255, 50, 0), 3: (255, 200, 0)}  # 灰/绿/红/黄

# ---- 自动截图 ----
AUTO_SNAP_WINDOW    = 5    # 统计窗口 (秒)
AUTO_SNAP_RATIO     = 0.8  # 检测帧占比阈值 (80%)
AUTO_SNAP_COOLDOWN  = 5    # 截图间隔 (秒)
TRAIL_LENGTH = 5


def load_config():
    """加载配置"""
    if CONFIG_PATH.exists():
        try:
            return json.loads(CONFIG_PATH.read_text(encoding='utf-8'))
        except Exception:
            pass
    return {
        'host': '192.168.1.10',
        'rtsp_port': 8554,
        'tcp_port': 9999,
        'log_dir': str(Path.home() / 'smoking_logs'),
        'snap_dir': str(Path.home() / 'Pictures' / 'smoking_snapshots'),
        'fullscreen_offset_x': 300,
    }


def save_config(cfg):
    """保存配置"""
    CONFIG_PATH.write_text(json.dumps(cfg, indent=2, ensure_ascii=False),
                           encoding='utf-8')


def load_history():
    """加载历史报警记录"""
    if HISTORY_PATH.exists():
        try:
            return json.loads(HISTORY_PATH.read_text(encoding='utf-8'))
        except Exception:
            pass
    return []


def save_history(records):
    """保存历史报警记录"""
    HISTORY_PATH.write_text(json.dumps(records, indent=2, ensure_ascii=False),
                            encoding='utf-8')


def add_history_record(screenshot_path, confidence, duration):
    """添加一条报警记录到历史"""
    records = load_history()
    records.append({
        'time': time.strftime('%Y-%m-%d %H:%M:%S'),
        'confidence': round(confidence, 2),
        'duration': round(duration, 1),
        'screenshot': str(screenshot_path),
    })
    # 最多保留 500 条
    if len(records) > 500:
        records = records[-500:]
    save_history(records)


# ================================================================
#  多源融合评分引擎 (吸烟/火灾分开评分)
# ================================================================
class FusionEngine:
    """板端评分 + Qt时间窗口判定"""

    SMOKE_TH1 = 6.5; SMOKE_TH2 = 7.5
    FIRE_TH1  = 5.5; FIRE_TH2  = 6.5

    STATES = {0:'正常', 1:'疑似吸烟', 2:'吸烟报警', 3:'疑似火灾', 4:'火灾报警', 5:'环境异常'}
    STATE_COLORS = {0:'#5f5', 1:'#ff0', 2:'#f80', 3:'#f55', 4:'#f00', 5:'#f80'}

    def __init__(self):
        self.current_state = 0
        self._state1_start = 0
        self.history = deque(maxlen=300)
        self.scores = {}

    def update_from_board(self, has_target, class_id, smoke_score, fire_score, sensor_alarm=0):
        now = time.time()
        self.history.append((now, smoke_score, fire_score))
        cutoff = now - 8
        while self.history and self.history[0][0] < cutoff:
            self.history.popleft()

        def ratio_above(seconds, idx, th, min_frames=5):
            t0 = now - seconds
            vals = [h[idx] for h in self.history if h[0] >= t0]
            if len(vals) < min_frames: return 0.0
            return sum(1 for v in vals if v >= th) / len(vals)

        now = time.time()
        new_state = self.current_state  
        if ratio_above(3, 2, self.FIRE_TH2, 3) >= 0.8 or ratio_above(3, 2, self.FIRE_TH1, 5) >= 0.8:
            new_state = 4                                     
        elif ratio_above(5, 2, self.FIRE_TH1, 10) >= 0.8:
            new_state = 3                                     
        elif ratio_above(3, 1, self.SMOKE_TH2, 3) >= 0.8:
            new_state = 2                                       
        elif ratio_above(3, 1, self.SMOKE_TH1, 5) >= 0.6:
            if self.current_state == 1:
              
                if ratio_above(5, 1, self.SMOKE_TH1, 10) >= 0.6:
                    new_state = 2
            else:
                new_state = 1                                   
        elif sensor_alarm == 3:
            new_state = 5                                      
        else:
            new_state = 0                                      

        changed = (new_state != self.current_state)
        if changed and new_state == 1:
            self._state1_start = now  
        self.current_state = new_state
        self.scores = {
            'smoke': round(smoke_score, 1), 'smoke_th1': self.SMOKE_TH1, 'smoke_th2': self.SMOKE_TH2,
            'fire': round(fire_score, 1), 'fire_th1': self.FIRE_TH1, 'fire_th2': self.FIRE_TH2,
        }
        return (self.STATES[self.current_state], self.STATE_COLORS[self.current_state],
                self.scores, changed)


# ================================================================
#  设置对话框
# ================================================================
class SettingsDialog(QDialog):
    def __init__(self, config, parent=None):
        super().__init__(parent)
        self.setWindowTitle("设置")
        self.resize(480, 300)
        self.cfg = config

        layout = QFormLayout(self)

        self.host_edit = QLineEdit(config.get('host', ''))
        self.host_edit.setPlaceholderText("192.168.1.10")
        layout.addRow("开发板 IP:", self.host_edit)

        self.rtsp_spin = QSpinBox()
        self.rtsp_spin.setRange(1, 65535)
        self.rtsp_spin.setValue(config.get('rtsp_port', 8554))
        layout.addRow("RTSP 端口:", self.rtsp_spin)

        self.port_spin = QSpinBox()
        self.port_spin.setRange(1, 65535)
        self.port_spin.setValue(config.get('tcp_port', 9999))
        layout.addRow("AI 坐标 TCP 端口:", self.port_spin)

        self.log_edit = QLineEdit(config.get('log_dir', ''))
        layout.addRow("日志保存目录:", self.log_edit)

        self.snap_edit = QLineEdit(config.get('snap_dir', ''))
        layout.addRow("截图保存目录:", self.snap_edit)

        btn = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btn.accepted.connect(self.accept)
        btn.rejected.connect(self.reject)
        layout.addRow(btn)

        self.setStyleSheet("""
            QDialog { background: #2b2b2b; color: #ddd; }
            QLabel { font-size: 14px; }
            QLineEdit, QSpinBox {
                background: #3c3c3c; color: #fff; border: 1px solid #555;
                padding: 4px; font-size: 14px;
            }
        """)

    def get_config(self):
        return {
            'host': self.host_edit.text(),
            'rtsp_port': self.rtsp_spin.value(),
            'tcp_port': self.port_spin.value(),
            'log_dir': self.log_edit.text(),
            'snap_dir': self.snap_edit.text(),
        }


# ================================================================
#  历史报警记录对话框
# ================================================================
class HistoryDialog(QDialog):
    COL_TIME, COL_CONF, COL_DUR, COL_PATH, COL_OPEN = range(5)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("历史报警记录")
        self.resize(850, 500)
        self.records = load_history()

        layout = QVBoxLayout(self)

        # 统计信息
        total = len(self.records)
        today = sum(1 for r in self.records
                    if r.get('time', '')[:10] == time.strftime('%Y-%m-%d'))
        lbl_stats = QLabel(f"共 {total} 条记录  |  今日: {today} 条")
        lbl_stats.setStyleSheet("color: #aaa; font-size: 14px; padding: 4px;")
        layout.addWidget(lbl_stats)

        # 表格
        self.table = QTableWidget()
        self.table.setColumnCount(5)
        self.table.setHorizontalHeaderLabels(
            ["时间", "置信度", "持续(s)", "截图路径", ""])
        self.table.setRowCount(len(self.records))
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setAlternatingRowColors(True)
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(3, QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(4, QHeaderView.ResizeToContents)
        self.table.verticalHeader().setVisible(False)
        self.table.cellClicked.connect(self.on_cell_clicked)

        for row, rec in enumerate(reversed(self.records)):
            self._fill_row(row, rec)

        layout.addWidget(self.table)

        # 底部按钮
        btn_layout = QHBoxLayout()
        btn_open_dir = QPushButton("📁 打开截图目录")
        btn_open_dir.clicked.connect(self.open_snap_dir)
        btn_clear_hist = QPushButton("🗑 清空历史")
        btn_clear_hist.clicked.connect(self.clear_history)
        btn_close = QPushButton("关闭")
        btn_close.clicked.connect(self.accept)

        btn_layout.addWidget(btn_open_dir)
        btn_layout.addWidget(btn_clear_hist)
        btn_layout.addStretch()
        btn_layout.addWidget(btn_close)
        layout.addLayout(btn_layout)

        self.setStyleSheet("""
            QDialog { background: #2b2b2b; color: #ddd; }
            QTableWidget {
                background: #1e1e1e; color: #ccc;
                gridline-color: #444; border: 1px solid #444;
                font-size: 13px;
            }
            QTableWidget::item { padding: 4px; }
            QTableWidget::item:selected { background: #3a5a3a; }
            QHeaderView::section {
                background: #333; color: #ddd;
                padding: 4px; border: 1px solid #444;
            }
            QPushButton {
                background: #3a3a3a; color: #ddd;
                border: 1px solid #555; border-radius: 4px;
                padding: 6px 14px; font-size: 13px;
            }
            QPushButton:hover { background: #4a4a4a; }
            QLabel { color: #aaa; }
        """)

    def _fill_row(self, row, rec):
        # 时间
        self.table.setItem(row, self.COL_TIME,
                           QTableWidgetItem(rec.get('time', '')))
        # 置信度
        conf_item = QTableWidgetItem(f"{rec.get('confidence', 0):.2f}")
        conf = rec.get('confidence', 0)
        if conf >= 0.8:
            conf_item.setForeground(QColor(0, 255, 0))
        elif conf >= 0.6:
            conf_item.setForeground(QColor(255, 200, 0))
        else:
            conf_item.setForeground(QColor(255, 100, 100))
        self.table.setItem(row, self.COL_CONF, conf_item)
        # 持续时间
        self.table.setItem(row, self.COL_DUR,
                           QTableWidgetItem(f"{rec.get('duration', 0):.1f}"))
        # 截图路径
        path = rec.get('screenshot', '')
        path_item = QTableWidgetItem(path)
        path_item.setToolTip(path)
        self.table.setItem(row, self.COL_PATH, path_item)
        # 打开按钮
        if path and Path(path).exists():
            btn_open = QPushButton("查看")
            btn_open.setFixedSize(50, 26)
            btn_open.setStyleSheet("""
                QPushButton {
                    background: #1a5c2a; color: #fff;
                    border: 1px solid #2a7a3a; border-radius: 3px;
                }
                QPushButton:hover { background: #227a35; }
            """)
            btn_open.clicked.connect(
                lambda checked=False, p=path: QDesktopServices.openUrl(
                    QUrl.fromLocalFile(p)))
            self.table.setCellWidget(row, self.COL_OPEN, btn_open)
        else:
            self.table.setItem(row, self.COL_OPEN,
                               QTableWidgetItem("(已删除)"))

    def on_cell_clicked(self, row, col):
        if col == self.COL_PATH:
            path = self.table.item(row, col).text()
            if path and Path(path).exists():
                QDesktopServices.openUrl(QUrl.fromLocalFile(path))

    def open_snap_dir(self):
        for r in reversed(self.records):
            p = r.get('screenshot', '')
            if p and Path(p).exists():
                QDesktopServices.openUrl(
                    QUrl.fromLocalFile(str(Path(p).parent)))
                return
        QMessageBox.information(self, "提示", "没有找到截图文件")

    def clear_history(self):
        ret = QMessageBox.question(
            self, "确认", "确定要清空所有历史报警记录吗？",
            QMessageBox.Yes | QMessageBox.No)
        if ret == QMessageBox.Yes:
            save_history([])
            self.records = []
            self.table.setRowCount(0)


# ================================================================
#  传感器波形图组件
# ================================================================
class SensorChartWidget(QWidget):
    """实时传感器波形图: TVOC(绿) / CH2O(蓝) / CO2(黄)"""

    MAX_POINTS = 300       # 显示最近 300 个采样点
    SAMPLE_MS  = 200       # 采样间隔 (毫秒), 降频避免点数浪费
    SMOOTH = 0.25          # EMA 平滑系数 (越小越平滑)
    COLORS = {
        'tvoc': QColor(0, 200, 0),    # 绿
        'ch2o': QColor(0, 150, 255),  # 蓝
        'co2': QColor(255, 180, 0),   # 黄
    }
    RANGES = {
        'tvoc': (0, 2.0),   # mg/m³
        'ch2o': (0, 0.3),   # mg/m³
        'co2':  (0, 2000),  # PPM
    }

    def __init__(self):
        super().__init__()
        self.setMinimumHeight(130)
        self.data = {'tvoc': deque(maxlen=self.MAX_POINTS),
                     'ch2o': deque(maxlen=self.MAX_POINTS),
                     'co2': deque(maxlen=self.MAX_POINTS)}
        self._ema = {'tvoc': 0.0, 'ch2o': 0.0, 'co2': 0.0}
        self._last_sample_ts = 0  # 上次采样时间戳 (ms)

    def add_sample(self, tvoc, ch2o, co2):
        # 降频: 每 SAMPLE_MS 毫秒采样一次 (300点 × 200ms = 60秒趋势)
        import time
        now_ms = time.time() * 1000
        if now_ms - self._last_sample_ts < self.SAMPLE_MS:
            return
        self._last_sample_ts = now_ms

        # EMA 滤波: smooth = α×new + (1-α)×old
        for key, val in [('tvoc', tvoc), ('ch2o', ch2o), ('co2', co2)]:
            self._ema[key] = self.SMOOTH * val + (1 - self.SMOOTH) * self._ema[key]
            self.data[key].append(self._ema[key])
        self.update()

    def paintEvent(self, event):
        if not self.data['tvoc']:
            return

        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QColor(30, 30, 30))

        # 三等分, 子图表间距
        margin_top = 5
        gap = 2
        sub_h = (h - margin_top - gap * 2) / 3
        for idx, key in enumerate(['tvoc', 'ch2o', 'co2']):
            color = self.COLORS[key]
            pts = self.data[key]
            if len(pts) < 2:
                continue

            # 独立动态范围, 上下留 10% padding
            vals = [v for v in pts if v > 0]
            lo = 0
            hi = max(vals) * 1.1 if vals else self.RANGES[key][1]
            hi = max(hi, self.RANGES[key][1] * 0.3)
            hi += (hi - lo) * 0.15  # 顶部 15% padding

            base_y = margin_top + idx * (sub_h + gap)
            poly = QPolygonF()
            for i, v in enumerate(pts):
                x = w * i / (self.MAX_POINTS - 1)
                ratio = (v - lo) / (hi - lo + 0.001)
                y = base_y + sub_h * (1.0 - ratio * 0.85)  # 留 15% 底部 padding
                y = max(base_y + 3, min(base_y + sub_h - 3, y))
                poly.append(QPointF(x, y))

            # 分隔线
            p.setPen(QPen(QColor(80, 80, 80), 1))
            sep_y = int(base_y + sub_h + 1)
            p.drawLine(0, sep_y, w, sep_y)

            # 网格线
            p.setPen(QPen(QColor(50, 50, 50), 1, Qt.DashLine))
            for gy in [base_y + sub_h * 0.5, base_y + sub_h]:
                p.drawLine(0, int(gy), w, int(gy))

            # 曲线
            if len(poly) > 1:
                p.setPen(QPen(color, 2))
                for i in range(1, poly.size()):
                    p.drawLine(poly[i - 1], poly[i])

            # 标签 + 数值
            p.setFont(QFont("Consolas", 9))
            p.fillRect(0, int(base_y), 50, 14, QColor(30, 30, 30, 200))
            p.setPen(color)
            p.drawText(3, int(base_y + 12), f'{key.upper()}')
            p.setPen(QColor(200, 200, 200))
            val_str = f'{pts[-1]:.3f}' if key != 'co2' else f'{pts[-1]:.0f}'
            p.drawText(3, int(base_y + sub_h - 3), val_str)

        p.end()

    def clear(self):
        for k in self.data:
            self.data[k].clear()
        self.update()


# ================================================================
#  RTSP 视频拉流线程 (独立线程, 不阻塞Qt事件循环)
# ================================================================
class VideoThread(QThread):
    frame_ready = Signal(object)       # 新帧到达 (numpy array)
    stream_status = Signal(bool, str)  # 连接状态 (connected, message)

    def __init__(self, rtsp_url, frame_buffer=None, parent=None):
        super().__init__(parent)
        self.url = rtsp_url
        self.buffer = frame_buffer  # FrameBuffer for DVR
        self.running = False

    def run(self):
        self.running = True
        cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
        if cap.isOpened():
            self.stream_status.emit(True, "RTSP: ● 已连接")
        else:
            self.stream_status.emit(False, "RTSP: ● 断开")
            self.running = False
            return

        while self.running:
            ret, frame = cap.read()
            if not ret:
                self.stream_status.emit(False, "RTSP: ● 断开")
                break
            # 喂入DVR帧缓存
            if self.buffer is not None:
                self.buffer.feed(frame, time.time())
            self.frame_ready.emit(frame)
            self.msleep(5)  # 释放CPU

        cap.release()
        self.stream_status.emit(False, "RTSP: ● 断开")

    def stop(self):
        self.running = False
        self.wait(2000)  # 等待线程退出


# ================================================================
#  主窗口
# ================================================================
class MonitorWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # 加载配置
        self.config = load_config()

        self.setWindowTitle("吸烟检测上位机")
        self.resize(1200, 720)
        self.setMinimumSize(900, 550)
        self.setFocusPolicy(Qt.StrongFocus)  # 确保接收键盘事件

        # ---- 状态变量 ----
        self.video_thread = None   # RTSP 拉流线程
        self.latest_frame = None   # 最新视频帧 (由视频线程更新)
        self.current_frame = None
        self.is_playing = True
        self.is_fullscreen = False
        self.fps_count = 0
        self.fps_timer = time.time()
        self.fps_text = "0"

        # AI 检测
        self.detection = {
            'has_target': 0,
            'x1': 0, 'y1': 0, 'x2': 0, 'y2': 0,
            'confidence': 0.0,
            'frame_w': 1920, 'frame_h': 1080,
        }
        self.tcp_msg_count = 0  # TCP 消息接收计数器
        # 传感器数据
        self.sensor = {
            'tvoc': 0.0, 'ch2o': 0.0, 'co2': 0, 'alarm': 0,
        }
        self.slave_sensors = {1: {}, 2: {}}
        self.recv_buffer = b''
        self.was_detected = False
        self.last_alarm_time = 0.0
        self.last_auto_snap_time = 0.0          # 截图冷却
        self.detect_smoking = deque()            # [(timestamp, has), ...] class=0
        self.detect_fire = deque()               # [(timestamp, has), ...] class=1

        # 多源融合
        self.fusion = FusionEngine()
        self.fusion_state = '正常'
        self.fusion_color = '#5f5'
        self.fusion_scores = {'smoke':0,'smoke_th1':6.5,'smoke_th2':7.5,
                               'fire':0,'fire_th1':7.0,'fire_th2':8.5}

        # DVR 帧缓存 (最近10分钟回放)
        self.dvr = FrameBuffer()
        self.dvr_playing = False  # DVR回放模式 (从缓存帧顺序播放)
        self.dvr_play_idx = 0     # DVR回放当前帧索引
        self.dvr_last_play_ts = 0 # 上次播放DVR帧的时间戳(实际时间)

        # UDP 音频推流
        self.audio_sock = None
        self.audio_stream = None
        self.audio_input = None
        self.talking = False

        # 运动尾迹
        self.trails = deque(maxlen=TRAIL_LENGTH)

        # TCP 状态
        self.tcp_connected = False
        self.rtsp_connected = False

        # 日志持久化
        self.log_dir = Path(self.config.get('log_dir', str(Path.home() / 'smoking_logs')))
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.log_file = self.log_dir / time.strftime("alarm_%Y%m%d.log")
        self.log_lines = []

        # ---- TCP Socket ----
        self.ai_socket = QTcpSocket()
        self.ai_socket.connected.connect(self.on_ai_connected)
        self.ai_socket.disconnected.connect(self.on_ai_disconnected)
        self.ai_socket.readyRead.connect(self.on_ai_ready_read)
        self.ai_socket.errorOccurred.connect(self.on_ai_error)

        # ---- UI ----
        self.init_ui()
        self.apply_dark_theme()

        # ---- 定时器 ----
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

        # 连接 AI 服务器
        self.connect_to_ai_server()

        # 全局按键监听 (回车喊话)
        QApplication.instance().installEventFilter(self)

    # ================================================================
    #  主题
    # ================================================================
    def apply_dark_theme(self):
        self.setStyleSheet("""
            QMainWindow {
                background-color: #0a0e14;
                border: 1px solid #1a2332;
            }
            QWidget {
                color: #b7c9e2;
                font-size: 13px;
                font-family: \"Segoe UI\", \"PingFang SC\", \"Microsoft YaHei\", sans-serif;
            }

            /* ── Buttons ── */
            QPushButton {
                background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #1c2535, stop:1 #141c28);
                color: #b7c9e2; border: 1px solid #2a3a55; border-radius: 4px;
                padding: 7px 18px; font-size: 13px; font-weight: 500;
                text-transform: uppercase; letter-spacing: 0.5px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #243045, stop:1 #1a2332);
                border-color: #4a9eff; color: #e8f0fe;
            }
            QPushButton:pressed { background: #101720; border-color: #2a5a99; }
            QPushButton#btn_snapshot {
                background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #0f2818, stop:1 #0a1c10);
                border-color: #1f4a2d; color: #5cdb7c;
            }
            QPushButton#btn_snapshot:hover { background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #163a22, stop:1 #0f2818); border-color: #3cc96a; }
            QPushButton#btn_talk {
                background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #2d1018, stop:1 #1f0a10);
                border-color: #4d1f2d; color: #f87171;
            }
            QPushButton#btn_talk:hover { background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #3d1520, stop:1 #2d1018); border-color: #e05555; }
            QPushButton#btn_live {
                background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #101e30, stop:1 #0a1520);
                border-color: #1f3d55; color: #5ca8f0;
            }
            QPushButton#btn_live:hover { background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #152a40, stop:1 #101e30); border-color: #3d88e0; }

            /* ── Text Log ── */
            QTextEdit {
                background-color: #070b10; color: #8899bb;
                border: 1px solid #1a2a3d; border-radius: 4px;
                font-size: 11px; padding: 8px;
                font-family: \"Consolas\", \"Courier New\", monospace;
                selection-background-color: #1a3a5c;
            }

            /* ── Status Bar ── */
            QStatusBar {
                background-color: #0c121c; color: #6a89b0;
                border-top: 1px solid #1a2a3d; font-size: 11px;
                padding: 2px 8px;
            }

            /* ── Group Box ── */
            QGroupBox {
                border: 1px solid #1f3148; border-radius: 6px;
                margin-top: 16px; padding: 22px 10px 10px 10px;
                color: #a0c4f0; font-weight: 600; font-size: 12px;
                background-color: rgba(15,22,36,0.6);
            }
            QGroupBox::title {
                subcontrol-origin: margin; left: 14px; padding: 0 10px;
                color: #5ca8f0; letter-spacing: 1px;
            }

            /* ── Slider ── */
            QSlider::groove:horizontal {
                background: #141f30; border-radius: 3px; height: 5px;
                border: 1px solid #1f3148;
            }
            QSlider::handle:horizontal {
                background: qradialgradient(cx:0.5,cy:0.5,radius:0.5, stop:0 #8cc8ff, stop:1 #3a78d8);
                width: 14px; height: 14px; margin: -5px 0; border-radius: 7px;
                border: 1px solid #5c9ce0;
            }
            QSlider::sub-page:horizontal { background: #2a5a99; border-radius: 3px; }

            /* ── Line Edit ── */
            QLineEdit {
                background: #0d1520; color: #c0d0e8;
                border: 1px solid #2a3d58; border-radius: 4px;
                padding: 8px 12px; font-size: 14px;
            }
            QLineEdit:focus { border-color: #4a9eff; background: #0f1826; }

            /* ── Tables ── */
            QTableWidget {
                background: #0d1520; color: #8899bb; alternate-background-color: #101a28;
                border: 1px solid #1a2a3d; border-radius: 4px;
                gridline-color: #141f30; font-size: 12px;
            }
            QTableWidget::item { padding: 4px 8px; }
            QTableWidget::item:selected { background: #1e3d66; color: #e0e8f8; }
            QHeaderView::section {
                background: #121d2e; color: #a0c4f0;
                padding: 6px 8px; border: none; font-weight: 600; font-size: 11px;
                text-transform: uppercase; letter-spacing: 0.5px;
            }

            /* ── Scrollbar ── */
            QScrollBar:vertical {
                background: #0a0e14; width: 8px; border-radius: 4px;
            }
            QScrollBar::handle:vertical {
                background: #2a3d58; min-height: 30px; border-radius: 4px;
            }
            QScrollBar::handle:vertical:hover { background: #3a5580; }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
        """)

    # ================================================================
    #  UI 初始化
    # ================================================================
    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)

        # -- 视频显示区 (SCADA-style) --
        self.video_label = QLabel("● SYSTEM INITIALIZING ●\n\nAwaiting video stream...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(760, 500)
        self.video_label.setStyleSheet("""
            QLabel {
                background-color: #05080d;
                color: #1a3050;
                border: 2px solid #1a2a3d;
                border-radius: 4px;
                font-size: 18px; font-weight: 300;
                letter-spacing: 2px;
            }
        """)
        self.video_label.setMouseTracking(True)

        # -- SCADA 状态卡片 --
        card = "background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #111c2a, stop:1 #0d1520); border: 1px solid #1f3148; border-radius: 3px; padding: 5px 12px; font-size: 11px; font-weight: 500; letter-spacing: 0.5px;"
        self.lbl_fps = QLabel("FPS  --"); self.lbl_fps.setStyleSheet(card + "color: #5ca8f0;")
        self.lbl_fusion = QLabel("STATUS  NORMAL"); self.lbl_fusion.setStyleSheet(card + "color: #5cdb7c; font-weight: 700; font-size: 12px;")
        self.lbl_sensor = QLabel("TVOC -- | CH2O -- | CO2 -- | TEMP --°C"); self.lbl_sensor.setStyleSheet(card + "color: #6a89b0;")
        self.lbl_rtsp_status = QLabel("RTSP  OFFLINE"); self.lbl_rtsp_status.setStyleSheet(card + "color: #f05555;")
        self.lbl_ai_status = QLabel("AI  OFFLINE"); self.lbl_ai_status.setStyleSheet(card + "color: #f05555;")

        info_layout = QHBoxLayout()
        info_layout.addWidget(self.lbl_fps)
        info_layout.addWidget(self.lbl_fusion)
        info_layout.addWidget(self.lbl_sensor)
        info_layout.addStretch()
        info_layout.addWidget(self.lbl_rtsp_status)
        info_layout.addWidget(self.lbl_ai_status)

        # -- 日志区 --
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setPlaceholderText("报警日志将在这里显示...")
        log_group = QGroupBox("报警 / 运行日志")
        log_layout = QVBoxLayout()
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)

        # -- 按钮 --
        self.btn_play = QPushButton("▶ 播放")
        self.btn_pause = QPushButton("⏸ 暂停")
        self.btn_live = QPushButton("🔴 实时")
        self.btn_snapshot = QPushButton("📷 截图")
        self.btn_settings = QPushButton("⚙ 设置")
        self.btn_history = QPushButton("📋 历史记录")
        self.btn_clear = QPushButton("清空日志")
        self.btn_talk = QPushButton("🎤 按住说话")
        self.btn_talk.setMinimumWidth(120)
        self.btn_reboot = QPushButton("🔄 重启")
        self.btn_shutdown = QPushButton("⏻ 关机")

        self.btn_snapshot.setObjectName("btn_snapshot")
        self.btn_talk.setObjectName("btn_talk")
        self.btn_live.setObjectName("btn_live")

        for btn in [self.btn_play, self.btn_pause, self.btn_live,
                     self.btn_snapshot, self.btn_settings,
                     self.btn_history, self.btn_clear, self.btn_talk, self.btn_reboot, self.btn_shutdown]:
            btn.setFixedHeight(36)
            btn.setCursor(Qt.PointingHandCursor)

        self.btn_play.clicked.connect(self.play_video)
        self.btn_pause.clicked.connect(self.pause_video)
        self.btn_live.clicked.connect(self.go_live)
        self.btn_snapshot.clicked.connect(self.snapshot)
        self.btn_settings.clicked.connect(self.open_settings)
        self.btn_history.clicked.connect(self.open_history)
        self.btn_clear.clicked.connect(self.clear_log)
        self.btn_talk.pressed.connect(self.start_talking)
        self.btn_talk.released.connect(self.stop_talking)
        self.btn_reboot.clicked.connect(self.reboot_board)
        self.btn_shutdown.clicked.connect(self.shutdown_board)

        btn_layout = QHBoxLayout()
        btn_layout.addWidget(self.btn_play)
        btn_layout.addWidget(self.btn_pause)
        btn_layout.addWidget(self.btn_live)
        btn_layout.addWidget(self.btn_snapshot)
        btn_layout.addWidget(self.btn_settings)
        btn_layout.addWidget(self.btn_history)
        btn_layout.addWidget(self.btn_clear)
        btn_layout.addWidget(self.btn_talk)
        btn_layout.addWidget(self.btn_reboot)
        btn_layout.addWidget(self.btn_shutdown)
        btn_layout.addStretch()

        # -- 时间轴回放滑块 (暂停时显示) --
        self.dvr_slider = QSlider(Qt.Horizontal)
        self.dvr_slider.setMinimum(0); self.dvr_slider.setMaximum(0)
        self.dvr_slider.setVisible(False)
        self.dvr_slider.sliderPressed.connect(self._on_dvr_slider_press)
        self.dvr_slider.sliderReleased.connect(self._on_dvr_slider_release)
        self.dvr_slider.valueChanged.connect(self._on_dvr_slider_move)
        self.dvr_time_label = QLabel("")
        self.dvr_time_label.setStyleSheet("color: #888; font-size: 12px;")
        self.dvr_time_label.setVisible(False)
        dvr_slider_layout = QHBoxLayout()
        dvr_slider_layout.addWidget(self.dvr_time_label)
        dvr_slider_layout.addWidget(self.dvr_slider)

        # -- 传感器波形 --
        self.sensor_chart = SensorChartWidget()
        self.sensor_chart.setMaximumHeight(140)

        # -- 左侧: 视频 + 时间轴 + 波形 + 状态 --
        left_layout = QVBoxLayout()
        left_layout.addWidget(self.video_label, 5)       # 视频占 5 份
        left_layout.addLayout(dvr_slider_layout)
        left_layout.addWidget(self.sensor_chart, 1)      # 波形占 1 份
        left_layout.addLayout(info_layout)

        # -- 右侧: 日志 --
        right_layout = QVBoxLayout()
        right_layout.addWidget(log_group)

        # -- 主体 --
        main_layout = QHBoxLayout()
        main_layout.addLayout(left_layout, 3)
        main_layout.addLayout(right_layout, 1)

        root_layout = QVBoxLayout()
        root_layout.addLayout(main_layout)
        root_layout.addLayout(btn_layout)

        central.setLayout(root_layout)

        # -- 状态栏 --
        self.status = self.statusBar()
        self.status.showMessage("就绪")

        # -- 快捷键 --
        space = QAction("暂停/播放", self)
        space.setShortcut(QKeySequence(Qt.Key_Space))
        space.triggered.connect(self.toggle_play)
        self.addAction(space)

    def eventFilter(self, obj, event):
        """全局按键过滤器 — 回车键联动喊话"""
        from PySide6.QtCore import QEvent
        if event.type() == QEvent.KeyPress:
            if event.key() in (Qt.Key_Return, 16777220) and not event.isAutoRepeat():
                self.start_talking()
                return True
        elif event.type() == QEvent.KeyRelease:
            if event.key() in (Qt.Key_Return, 16777220) and not event.isAutoRepeat():
                self.stop_talking()
                return True
        return super().eventFilter(obj, event)

    # ================================================================
    #  RTSP 视频 (独立线程, 不阻塞UI)
    # ================================================================
    def init_video(self):
        """启动 RTSP 拉流线程"""
        host = self.config.get('host', '192.168.1.10')
        rtsp_port = self.config.get('rtsp_port', 8554)
        url = f'rtsp://{host}:{rtsp_port}/live'
        self.add_log(f"正在连接视频流: {url}")
        self.video_thread = VideoThread(url, self.dvr)
        self.video_thread.frame_ready.connect(self.on_frame_ready)
        self.video_thread.stream_status.connect(self.on_stream_status)
        self.video_thread.start()

    def on_frame_ready(self, frame):
        """接收视频线程传来的帧 (Qt信号, 主线程执行)"""
        if self.is_playing:
            self.latest_frame = frame

    def on_stream_status(self, connected, status_text):
        """接收视频线程传来的连接状态"""
        self.rtsp_connected = connected
        self.lbl_rtsp_status.setText(status_text)
        if connected:
            self.lbl_rtsp_status.setStyleSheet("color: #5f5; font-size: 13px;")
            self.add_log("视频流已连接 ✓")
        else:
            self.lbl_rtsp_status.setStyleSheet("color: #f55; font-size: 13px;")
            self.add_log("视频流断开，3 秒后重试...")
            QTimer.singleShot(3000, self.init_video)

    # ================================================================
    #  TCP 坐标
    # ================================================================
    def connect_to_ai_server(self):
        host = self.config.get('host', '192.168.1.10')
        port = self.config.get('tcp_port', 9999)
        self.add_log(f"正在连接 AI 服务器 {host}:{port} ...")
        self.ai_socket.connectToHost(host, port)

    def on_ai_connected(self):
        self.tcp_connected = True
        self.lbl_ai_status.setText("AI  ONLINE")
        self.lbl_ai_status.setStyleSheet("background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #111c2a, stop:1 #0d1520); border: 1px solid #1f3148; border-radius: 3px; padding: 5px 12px; font-size: 11px; font-weight: 500; letter-spacing: 0.5px; color: #5cdb7c;")
        self.add_log("AI 坐标服务器已连接 ✓")
        self.status.showMessage("AI 坐标已连接")

    def on_ai_disconnected(self):
        self.tcp_connected = False
        self.lbl_ai_status.setText("AI  OFFLINE")
        self.lbl_ai_status.setStyleSheet("background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #111c2a, stop:1 #0d1520); border: 1px solid #1f3148; border-radius: 3px; padding: 5px 12px; font-size: 11px; font-weight: 500; letter-spacing: 0.5px; color: #f05555;")
        self.add_log("AI 坐标服务器已断开，5 秒后重连...")
        self.status.showMessage("AI 坐标已断开")
        QTimer.singleShot(5000, self.connect_to_ai_server)

    def on_ai_error(self, error):
        self.tcp_connected = False
        self.lbl_ai_status.setText("AI  ERROR")
        self.lbl_ai_status.setStyleSheet("background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #111c2a, stop:1 #0d1520); border: 1px solid #1f3148; border-radius: 3px; padding: 5px 12px; font-size: 11px; font-weight: 500; letter-spacing: 0.5px; color: #f05555;")
        self.add_log(f"AI 坐标连接错误: {error}")

    def on_ai_ready_read(self):
        self.recv_buffer += bytes(self.ai_socket.readAll())
        while len(self.recv_buffer) >= MSG_SIZE:
            msg = self.recv_buffer[:MSG_SIZE]
            self.recv_buffer = self.recv_buffer[MSG_SIZE:]
            try:
                self.parse_detection(msg)
            except Exception as e:
                print(f"[TCP PARSE ERROR] {e}", flush=True)

    def parse_detection(self, data):
        (slave_id, cx, cy, w, h, frame_w, frame_h, confidence, has_target, class_id,
         tvoc, ch2o, co2, sensor_alarm, temperature,
         smoke_score, fire_score) = struct.unpack(MSG_FMT, data)

        # 根据 slave_id 分流处理
        if slave_id >= 1:
            # 从机传感器数据 → 存到独立字典
            self.slave_sensors[slave_id] = {
                'tvoc': tvoc, 'ch2o': ch2o, 'co2': co2, 'temp': temperature,
            }
            return  # 传感器消息不触发 AI 逻辑

        # slave_id=0: AI 检测数据
        self.sensor = {
            'tvoc': tvoc, 'ch2o': ch2o, 'co2': co2, 'alarm': sensor_alarm,
            'temperature': temperature,
        }
        self.sensor_chart.add_sample(tvoc, ch2o, co2)

        # 多源融合判定 (评分由板端计算)
        self.fusion_state, self.fusion_color, self.fusion_scores, fusion_changed = \
            self.fusion.update_from_board(has_target, class_id, smoke_score, fire_score, sensor_alarm)

        # 状态变化 → 记录日志
        if fusion_changed:
            if self.fusion_state == '火灾报警':
                self.add_alarm(f"🔥 火灾报警! (烟:{self.fusion_scores['smoke']} 火:{self.fusion_scores['fire']})")
            elif self.fusion_state == '疑似火灾':
                self.add_log(f"⚠ 疑似火灾 (火:{self.fusion_scores['fire']})")
            elif self.fusion_state == '吸烟报警':
                self.add_alarm(f"🚬 吸烟报警! (烟:{self.fusion_scores['smoke']})")
            elif self.fusion_state == '疑似吸烟':
                self.add_log(f"🔍 疑似吸烟 (烟:{self.fusion_scores['smoke']})")
            elif self.fusion_state == '正常':
                self.add_log("系统恢复正常")

        # 实时更新状态标签 (独立于视频帧循环, TCP数据到达即刷新)
        self.tcp_msg_count += 1
        self.lbl_fusion.setText(f"判定: {self.fusion_state}")
        self.lbl_fusion.setStyleSheet(
            f"color: {self.fusion_color}; font-size: 14px; font-weight: bold;")
        sc = self.fusion_scores
        s = self.sensor
        self.lbl_sensor.setText(
            f"烟:{sc['smoke']:.1f}/{sc['smoke_th1']:.1f}/{sc['smoke_th2']:.1f} | "
            f"火:{sc['fire']:.1f}/{sc['fire_th1']:.1f}/{sc['fire_th2']:.1f} | "
            f"TVOC:{s['tvoc']:.3f} CH2O:{s['ch2o']:.3f} CO2:{s['co2']} TEMP:{s['temperature']:.1f}°C")
        self.status.showMessage(f"TCP: {self.tcp_msg_count} msgs | 判定: {self.fusion_state}")

        # 自动截图: 报警状态持续时每5s截一次
        now = time.time()
        if self.fusion_state in ('吸烟报警', '火灾报警'):
            if now - self.last_auto_snap_time > 5:
                snap_type = 'smoking' if self.fusion_state == '吸烟报警' else 'fire'
                self.auto_snapshot(snap_type)
                self.last_auto_snap_time = now

        # 记录到各类别滑动窗口
        now = time.time()
        cutoff = now - AUTO_SNAP_WINDOW
        # smoking (class 1)
        is_smoking = has_target and class_id == 1
        self.detect_smoking.append((now, is_smoking))
        while self.detect_smoking and self.detect_smoking[0][0] < cutoff:
            self.detect_smoking.popleft()
        # fire (class 2)
        is_fire = has_target and class_id == 2
        self.detect_fire.append((now, is_fire))
        while self.detect_fire and self.detect_fire[0][0] < cutoff:
            self.detect_fire.popleft()

        if has_target:
            x1 = max(0, int(cx - w / 2))
            y1 = max(0, int(cy - h / 2))
            x2 = min(frame_w, int(cx + w / 2))
            y2 = min(frame_h, int(cy + h / 2))

            label = CLASS_NAMES.get(class_id, 'unknown')
            self.detection = {
                'has_target': 1,
                'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                'confidence': confidence,
                'class_id': class_id,
                'label': label,
                'frame_w': frame_w, 'frame_h': frame_h,
            }

            # 运动尾迹
            self.trails.append((cx, cy, w, h, confidence, class_id))

            self.was_detected = True
        else:
            self.was_detected = False
            self.trails.clear()
            self.detection = {
                'has_target': 0,
                'x1': 0, 'y1': 0, 'x2': 0, 'y2': 0,
                'confidence': 0.0,
                'frame_w': frame_w, 'frame_h': frame_h,
            }

        # (自动截图已集成到多源融合判定中)

    # ================================================================
    #  视频帧更新 + 画框
    # ================================================================
    def update_frame(self):
        if not self.is_playing and not self.dvr_slider.isVisible():
            return  # 不是播放也不是DVR回放模式 → 不刷新

        # DVR回放模式: 按原始时间戳间隔播放 (2fps采样→500ms/帧)
        if self.dvr_playing:
            if self.dvr_play_idx >= self.dvr.frame_count():
                self.dvr_playing = False  # 已追上, 切回直播
                self.status.showMessage("播放中")
            elif time.time() - self.dvr_last_play_ts >= 0.167:  # 167ms ≈ 6fps
                jpg = self.dvr.frames[self.dvr_play_idx][1]
                frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                self.dvr_play_idx += 1
                self.dvr_last_play_ts = time.time()
                if frame is None:
                    return
                self.current_frame = frame.copy()
                pixmap = self._render_frame(frame)
                self._update_info_labels()
                lw = self.video_label.width() or self.width()
                lh = self.video_label.height() or self.height()
                pixmap = pixmap.scaled(lw, lh, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                self.video_label.setPixmap(pixmap)
            return

        frame = self.latest_frame
        if frame is None:
            return

        # FPS (仅在播放时计数)
        if self.is_playing:
            self.fps_count += 1
        now = time.time()
        if now - self.fps_timer >= 1.0:
            self.fps_text = str(self.fps_count)
            self.lbl_fps.setText(f"FPS: {self.fps_text}")
            self.fps_count = 0
            self.fps_timer = now

        self.current_frame = frame.copy()

        # BGR → RGB → QPixmap + 画叠加层
        pixmap = self._render_frame(frame)

        # 更新标签
        self._update_info_labels()

        # 缩放显示
        lw = self.video_label.width() or self.width()
        lh = self.video_label.height() or self.height()
        pixmap = pixmap.scaled(lw, lh, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.video_label.setPixmap(pixmap)

    def _render_frame(self, frame):
        """BGR numpy array → QPixmap (带标注框/FPS/传感器叠加)"""
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = frame_rgb.shape
        image = QImage(frame_rgb.data, w, h, ch * w, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(image)

        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing)
        is_dvr = self.dvr_playing or self.dvr_slider.isVisible()  # 回放/暂停模式不画框

        if self.detection['has_target'] and not is_dvr:
            x1, y1 = self.detection['x1'], self.detection['y1']
            x2, y2 = self.detection['x2'], self.detection['y2']
            cid = self.detection.get('class_id', 0)
            r, g, b = CLASS_COLORS.get(cid, (0, 255, 0))
            pen = QPen(QColor(r, g, b)); pen.setWidth(4)
            painter.setPen(pen)
            painter.drawRect(x1, y1, x2 - x1, y2 - y1)
            font = QFont("Arial", 32, QFont.Bold); painter.setFont(font)
            label = f"{self.detection['label']} {self.detection['confidence']:.2f}"
            fm = painter.fontMetrics()
            text_rect = fm.boundingRect(label)
            bg_rect = text_rect.translated(x1 + 2, y1 - text_rect.height() - 6)
            bg_rect.setWidth(bg_rect.width() + 8)
            if bg_rect.top() < 0: bg_rect.moveTop(y1 + 4)
            painter.fillRect(bg_rect, QColor(r, g, b, 200))
            painter.setPen(QColor(255, 255, 255))
            painter.drawText(bg_rect, Qt.AlignCenter, label)

        for i, item in enumerate(self.trails):
            if is_dvr: break  # 回放/暂停模式不画尾迹
            if len(item) == 6: cx, cy, tw, th, conf, cid = item
            else: cx, cy, tw, th, conf = item; cid = 0
            alpha = 60 + int(100 * i / max(len(self.trails), 1))
            tx1, ty1 = max(0, cx - tw // 2), max(0, cy - th // 2)
            tx2, ty2 = max(0, cx + tw // 2), max(0, cy + th // 2)
            r2, g2, b2 = CLASS_COLORS.get(cid, (0, 255, 0))
            pen = QPen(QColor(r2, g2, b2, alpha)); pen.setWidth(1)
            painter.setPen(pen)
            painter.drawRect(tx1, ty1, tx2 - tx1, ty2 - ty1)

        if self.fps_text != "0":
            fps_font = QFont("Consolas", 18, QFont.Bold); painter.setFont(fps_font)
            painter.setPen(QColor(0, 255, 0))
            painter.drawText(12, 30, f"FPS: {self.fps_text}")

        painter.end()
        return pixmap

    def _update_info_labels(self):
        """更新顶部状态标签"""
        self.lbl_fusion.setText(f"判定: {self.fusion_state}")
        self.lbl_fusion.setStyleSheet(f"color: {self.fusion_color}; font-size: 14px; font-weight: bold;")
        sc = self.fusion_scores; s = self.sensor; ss = getattr(self, 'slave_sensors', {})
        s1 = ss.get(1, {}); s2 = ss.get(2, {})
        self.lbl_sensor.setText(
            f"烟:{sc['smoke']:.1f}/{sc['smoke_th1']:.1f}/{sc['smoke_th2']:.1f} | "
            f"火:{sc['fire']:.1f}/{sc['fire_th1']:.1f}/{sc['fire_th2']:.1f} | "
            f"S1:TVOC{s1.get('tvoc',0):.3f} CO2{s1.get('co2',0)} {s1.get('temp',0):.1f}°C | "
            f"S2:TVOC{s2.get('tvoc',0):.3f} CO2{s2.get('co2',0)} {s2.get('temp',0):.1f}°C")

    # ================================================================
    #  按钮操作
    # ================================================================
    def play_video(self):
        # 如果时间轴可见(曾暂停/拖动过), 从DVR缓存帧开始回放
        if self.dvr_slider.isVisible() and self.dvr.frame_count() > 0:
            self.dvr_play_idx = self.dvr_slider.value()
            self.dvr_playing = True
            self.dvr_last_play_ts = 0  # 首帧立即播放
        self.is_playing = True
        self.dvr_slider.setVisible(False)
        self.dvr_time_label.setVisible(False)
        self.status.showMessage("播放中" if not self.dvr_playing else "DVR回放中...")

    def pause_video(self):
        was_dvr = self.dvr_playing
        self.is_playing = False
        self.dvr_playing = False
        # 显示时间轴滑块
        if self.dvr.frame_count() > 1:
            self.dvr_slider.setMaximum(self.dvr.frame_count() - 1)
            # DVR回放中暂停 → 停在当前播放位置; 直播暂停 → 停在最新帧
            pos = self.dvr_play_idx if was_dvr else self.dvr.frame_count() - 1
            self.dvr_slider.setValue(pos)
            # 显示当前帧
            if 0 <= pos < self.dvr.frame_count():
                jpg = self.dvr.frames[pos][1]
                img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    self.latest_frame = img
                    self.current_frame = img.copy()
            self.dvr_slider.setVisible(True)
            self.dvr_time_label.setVisible(True)
            self._update_dvr_label(pos)
        self.status.showMessage("已暂停")

    # ---- DVR 时间轴回调 ----
    def _on_dvr_slider_press(self):
        pass  # 开始拖拽

    def _on_dvr_slider_move(self, val):
        self._update_dvr_label(val)

    def _on_dvr_slider_release(self):
        val = self.dvr_slider.value()
        if 0 <= val < self.dvr.frame_count():
            jpg = self.dvr.frames[val][1]
            img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                self.latest_frame = img
                self.current_frame = img.copy()
        self._update_dvr_label(val)

    def _update_dvr_label(self, val=None):
        if val is None: val = self.dvr_slider.value()
        if self.dvr.frame_count() > val:
            ts = self.dvr.frames[val][0]
            tstr = time.strftime("%H:%M:%S", time.localtime(ts))
            latest_ts = self.dvr.frames[-1][0]
            offset = int(latest_ts - ts)
            self.dvr_time_label.setText(f"  ⏪ {-offset}s  [{tstr}]")

    def toggle_play(self):
        if self.is_playing:
            self.pause_video()
        else:
            self.play_video()

    def go_live(self):
        """跳到最新实时画面"""
        self.dvr_playing = False
        self.is_playing = True
        self.dvr_slider.setVisible(False)
        self.dvr_time_label.setVisible(False)
        self.status.showMessage("实时 — 已跳到最新画面")

    def snapshot(self):
        if self.current_frame is None:
            QMessageBox.warning(self, "提示", "当前没有可截图的画面")
            return

        save_dir = Path(self.config.get('snap_dir',
                        str(Path.home() / 'Pictures' / 'smoking_snapshots')))
        save_dir.mkdir(parents=True, exist_ok=True)

        filename = time.strftime("snapshot_%Y%m%d_%H%M%S.jpg")
        save_path = save_dir / filename

        frame_with_box = self.current_frame.copy()
        if self.detection['has_target']:
            x1, y1 = self.detection['x1'], self.detection['y1']
            x2, y2 = self.detection['x2'], self.detection['y2']
            cid = self.detection.get('class_id', 0)
            r, g, b = CLASS_COLORS.get(cid, (0, 255, 0))
            cv2.rectangle(frame_with_box, (x1, y1), (x2, y2), (r, g, b), 3)
            label = f"{self.detection['label']} {self.detection['confidence']:.2f}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)
            ty = y1 - 8 if y1 - 8 > th else y1 + th + 8
            cv2.rectangle(frame_with_box, (x1, ty - th - 4),
                          (x1 + tw + 4, ty + 4), (r, g, b), -1)
            cv2.putText(frame_with_box, label, (x1 + 2, ty),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 2)

        cv2.imwrite(str(save_path), frame_with_box)
        self.add_log(f"📷 截图已保存: {save_path}")

    def auto_snapshot(self, snap_type='smoking'):
        """自动截图: 5s 窗口内检测帧 >=80% 触发"""
        if self.current_frame is None:
            return
        save_dir = Path(self.config.get('snap_dir',
                        str(Path.home() / 'Pictures' / 'smoking_snapshots')))
        save_dir.mkdir(parents=True, exist_ok=True)
        filename = time.strftime(f"{snap_type}_%Y%m%d_%H%M%S.jpg")
        save_path = save_dir / filename

        frame = self.current_frame.copy()
        if self.detection['has_target']:
            x1, y1 = self.detection['x1'], self.detection['y1']
            x2, y2 = self.detection['x2'], self.detection['y2']
            cid = self.detection.get('class_id', 0)
            r, g, b = CLASS_COLORS.get(cid, (0, 255, 0))
            cv2.rectangle(frame, (x1, y1), (x2, y2), (r, g, b), 3)
            label = f"{self.detection['label']} {self.detection['confidence']:.2f}"
            cv2.putText(frame, label, (x1, y1-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (r, g, b), 2)

        cv2.imwrite(str(save_path), frame)

        confidence = self.detection.get('confidence', 0.0)
        add_history_record(save_path, confidence, AUTO_SNAP_WINDOW)
        self.add_alarm(f"📷 自动截图 [{snap_type}] (置信度: {confidence:.2f})")

    def start_talking(self):
        """按住说话: arecord 采集麦克风 → UDP 推流到开发板"""
        if self.talking:
            return

        try:
            self.audio_sock = sock_mod.socket(sock_mod.AF_INET, sock_mod.SOCK_DGRAM)

            # arecord → raw PCM → stdout → Python 读取 → UDP 发送
            self.audio_proc = QProcess()
            self.audio_proc.setProcessChannelMode(QProcess.SeparateChannels)
            self.audio_proc.readyReadStandardOutput.connect(self._send_audio)
            self.audio_proc.start('arecord', [
                '-q', '-f', 'S16_LE', '-r', '16000', '-c', '1', '-t', 'raw'
            ])
            self.talking = True
            self.btn_talk.setText("🎤 正在说话...")
            self.btn_talk.setStyleSheet("background-color: #c0392b; color: white;")
            self.add_log("🎤 开始喊话")
        except Exception as e:
            self.add_log(f"麦克风启动失败: {e}")

    def _send_audio(self):
        """读取 arecord 的 PCM 数据并通过 UDP 发送"""
        if self.audio_proc and self.audio_sock:
            data = self.audio_proc.readAllStandardOutput()
            if data:
                try:
                    self.audio_sock.sendto(bytes(data), (self.config['host'], 9998))
                except Exception:
                    pass

    def stop_talking(self):
        """松开按钮: 停止麦克风"""
        self.talking = False
        self.btn_talk.setText("🎤 按住说话")
        self.btn_talk.setStyleSheet("")
        if self.audio_proc:
            self.audio_proc.terminate()
            self.audio_proc = None
        if self.audio_sock:
            self.audio_sock.close()
            self.audio_sock = None
        self.add_log("🎤 喊话结束")

    def reboot_board(self):
        ret = QMessageBox.question(self, "确认", "确定要远程重启开发板吗？\n重启后约40秒恢复。")
        if ret == QMessageBox.Yes:
            try:
                s = sock_mod.socket(sock_mod.AF_INET, sock_mod.SOCK_DGRAM)
                s.sendto(b"REBOOT\n", (self.config['host'], 9997))
                s.close()
                self.add_log("🔄 已发送重启指令到开发板")
            except Exception as e:
                self.add_log(f"重启指令发送失败: {e}")

    def shutdown_board(self):
        ret = QMessageBox.question(self, "确认", "确定要远程关闭开发板吗？\n关机后需手动上电。")
        if ret == QMessageBox.Yes:
            try:
                s = sock_mod.socket(sock_mod.AF_INET, sock_mod.SOCK_DGRAM)
                s.sendto(b"SHUTDOWN\n", (self.config['host'], 9997))
                s.close()
                self.add_log("⏻ 已发送关机指令到开发板")
            except Exception as e:
                self.add_log(f"关机指令发送失败: {e}")

    def clear_log(self):
        self.log_text.clear()
        self.log_lines.clear()

    def open_settings(self):
        dlg = SettingsDialog(self.config, self)
        if dlg.exec():
            self.config = dlg.get_config()
            save_config(self.config)
            self.add_log("设置已保存 ✓")
            self.status.showMessage("设置已保存，部分修改需重启生效")

    def open_history(self):
        dlg = HistoryDialog(self)
        dlg.exec()

    # ================================================================
    #  全屏
    # ================================================================
    def toggle_fullscreen(self, event=None):
        if self.is_fullscreen:
            self.exit_fullscreen()
        else:
            self.is_fullscreen = True
            self.showFullScreen()
            self.sensor_chart.hide()
            self.log_text.hide()
            for g in self.findChildren(QGroupBox):
                g.hide()
            self.btn_play.hide(); self.btn_pause.hide(); self.btn_live.hide()
            self.btn_snapshot.hide(); self.btn_settings.hide()
            self.btn_history.hide(); self.btn_clear.hide(); self.btn_talk.hide()
            self.lbl_fps.hide(); self.lbl_fusion.hide(); self.lbl_sensor.hide()
            self.lbl_rtsp_status.hide(); self.lbl_ai_status.hide()
            self.statusBar().hide()
            self.video_label.setAlignment(Qt.AlignCenter)
            self.video_label.setStyleSheet("background-color: black;")

    def exit_fullscreen(self):
        if self.is_fullscreen:
            self.is_fullscreen = False
            self.showNormal()
            self.video_label.setAlignment(Qt.AlignCenter)
            self.video_label.setStyleSheet("""
                QLabel {
                    background-color: #000; color: #555;
                    border: 2px solid #333; font-size: 22px;
                }
            """)
            self.sensor_chart.show()
            self.log_text.show()
            for g in self.findChildren(QGroupBox):
                g.show()
            for btn in [self.btn_play, self.btn_pause, self.btn_live,
                         self.btn_snapshot, self.btn_settings,
                         self.btn_history, self.btn_clear, self.btn_talk]:
                btn.show()
            self.lbl_fps.show(); self.lbl_fusion.show(); self.lbl_sensor.show()
            self.lbl_rtsp_status.show(); self.lbl_ai_status.show()
            self.statusBar().show()

    # ================================================================
    #  日志
    # ================================================================
    def add_log(self, text):
        now = time.strftime("%H:%M:%S")
        line = f"[{now}] {text}"
        self.log_text.append(line)
        self.log_lines.append(line)
        self._write_log(line)

    def add_alarm(self, text):
        now = time.strftime("%H:%M:%S")
        line = f"[{now}] 报警：{text}"
        self.log_text.append(
            f'<span style="color:red; font-weight:bold;">{line}</span>')
        self.log_lines.append(line)
        self._write_log(line)
        self.status.showMessage(text)

    def _write_log(self, line):
        try:
            log_file = self.log_dir / time.strftime("alarm_%Y%m%d.log")
            with open(log_file, 'a', encoding='utf-8') as f:
                f.write(line + '\n')
        except Exception:
            pass

    # ================================================================
    #  清理
    # ================================================================
    def closeEvent(self, event):
        if self.video_thread is not None:
            self.video_thread.stop()
        event.accept()


# ================================================================
#  登录对话框
# ================================================================
LOGIN_PASSWORD = "123456"  # 默认登录密码

class LoginDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("ELF-RV1126B 智能物联系统")
        self.setFixedSize(600, 500)
        self.setWindowFlags(Qt.Dialog | Qt.FramelessWindowHint)
        self.setStyleSheet("QDialog { background: #0a0a1a; }")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # 封面图
        cover_path = Path(__file__).parent / "login_cover.png"
        self.cover_label = QLabel()
        if cover_path.exists():
            pix = QPixmap(str(cover_path))
            pix = pix.scaled(600, 338, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.cover_label.setPixmap(pix)
            self.cover_label.setAlignment(Qt.AlignCenter)
        else:
            self.cover_label.setText("封面图未找到")
            self.cover_label.setStyleSheet("color: #555; font-size: 18px;")
            self.cover_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.cover_label)

        # 下半部分
        bottom = QWidget()
        bottom.setStyleSheet("background: #0d0d20;")
        bl = QVBoxLayout(bottom)
        bl.setContentsMargins(60, 25, 60, 25)
        bl.setSpacing(12)

        title = QLabel("智能物联系统")
        title.setStyleSheet("color: #fff; font-size: 20px; font-weight: bold;")
        title.setAlignment(Qt.AlignCenter)
        bl.addWidget(title)

        self.pwd_edit = QLineEdit()
        self.pwd_edit.setPlaceholderText("请输入登录密码")
        self.pwd_edit.setEchoMode(QLineEdit.Password)
        self.pwd_edit.setStyleSheet("""
            QLineEdit {
                background: #1a1a30; color: #fff; border: 2px solid #333;
                border-radius: 6px; padding: 10px; font-size: 16px;
            }
            QLineEdit:focus { border-color: #3388ff; }
        """)
        self.pwd_edit.returnPressed.connect(self.check_password)
        bl.addWidget(self.pwd_edit)

        self.msg_label = QLabel("")
        self.msg_label.setStyleSheet("color: #f55; font-size: 13px;")
        self.msg_label.setAlignment(Qt.AlignCenter)
        bl.addWidget(self.msg_label)

        btn = QPushButton("登  录")
        btn.setStyleSheet("""
            QPushButton {
                background: #2266cc; color: #fff; border: none;
                border-radius: 6px; padding: 10px; font-size: 16px; font-weight: bold;
            }
            QPushButton:hover { background: #3388ee; }
            QPushButton:pressed { background: #1a55aa; }
        """)
        btn.clicked.connect(self.check_password)
        bl.addWidget(btn)

        layout.addWidget(bottom)

    def check_password(self):
        if self.pwd_edit.text() == LOGIN_PASSWORD:
            self.accept()
        else:
            self.msg_label.setText("密码错误")
            self.pwd_edit.clear()
            self.pwd_edit.setFocus()


if __name__ == "__main__":
    app = QApplication(sys.argv)

    # 登录
    login = LoginDialog()
    if login.exec() != QDialog.Accepted:
        sys.exit(0)

    window = MonitorWindow()
    window.show()
    app.processEvents()
    QTimer.singleShot(100, window.init_video)
    sys.exit(app.exec())
