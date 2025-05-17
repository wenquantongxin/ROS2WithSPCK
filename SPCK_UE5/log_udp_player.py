#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS日志UDP回放器 - 读取 Result_UDPall.log 文件并通过UDP发送数据
支持进度条控制和多种回放速率
"""

import sys
import os
import socket
import time
import struct
import numpy as np
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QLabel, QPushButton, QSlider, QComboBox,
                           QGridLayout, QGroupBox, QFileDialog, 
                           QProgressBar, QLineEdit)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt6.QtGui import QFont

# 常量定义
DEFAULT_LOG_PATH = "/home/yaoyao/Documents/myProjects/ROS2WithSPCK/PostAnalysis/Result_UDPall.log"
DEFAULT_TARGET_IP = "192.168.1.131"
DEFAULT_TARGET_PORT = 10099

# 仅保留 0.2x、0.5x、1x、2x
PLAY_SPEEDS = {
    "0.2x": 0.2,
    "0.5x": 0.5,
    "1x": 1.0,
    "2x": 2.0
}

class DataLoader(QThread):
    """数据加载线程，用于异步加载大型日志文件"""
    progress_signal = pyqtSignal(int)
    finished_signal = pyqtSignal(list, list, list)
    error_signal = pyqtSignal(str)
    
    def __init__(self, filepath):
        super().__init__()
        self.filepath = filepath
        
    def run(self):
        try:
            # 首先统计文件行数以计算进度
            # 使用 "with" 语句保证文件在读取后被正确关闭
            with open(self.filepath, 'r') as f:
                total_lines = sum(1 for _ in f)
            
            data = []
            times = []
            column_names = []
            
            with open(self.filepath, 'r') as file:
                # 读取标题行
                header = file.readline().strip()
                column_names = [col.strip('"') for col in header.split('\t')]
                
                # 逐行读取数据
                line_count = 1  # 已经读过1行标题
                for line in file:
                    line_count += 1
                    line = line.strip()
                    if line:
                        values = [float(v) for v in line.split('\t')]
                        times.append(values[0])  # 第一列是时间
                        data.append(values)
                    
                    # 每读1000行更新一次进度
                    if line_count % 1000 == 0:
                        progress = int(100 * line_count / total_lines)
                        # 确保不会超100
                        progress = 100 if progress > 100 else progress
                        self.progress_signal.emit(progress)
            
            # 读取完成后，手动将进度置为100%
            self.progress_signal.emit(100)
            
            # 转换为NumPy数组，便于后续处理
            times = np.array(times)
            data = np.array(data)
            
            self.finished_signal.emit(column_names, times, data)
            
        except Exception as e:
            self.error_signal.emit(f"数据加载错误: {str(e)}")


class UDPSender(QThread):
    """UDP发送线程，用于按照指定速率发送数据"""
    progress_signal = pyqtSignal(float)
    status_signal = pyqtSignal(str)
    data_info_signal = pyqtSignal(str, str)
    
    def __init__(self, data, times, ip, port, speed_factor=1.0, start_index=0):
        super().__init__()
        self.data = data
        self.times = times
        self.ip = ip
        self.port = port
        self.speed_factor = speed_factor
        self.start_index = start_index
        self.running = True
        self.paused = False
        self.sock = None
        
    def run(self):
        try:
            # 创建UDP套接字
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            
            # 从指定索引开始发送
            i = self.start_index
            start_time = time.time()
            sim_start_time = self.times[i]
            
            self.status_signal.emit("正在发送UDP数据...")
            
            while i < len(self.times) and self.running:
                # 暂停逻辑
                if self.paused:
                    time.sleep(0.1)
                    continue
                
                # 计算当前仿真时间
                current_real_time = time.time() - start_time
                target_sim_time = sim_start_time + current_real_time * self.speed_factor
                
                # 找到下一个要发送的数据点
                while i < len(self.times) and self.times[i] <= target_sim_time:
                    # 发送数据
                    data_to_send = self.data[i]
                    packed_data = struct.pack(f'{len(data_to_send)}d', *data_to_send)
                    self.sock.sendto(packed_data, (self.ip, self.port))
                    
                    # 更新显示信息
                    current_time = self.times[i]
                    percent = (i / (len(self.times) - 1)) * 100
                    self.progress_signal.emit(percent)
                    
                    self.data_info_signal.emit(
                        f"时间: {current_time:.3f}s", 
                        f"速度: {self.data[i][2]:.2f} m/s"
                    )
                    
                    i += 1
                    
                    # 判断是否到达末尾
                    if i >= len(self.times):
                        break
                
                # 控制发送速率，避免CPU占用过高
                time.sleep(0.001)
            
            if i >= len(self.times):
                self.status_signal.emit("数据发送完成")
            else:
                self.status_signal.emit("数据发送已停止")
                
        except Exception as e:
            self.status_signal.emit(f"发送错误: {str(e)}")
        finally:
            if self.sock:
                self.sock.close()
    
    def set_speed(self, speed):
        """设置播放速度倍率"""
        self.speed_factor = speed
    
    def pause(self):
        """暂停发送"""
        self.paused = True
        self.status_signal.emit("已暂停")
    
    def resume(self):
        """恢复发送"""
        self.paused = False
        self.status_signal.emit("正在发送UDP数据...")
    
    def stop(self):
        """停止发送"""
        self.running = False


class MainWindow(QMainWindow):
    """主窗口类"""
    def __init__(self):
        super().__init__()
        self.init_ui()
        
        # 数据相关变量
        self.column_names = []
        self.times = []
        self.data = []
        self.data_loaded = False
        
        # UDP发送器
        self.sender = None
        
        # 播放器状态：False表示当前未播放或已暂停，True表示正在播放
        self.is_playing = False
        
        # 自动加载默认文件
        self.load_data(DEFAULT_LOG_PATH)
    
    def init_ui(self):
        """初始化UI界面"""
        self.setWindowTitle("SPCK-ROS2-UE5日志UDP回放器")
        # 高度改为原先的2/3：从 600 -> 400；宽度保持800
        self.setGeometry(100, 100, 800, 400)
        
        # 创建中央部件
        central_widget = QWidget()
        main_layout = QVBoxLayout(central_widget)
        
        # ===== 顶部控制区 =====
        control_group = QGroupBox("控制面板")
        control_layout = QGridLayout()
        
        # 文件选择
        self.file_path_label = QLabel(DEFAULT_LOG_PATH)
        self.file_path_label.setWordWrap(True)
        self.file_button = QPushButton("选择日志文件")
        self.file_button.clicked.connect(self.select_file)
        
        # IP和端口设置（改为可编辑的输入框）
        self.ip_label = QLabel("目标IP:")
        self.ip_input = QLineEdit(DEFAULT_TARGET_IP)
        self.port_label = QLabel("端口:")
        self.port_input = QLineEdit(str(DEFAULT_TARGET_PORT))
        
        # 进度控制
        self.progress_label = QLabel("时间进度:")
        self.progress_slider = QSlider(Qt.Orientation.Horizontal)
        self.progress_slider.setRange(0, 1000)
        self.progress_slider.valueChanged.connect(self.on_slider_change)
        
        # 播放速度控制
        self.speed_label = QLabel("播放速度:")
        self.speed_combo = QComboBox()
        for speed in PLAY_SPEEDS.keys():
            self.speed_combo.addItem(speed)
        self.speed_combo.setCurrentText("1x")
        self.speed_combo.currentTextChanged.connect(self.on_speed_change)
        
        # 播放/暂停控制按钮（合并为一个）
        self.play_pause_button = QPushButton("播放")
        self.play_pause_button.clicked.connect(self.on_play_pause)
        self.play_pause_button.setEnabled(False)
        
        # 停止按钮
        self.stop_button = QPushButton("停止以拖动进度条")
        self.stop_button.clicked.connect(self.on_stop)
        self.stop_button.setEnabled(False)
        
        # 布局排列
        control_layout.addWidget(self.file_button, 0, 0)
        control_layout.addWidget(self.file_path_label, 0, 1, 1, 3)
        
        control_layout.addWidget(self.ip_label, 1, 0)
        control_layout.addWidget(self.ip_input, 1, 1)
        control_layout.addWidget(self.port_label, 1, 2)
        control_layout.addWidget(self.port_input, 1, 3)
        
        control_layout.addWidget(self.progress_label, 2, 0)
        control_layout.addWidget(self.progress_slider, 2, 1, 1, 3)
        
        control_layout.addWidget(self.speed_label, 3, 0)
        control_layout.addWidget(self.speed_combo, 3, 1)
        
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.play_pause_button)
        button_layout.addWidget(self.stop_button)
        control_layout.addLayout(button_layout, 3, 2, 1, 2)
        
        control_group.setLayout(control_layout)
        
        # ===== 底部状态区 =====
        status_group = QGroupBox("状态信息")
        status_layout = QGridLayout()
        
        self.current_time_label = QLabel("时间: 0.000s")
        self.current_speed_label = QLabel("速度: 0.00 m/s")
        
        self.loading_progress = QProgressBar()
        self.loading_progress.setRange(0, 100)
        self.loading_progress.setValue(0)
        
        self.status_label = QLabel("准备就绪")
        font = QFont()
        font.setBold(True)
        self.status_label.setFont(font)
        
        status_layout.addWidget(QLabel("加载进度:"), 0, 0)
        status_layout.addWidget(self.loading_progress, 0, 1, 1, 3)
        status_layout.addWidget(self.current_time_label, 1, 0, 1, 2)
        status_layout.addWidget(self.current_speed_label, 1, 2, 1, 2)
        status_layout.addWidget(self.status_label, 2, 0, 1, 4)
        
        status_group.setLayout(status_layout)
        
        # 添加到主布局
        main_layout.addWidget(control_group)
        main_layout.addWidget(status_group)
        
        self.setCentralWidget(central_widget)
        
        # 设置一个定时器来更新UI
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_ui)
        self.update_timer.start(100)  # 100毫秒更新一次
    
    def select_file(self):
        """选择日志文件"""
        filepath, _ = QFileDialog.getOpenFileName(
            self, "选择日志文件", os.path.dirname(DEFAULT_LOG_PATH), "Log Files (*.log)"
        )
        if filepath:
            self.file_path_label.setText(filepath)
            self.load_data(filepath)
    
    def load_data(self, filepath):
        """加载日志文件数据"""
        if not os.path.exists(filepath):
            self.status_label.setText(f"错误: 文件不存在 - {filepath}")
            return
        
        self.status_label.setText(f"正在加载文件...")
        self.play_pause_button.setEnabled(False)
        self.data_loaded = False
        
        # 创建并启动数据加载线程
        self.loader = DataLoader(filepath)
        self.loader.progress_signal.connect(self.loading_progress.setValue)
        self.loader.finished_signal.connect(self.on_data_loaded)
        self.loader.error_signal.connect(self.on_loading_error)
        self.loader.start()
    
    def on_data_loaded(self, column_names, times, data):
        """数据加载完成回调"""
        self.column_names = column_names
        self.times = times
        self.data = data
        
        self.data_loaded = True
        self.status_label.setText(f"数据加载完成, 共 {len(times)} 条记录, 时长 {times[-1]:.2f}s")
        self.play_pause_button.setEnabled(True)
        self.progress_slider.setValue(0)
    
    def on_loading_error(self, error_msg):
        """数据加载错误回调"""
        self.status_label.setText(error_msg)
    
    def on_slider_change(self):
        """进度条拖动回调"""
        if not self.data_loaded or len(self.times) == 0:
            return
        
        # 计算对应的时间索引
        percentage = self.progress_slider.value() / 1000.0
        index = int(percentage * (len(self.times) - 1))
        
        time_val = self.times[index]
        self.current_time_label.setText(f"时间: {time_val:.3f}s")
        
        # 更新车辆速度显示
        if index < len(self.data):
            speed = self.data[index][2]  # 速度在第三列
            self.current_speed_label.setText(f"速度: {speed:.2f} m/s")
    
    def on_speed_change(self, speed_text):
        """播放速度改变回调"""
        if self.sender:
            self.sender.set_speed(PLAY_SPEEDS[speed_text])
    
    def on_play_pause(self):
        """播放/暂停按钮回调"""
        if not self.data_loaded:
            return
        
        # 如果当前没有在播放，则开始播放（或从暂停中恢复）
        if not self.is_playing:
            # 如果还没有 sender 或者 sender 已经结束，则重新创建 sender
            if not self.sender or not self.sender.isRunning():
                # 获取当前进度条对应的索引
                percentage = self.progress_slider.value() / 1000.0
                start_index = int(percentage * (len(self.times) - 1))
                
                # 读取用户输入的IP和端口
                ip = self.ip_input.text().strip()
                try:
                    port = int(self.port_input.text().strip())
                except ValueError:
                    port = DEFAULT_TARGET_PORT
                
                speed_factor = PLAY_SPEEDS[self.speed_combo.currentText()]
                self.sender = UDPSender(
                    self.data, self.times, 
                    ip, port, 
                    speed_factor, start_index
                )
                self.sender.progress_signal.connect(self.on_playback_progress)
                self.sender.status_signal.connect(self.on_sender_status)
                self.sender.data_info_signal.connect(self.on_data_info)
                self.sender.start()
            
            else:
                # 如果 sender 正在运行，但处于暂停状态，则恢复
                self.sender.resume()
            
            self.is_playing = True
            self.play_pause_button.setText("暂停")
            self.stop_button.setEnabled(True)
        
        else:
            # 如果当前正在播放，则暂停
            if self.sender and self.sender.isRunning():
                self.sender.pause()
            self.is_playing = False
            self.play_pause_button.setText("播放")
    
    def on_stop(self):
        """停止按钮回调"""
        if self.sender and self.sender.isRunning():
            self.sender.stop()
            self.sender.wait()
        
        # 恢复到初始状态
        self.is_playing = False
        self.play_pause_button.setText("播放")
        self.stop_button.setEnabled(False)
    
    def on_playback_progress(self, percent):
        """回放进度更新回调"""
        self.progress_slider.setValue(int(percent * 10))
    
    def on_sender_status(self, status):
        """发送器状态更新回调"""
        self.status_label.setText(status)
        
        # 如果发送完成或出错，则重置播放按钮状态
        if status in ["数据发送完成", "数据发送已停止"] or status.startswith("发送错误"):
            self.is_playing = False
            self.play_pause_button.setText("播放")
            self.stop_button.setEnabled(False)
    
    def on_data_info(self, time_text, speed_text):
        """数据信息更新回调"""
        self.current_time_label.setText(time_text)
        self.current_speed_label.setText(speed_text)
    
    def update_ui(self):
        """定时更新UI"""
        # 可以在这里添加需要定时更新的UI元素
        pass
    
    def closeEvent(self, event):
        """窗口关闭事件"""
        # 确保所有线程都停止
        if self.sender and self.sender.isRunning():
            self.sender.stop()
            self.sender.wait()
        
        event.accept()


def main():
    """主函数"""
    app = QApplication(sys.argv)
    
    # 设置应用样式
    app.setStyle("Fusion")
    
    window = MainWindow()
    window.show()
    
    sys.exit(app.exec())


if __name__ == "__main__":
    main()



'''

 运行：

    conda activate pyqt 
    cd /home/yaoyao/Documents/myProjects/ROS2WithSPCK/SPCK_UE5 
    python log_udp_player.py

'''