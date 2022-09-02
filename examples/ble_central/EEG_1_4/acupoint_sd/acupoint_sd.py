# coding:utf8
# 时间：2020.5.1
# 适用于测试穴位电路的串扰问题
import pyqtgraph as pg
import array
import serial
import threading
import numpy as np
from queue import Queue
import time
from PyQt5 import QtWidgets
import sys
from PyQt5.QtWidgets import QFileDialog
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot
from PyQt5.QtGui import *
from PyQt5 import QtGui
import pytz
from datetime import datetime, timedelta

i = 0
control = 0
data_length = 1000
curve = []
data = []
data_bytes = bytearray()

que = []
index_now = []
process = []
f = []
velocity = 0
label = []
rate = 0
mean_array = [[0 for i in range(50)] for j in range(3)]
mean_index = np.zeros(3)

pre_pos = 0
change_times = 5
xxx = [i1 * 17 / data_length for i1 in range(data_length)]

def hex_to_int(s):
    # assert s[:2] == "0B"
    return int("0x" + s, 16)


def around_day_time(start_time, time_zone) -> (datetime, datetime):
    tz = pytz.timezone(time_zone)
    dt = datetime.fromtimestamp(start_time, tz)
    # 向前修正时间为双数
    hours_offset = dt.hour % 2
    hour_around = dt - timedelta(microseconds=dt.microsecond, minutes=dt.minute, seconds=dt.second, hours=hours_offset)
    day_around = hour_around - timedelta(hours=hour_around.hour)
    # print(dt, hour_around, day_around)
    return hour_around, day_around


def get_day_hour(index, hour_around_time, day_around_time):
    assert index > 0
    index_time = hour_around_time + timedelta(hours=2 * index)
    delta = index_time - day_around_time
    days = delta.days + 1
    hour = delta.total_seconds() // 3600
    return days, hour


def parse(start_time, data, sensor_index=1):
    hour_around_time, day_around_time = around_day_time(start_time, 'Asia/Shanghai')
    data = data.decode()
    data = data.strip().split(" ")
    # print(data)
    assert len(data) % 4 == 0
    res = []
    while len(data) > 0:
        index, d1, d2, d3 = data[:4]
        data = data[4:]
        index = int(index)
        d1, d2, d3 = [round(hex_to_int(d) * 3600 / (4096 * 8.5)) for d in (d1, d2, d3)]
        day, hour = get_day_hour(index, hour_around_time, day_around_time)
        hour = hour // 2
        print(hour, day, d1, d2, d3)
        res.append((hour, sensor_index, d1, d2, d3))
    return res


def write_csv(file_path, data):
    with open(file_path, "w") as f:
        for d in data:
            f.write(",".join([str(n) for n in d]) + "\n")

# this_timestamp_line = None


def serial_xx(gui):
    global data_bytes
    global f, control
    global rate, mean_array, mean_index, pre_pos, change_times
    # global this_timestamp_line
    while True:
        count = mSerial.inWaiting()
        if count:
            rec_str = mSerial.read(count)
            data_bytes = data_bytes + rec_str
            data_len = len(data_bytes)
            k = 0
            while k + 14 <= data_len:
                if data_bytes[k] == 0XFF and data_bytes[k + 1] == 0X88 and data_bytes[k + 12] == data_bytes[k + 13]:
                    if data_bytes[k + 12] != pre_pos:
                        change_times = 0
                    if change_times > 1:
                        t = time.time()
                        rate = rate + 1
                        loc = data_bytes[k + 12]
                        if control:
                            f.write('\r\n' + str(round(t * 1000)) + ' ' + str(loc) + ' ')

                        for k2 in range(5):
                            data_put = data_bytes[k + 2 + k2 * 2] * 256 + data_bytes[k + 3 + k2 * 2]
                            if data_put > 5000:
                                que[loc].put(0)
                            elif data_put < 3800:
                                mean_array[loc][int(mean_index[loc] % 50)] = data_put
                                mean_index[loc] += 1
                                data_mean = np.mean(mean_array[loc])
                                que[loc].put(data_mean)
                            if control:
                                f.write(str(data_put) + ' ')
                    change_times += 1
                    pre_pos = data_bytes[k + 12]
                    k = k + 14
                elif data_bytes[k] == 0XFF and data_bytes[k + 1] == 0X88 and data_bytes[k + 12] == 0XAA and data_bytes[k + 13] == 0XBB:
                    this_time = data_bytes[k + 2] * 16777216 + data_bytes[k + 3] * 65536 + data_bytes[k + 4] * 256 + \
                                data_bytes[k + 5]
                    gui.this_timestamp_line.setText(str(this_time))

                    now_time = data_bytes[k + 6] * 16777216 + data_bytes[k + 7] * 65536 + data_bytes[k + 8] * 256 + \
                               data_bytes[k + 9]
                    gui.now_timestamp_line.setText(str(now_time))

                    time_state = data_bytes[10] - 1
                    gui.timer_state_line.setText(str(time_state))
                    k = k + 14
                # 收到从机的拒绝FRStart命令
                elif data_bytes[k] == 0XFF and data_bytes[k + 1] == 0X88 and data_bytes[k + 10] <= 244 and data_bytes[k + 11] == 0XAA and data_bytes[k + 12] == 0XAA and data_bytes[k + 13] == 0XCC:
                    this_time = data_bytes[k + 2] * 16777216 + data_bytes[k + 3] * 65536 + data_bytes[k + 4] * 256 + \
                                data_bytes[k + 5]
                    gui.this_timestamp_line.setText(str(this_time))

                    now_time = data_bytes[k + 6] * 16777216 + data_bytes[k + 7] * 65536 + data_bytes[k + 8] * 256 + \
                               data_bytes[k + 9]
                    gui.now_timestamp_line.setText(str(now_time))
                    time_state = data_bytes[10] - 1
                    gui.timer_state_line.setText(str(time_state))
                    gui.warningSignal.emit(('指令未被执行', '请点击“停止时辰模式”按钮，停止上一次的穴位时辰采集后，再开始本次采样！', QMessageBox.Ok))
                    k = k + 14
                # 收到从机的拒绝FRead命令
                elif data_bytes[k] == 0XFF and data_bytes[k + 1] == 0X88 and data_bytes[k + 10] <= 244 and data_bytes[k + 11] == 0XAA and data_bytes[k + 12] == 0XAA and data_bytes[k + 13] == 0XDD:
                    this_time = data_bytes[k + 2] * 16777216 + data_bytes[k + 3] * 65536 + data_bytes[k + 4] * 256 + \
                                data_bytes[k + 5]
                    gui.this_timestamp_line.setText(str(this_time))

                    now_time = data_bytes[k + 6] * 16777216 + data_bytes[k + 7] * 65536 + data_bytes[k + 8] * 256 + \
                               data_bytes[k + 9]
                    gui.now_timestamp_line.setText(str(now_time))
                    time_state = data_bytes[10] - 1
                    gui.timer_state_line.setText(str(time_state))
                    gui.warningSignal.emit(('指令未被执行', '目前正在采集穴位数据，请在本次穴位数据采集完成后（每次采集约5分钟），再尝试读取已经采集和存储的穴位数据！', QMessageBox.Ok))
                    k = k + 14
                # 收到“未进入文件操作模式命令”
                elif data_bytes[k] == 0XFF and data_bytes[k + 1] == 0X88 and data_bytes[k + 10] <= 244 and data_bytes[k + 11] == 0XAA and data_bytes[k + 12] == 0XAA and data_bytes[k + 13] == 0XEE:
                    gui.warningSignal.emit(('指令未被执行', '请先点击“使能功能按钮”后，再进行时辰模式相关指令操作！', QMessageBox.Ok))
                    k = k + 14
                elif data_bytes[k] == 0X88 and data_bytes[k + 1] == 0X88 and data_bytes[k + 2] == 0X00 and data_bytes[k + 3] < 244:
                    if data_len-k == data_bytes[k + 3]+3:
                        print(data_bytes[k+4:])

                        time_stamp_set = gui.this_timestamp_line.text()
                        res = parse(int(time_stamp_set), data_bytes[k+4:])
                        write_csv(time_stamp_set + ".csv", res)
                        k = k + data_bytes[k + 3] + 3
                    elif data_len-k < data_bytes[k + 3]+3:
                        break

                else:
                    k = k + 1
            data_bytes[0:k] = b''


class MainWidget(QtWidgets.QMainWindow):
    warningSignal = pyqtSignal(tuple)

    def action_save(self):
        global f
        global control
        global rate, velocity, label, xxx

        if self.saveButton.text() == "开始记录数据":
            self.saveButton.setText("停止记录数据")
            fileName2, ok2 = QFileDialog.getSaveFileName(self,
                                                         "文件保存",
                                                         "./",
                                                         "Text Files (*.txt)")
            if not fileName2:
                fileName2 = "test1111.txt"
            f = open(fileName2, 'w')
            f.write('Acupoint Data:')
            control = 1
        elif self.saveButton.text() == "停止记录数据":
            self.saveButton.setText("开始记录数据")
            control = 0
            f.close()

    def action_save_push_time(self):
        file_time = open("插针及拔针时间.txt", 'a')
        t = time.time()
        file_time.write('\r\n' + "插针时间：" + time.asctime(time.localtime(t)) + ' ' + str(round(t * 1000)))
        file_time.close()

    def action_save_poll_time(self):
        file_time = open("插针及拔针时间.txt", 'a')
        t = time.time()
        file_time.write('\r\n' + "拔针时间：" + time.asctime(time.localtime(t)) + ' ' + str(round(t * 1000)))
        file_time.close()

    # @pyqtSlot()
    def warning(self, args):
        QMessageBox.warning(self, *args)

    def action_fileOn(self):
        mSerial.write(("FileOn" + "\r\n").encode())

    def action_start(self):
        mSerial.write((str(round(time.time())) + 'Start\r\n').encode())

    def action_stop(self):
        mSerial.write(("Stop" + "\r\n").encode())

    def action_getTS(self):
        mSerial.write(("Get" + "\r\n").encode())

    def action_read(self):
        if self.this_timestamp_line.text().isdigit() and len(self.this_timestamp_line.text())==10:
            mSerial.write((self.this_timestamp_line.text() + "FRead" + "\r\n").encode())
        else:
            self.warningSignal.emit(('指令未被执行', '请先点击“获取从机时间戳”按钮或在“本次时辰采集开始于”文本框中手动输入本次时辰采集的时间戳，请注意有效时间戳为10位！', QMessageBox.Ok))

    def action_fileOff(self):
        mSerial.write(("FileOff" + "\r\n").encode())

    def __init__(self):
        super().__init__()
        self.setWindowTitle("ImpedanceData")  # 设置窗口标题
        main_widget = QtWidgets.QWidget()  # 实例化一个widget部件
        main_layout = QtWidgets.QGridLayout()  # 实例化一个网格布局层
        main_widget.setLayout(main_layout)  # 设置主widget部件的布局为网格布局

        palette = QPalette()
        palette.setColor(QPalette.Background, Qt.white)
        self.setPalette(palette)
        self.setGeometry(0, 25, 1920, 1000)
        main_widget.setFixedSize(1920, 1000)

        # self.warningSignal = pyqtSignal(tuple)

        self.warningSignal.connect(self.warning)

        pw = pg.PlotWidget()
        pw.setLabel(axis='left', text='腧穴电阻抗 / 千欧')
        pw.setLabel(axis='bottom', text='时间 / 秒')

        for k, graph_title in zip(range(3),
                                  ['次外', '最外', '最里']):
            data.insert(k, array.array('i'))
            data[k] = np.zeros(data_length).__array__('d')
            que.insert(k, Queue(maxsize=0))
            index_now.insert(k, 0)

            # label.insert(k, QtWidgets.QLabel())
            # label[k].setAlignment(Qt.AlignCenter)
            # label[k].setText(' Efficiency:  0 %')

        label = QtWidgets.QLabel()
        label.setAlignment(Qt.AlignHCenter)
        label2 = QtWidgets.QLabel()
        jpg = QtGui.QPixmap("穴位定位.jpg").scaled(600, 700)
        label2.setStyleSheet("QLabel{background:white;}")
        label2.setPixmap(jpg)
        label.setStyleSheet('font-size:40px;color:black;font-family:黑体')
        label.setText("人体腧穴电阻测量系统")
        label3 = QtWidgets.QLabel()
        jpg2 = QtGui.QPixmap("图例.png").scaled(300, 250)
        label3.setPixmap(jpg2)
        label3.setStyleSheet("QLabel{background:white;}")

        label4 = QtWidgets.QLabel()
        jpg3 = QtGui.QPixmap("穴位辐射.png").scaled(180, 180)
        label4.setPixmap(jpg3)
        label4.setStyleSheet("QLabel{background:white;}")

        pw.setYRange(10, 1000)
        # 'b', 'y', 'r'
        for k, d, color in zip(range(3), data, [(122, 187, 251), (252, 210, 67), (238, 129, 116)]):
            # pen = pg.mkPen(color=color, width=5)
            pen = pg.mkPen(color=color)
            curve.insert(k, (pw.plot(x=xxx, y=d, pen=pen)))

        pw.setBackground("w")
        main_layout.addWidget(pw, 2, 7, 3, 8)  # 添加绘图部件到网格布局层
        main_layout.addWidget(label, 1, 5, 1, 2)
        main_layout.addWidget(label2, 2, 1, 3, 4)
        main_layout.addWidget(label3, 3, 5, 1, 2)
        main_layout.addWidget(label4, 2, 5, 1, 2)
        # pw[4].setRange(yRange=[25000, 65000])
        # pw[5].setRange(yRange=[25000, 65000])

        # 开启文件读取模式 按钮
        self.fileOnButton = QtWidgets.QPushButton(main_widget)
        self.fileOnButton.setText("使能功能按钮")
        self.fileOnButton.setStyleSheet("QPushButton{color:#FFA500}""QPushButton:hover{color:#DC143C}""QPushButton{background-color:#000000}""QPushButton{border:1px}""QPushButton{border-radius:10px}""QPushButton{padding:6px 6px}""QPushButton{font:bold 20px}")
        self.fileOnButton.clicked.connect(self.action_fileOn)
        main_layout.addWidget(self.fileOnButton, 5, 2, 1, 1, alignment=Qt.AlignCenter)

        # 开启时辰采样 按钮
        self.startButton = QtWidgets.QPushButton(main_widget)
        self.startButton.setText("开启时辰模式")
        self.startButton.setStyleSheet("QPushButton{color:#D2691E}""QPushButton:hover{color:#FF4500}""QPushButton{background-color:	#D3D3D3}""QPushButton{border:1px}""QPushButton{border-radius:10px}""QPushButton{padding:6px 6px}""QPushButton{font:bold 20px}")
        self.startButton.clicked.connect(self.action_start)
        main_layout.addWidget(self.startButton, 5, 3, 1, 1, alignment=Qt.AlignCenter)

        # 停止时辰采样 按钮
        self.stopButton = QtWidgets.QPushButton(main_widget)
        self.stopButton.setText("停止时辰模式")
        self.stopButton.setStyleSheet("QPushButton{color:#D2691E}""QPushButton:hover{color:	#FF4500}""QPushButton{background-color:	#D3D3D3}""QPushButton{border:1px}""QPushButton{border-radius:10px}""QPushButton{padding:6px 6px}""QPushButton{font:bold 20px}")
        self.stopButton.clicked.connect(self.action_stop)
        main_layout.addWidget(self.stopButton, 5, 4, 1, 1, alignment=Qt.AlignCenter)

        # 获取从机时间戳 按钮
        self.getTSButton = QtWidgets.QPushButton(main_widget)
        self.getTSButton.setText("获取设备时间戳")
        self.getTSButton.setStyleSheet("QPushButton{color:#D2691E}""QPushButton:hover{color:#FF4500}""QPushButton{background-color:	#D3D3D3}""QPushButton{border:1px}""QPushButton{border-radius:10px}""QPushButton{padding:6px 6px}""QPushButton{font:bold 20px}")
        self.getTSButton.clicked.connect(self.action_getTS)
        main_layout.addWidget(self.getTSButton, 5, 5, 1, 1, alignment=Qt.AlignCenter)

        # 获取文件 按钮
        self.readButton = QtWidgets.QPushButton(main_widget)
        self.readButton.setText("获取本次时辰采集已存储信息")
        self.readButton.setStyleSheet("QPushButton{color:#D2691E}""QPushButton:hover{color:	#FF4500}""QPushButton{background-color:	#D3D3D3}""QPushButton{border:1px}""QPushButton{border-radius:10px}""QPushButton{padding:6px 6px}""QPushButton{font:bold 20px}")
        self.readButton.clicked.connect(self.action_read)
        main_layout.addWidget(self.readButton, 5, 6, 1, 1, alignment=Qt.AlignCenter)

        # 关闭文件读取模式 按钮
        self.fileOffButton = QtWidgets.QPushButton(main_widget)
        self.fileOffButton.setText("开启实时传输模式")
        self.fileOffButton.setStyleSheet(
            "QPushButton{color:#FFA500}""QPushButton:hover{color:#DC143C}""QPushButton{background-color:#000000}""QPushButton{border:1px}""QPushButton{border-radius:10px}""QPushButton{padding:6px 6px}""QPushButton{font:bold 20px}")
        self.fileOffButton.clicked.connect(self.action_fileOff)
        main_layout.addWidget(self.fileOffButton, 5, 10, 1, 1, alignment=Qt.AlignCenter)

        # 保存按钮
        self.saveButton = QtWidgets.QPushButton(main_widget)
        self.saveButton.setText("开始记录数据")
        self.saveButton.setStyleSheet("QPushButton{color:#D2691E}""QPushButton:hover{color:	#FF4500}""QPushButton{background-color:	#D3D3D3}""QPushButton{border:1px}""QPushButton{border-radius:10px}""QPushButton{padding:6px 6px}""QPushButton{font:bold 20px}")
        self.saveButton.clicked.connect(self.action_save)
        main_layout.addWidget(self.saveButton, 5, 12, 1, 1, alignment=Qt.AlignCenter)

        # 记录扎针时间按钮
        self.savePushTimeButton = QtWidgets.QPushButton(main_widget)
        self.savePushTimeButton.setText("记录扎针时间")
        self.savePushTimeButton.setStyleSheet("QPushButton{color:#D2691E}""QPushButton:hover{color:	#FF4500}""QPushButton{background-color:	#D3D3D3}""QPushButton{border:1px}""QPushButton{border-radius:10px}""QPushButton{padding:6px 6px}""QPushButton{font:bold 20px}")
        self.savePushTimeButton.clicked.connect(self.action_save_push_time)
        main_layout.addWidget(self.savePushTimeButton, 6, 10, 1, 1, alignment=Qt.AlignCenter)

        # 记录拔针时间按钮
        self.savePollTimeButton = QtWidgets.QPushButton(main_widget)
        self.savePollTimeButton.setText("记录拔针时间")
        self.savePollTimeButton.setStyleSheet("QPushButton{color:#D2691E}""QPushButton:hover{color:	#FF4500}""QPushButton{background-color:	#D3D3D3}""QPushButton{border:1px}""QPushButton{border-radius:10px}""QPushButton{padding:6px 6px}""QPushButton{font:bold 20px}")
        self.savePollTimeButton.clicked.connect(self.action_save_poll_time)
        main_layout.addWidget(self.savePollTimeButton, 6, 12, 1, 1, alignment=Qt.AlignCenter)

        # 设置label和text
        self.this_timestamp_label = QtWidgets.QLabel()
        self.this_timestamp_label.setAlignment(Qt.AlignCenter)
        self.this_timestamp_label.setText('本次时辰采集开始于：')
        main_layout.addWidget(self.this_timestamp_label, 6, 1, 1, 1, alignment=Qt.AlignCenter)
        self.this_timestamp_line = QtWidgets.QLineEdit()
        main_layout.addWidget(self.this_timestamp_line, 6, 2, 1, 1, alignment=Qt.AlignCenter)

        self.now_timestamp_label = QtWidgets.QLabel()
        self.now_timestamp_label.setAlignment(Qt.AlignCenter)
        self.now_timestamp_label.setText('当前时辰采集时间戳：')
        main_layout.addWidget(self.now_timestamp_label, 6, 3, 1, 1, alignment=Qt.AlignCenter)
        self.now_timestamp_line = QtWidgets.QLineEdit()
        main_layout.addWidget(self.now_timestamp_line, 6, 4, 1, 1, alignment=Qt.AlignCenter)

        self.timer_state_label = QtWidgets.QLabel()
        self.timer_state_label.setAlignment(Qt.AlignCenter)
        self.timer_state_label.setText('当前已采集时辰个数：')
        main_layout.addWidget(self.timer_state_label, 6, 5, 1, 1, alignment=Qt.AlignCenter)
        self.timer_state_line = QtWidgets.QLineEdit()
        main_layout.addWidget(self.timer_state_line, 6, 6, 1, 1, alignment=Qt.AlignCenter)

        self.setCentralWidget(main_widget)  # 设置窗口默认部件为主widget


# ，图例，整体背景变白，把折线图调小 ，加上示意图

def consumer(a):
    while True:
        if index_now[a] < data_length:
            v = que[a].get() * 3.6 / 4096
            # 分压法计算公式
            # data[a][index_now[a]] = v * 10000 / (3.3 - v)
            # 恒流法计算公式
            data[a][index_now[a]] = v * 1000 / 8.5
            index_now[a] = index_now[a] + 1

        else:
            data[a][:-1] = data[a][1:]
            v = que[a].get() * 3.6 / 4096
            # 分压法计算公式
            # data[a][index_now[a] - 1] = v * 10000 / (3.3 - v)
            # 恒流法计算公式
            data[a][index_now[a] - 1] = v * 1000 / 8.5


def plot_data():
    global xxx
    for k in range(3):
        curve[k].setData(xxx, data[k])


if __name__ == "__main__":

    mSerial = serial.Serial('COM3', 115200)
    if mSerial.isOpen():
        print("open success")
        mSerial.flushInput()  # 清空缓冲区

    else:
        print("open failed")
        mSerial.close()  # 关闭端口
    app = QtWidgets.QApplication(sys.argv)

    gui = MainWidget()
    th1 = threading.Thread(target=serial_xx, args=(gui,))
    th1.start()
    gui.show()
    timer = pg.QtCore.QTimer()
    timer.timeout.connect(plot_data)  # 定时刷新数据显示
    timer.start(30)  # 多少ms调用一次
    timer1 = pg.QtCore.QTimer()
    timer1.start(3000)  # 多少ms调用一次

    for k1 in range(3):
        process.insert(k1, threading.Thread(target=consumer, args=(k1,)))
        process[k1].start()
    sys.exit(app.exec_())
