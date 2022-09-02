import serial
import time

data = ['0000','0001','0002','0003','0004','0005','0006','0007','0008','0009','000A','000B','000C','000D','000E','000F']
if __name__ == "__main__":

    # 串口执行到这已经打开 再用open命令会报错 57600
    mSerial = serial.Serial('COM3', 115200)
    mSerial.open()
    if mSerial.isOpen():
        print("open success")
        mSerial.flushInput()  # 清空缓冲区

    else:
        print("open failed")
        mSerial.close()  # 关闭端口
    index = 0
    # timer = pg.QtCore.QTimer()
    # timer.timeout.connect()  # 定时刷新数据显示
    # timer.start(2)  # 多少ms调用一次
    Count = 1000
    for i in range(Count):
        strdata = 'AAAA048002'+data[index % len(data)]+'11'
        Hex_str = bytes.fromhex(strdata)
        index += 1
        num = mSerial.write(Hex_str)
        time.sleep(0.02)
        print(i, num, strdata)
        # break