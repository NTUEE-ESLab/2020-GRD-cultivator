from bluepy.btle import Peripheral, UUID
from bluepy.btle import Scanner, DefaultDelegate
import threading
import time
import asyncio
import websockets
import pickle
import json
import socket

led_light = bytes(0x00)
ID = 0
run = True


class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)


scanner = Scanner().withDelegate(ScanDelegate())
devices = scanner.scan(3.0)
n = 0
data = []
ave_dis = []
time_from_start = 0
reading_near = False
reading_long = False
# web socket
Gateway_IP = '127.0.0.1'  # for websocket server


async def web_socket(websocket, path):
    global data
    global run
    global reading_near
    global reading_long
    data_time = 0
    data_len = 0
    while(run):

        if(len(data) > data_len):
            data_len = len(data)
            data_web = data[-1]
            if(data_web[1] > data_time + 1):

                data_time = int(data_web[1])
                data_val = data_web[0]
                # send data to socket
                data_web = "{\"val\":%d,\"time\":%d,\"near\":%d,\"long\":%d}" % (
                    data_val, data_time, reading_near, reading_long)

                line = await websocket.recv()
                # wait for notification
                if line is None:
                    print("line is None")
                    return
                await websocket.send(data_web)
# data_process
reading_state = 0  # from 0 to 5
reading = False


def data_process():
    global data
    global run
    global ave_dis
    global reading
    global reading_state
    threshold = 100
    ave_dis_len = 0
    # print("data_process")
    while(run):
        if((len(ave_dis) > 2) and (len(ave_dis) > ave_dis_len)):
            ave_dis_len = len(ave_dis)

            if ((abs(ave_dis[-1][0] - ave_dis[-2][0]) < threshold)
                    and (abs(ave_dis[-1][0] < 10000)) and (reading_state < 5)):
                reading_state = reading_state + 1
            elif(((abs(ave_dis[-1][0] - ave_dis[-2][0]) > threshold) or
                    ((ave_dis[-1][0] == 10000) and (ave_dis[-2][0] == 10000)
                     and (ave_dis[-3][0] == 10000))) and (reading_state > 0)):
                reading_state = reading_state - 1

        if(ave_dis_len > 0):
            if((reading_state == 4) or (reading_state == 5)):
                reading = True
            else:
                reading = False
        time.sleep(1)


def reading_judge():
    global reading
    global ave_dis
    global run
    global reading_near
    global reading_long
    global time_from_start
    mode = False
    time_rec = 0
    while(run):
        # record time
        if(mode == False):
            if((reading == True)):
                mode = True
                time_rec = time_from_start
            else:
                mode = False
        else:
            if((reading == False)):
                mode = False
        # reading_long
        if((mode == True) and (reading == True)
           and (abs(time_rec - time_from_start) > 0.25 * 60)):  # 0.25min
            reading_long = True
            print("reading_long! time: ", abs(
                time_rec - time_from_start), " sec")
        else:
            reading_long = False
        # reading_near
        if((reading == True) and (ave_dis[-1][0] < 350)):
            reading_near = True
            print("reading_near! distance: ", ave_dis[-1][0], " mm")
        else:
            reading_near = False

        time.sleep(1)


for dev in devices:
    print(n, ": Device ", dev.addr, "(", dev.addrType, ")",
          ", RSSI= ", dev.rssi, " dB")
    n += 1
    for (adtype, desc, value) in dev.getScanData():
        print(desc, "=", value)
number = input('Enter your device number: ')
print('Device', number)
print(list(devices)[int(number)].addr)
print("Connecting...")
dev = Peripheral(list(devices)[int(number)].addr, 'random')
dev.withDelegate(ScanDelegate())
print("Services...")
for svc in dev.services:
    print("dev: ", str(svc))
try:

    testService = dev.getServiceByUUID(UUID(0xa000))
    for ch in testService.getCharacteristics():
        print("0xa000: ", str(ch))

    but_ch = dev.getCharacteristics(uuid=UUID(0xa001))[0]
    print("but_ch: ", but_ch)

    time_start = time.perf_counter()

    def ble_receive():
        global data
        global run
        global but_ch
        global time_start
        global time_from_start
        data_len = 0

        while(run):
            if (but_ch.supportsRead()):
                time_from_start = time.perf_counter() - time_start
                data.append(
                    [int.from_bytes(but_ch.read(), byteorder='little'), time_from_start])
                # simple filter
                if(((len(data) % 10) == 0) and (data_len != len(data))):
                    data_len = len(data)
                    average_ = 0
                    for i in range(10):
                        average_ = average_ + data[-i - 1][0]
                    average_ = average_ / 10.0
                    ave_dis.append([average_, data[-i][1]])

            time.sleep(0.05)

    t1 = threading.Thread(target=ble_receive)
    t1.start()

    t2 = threading.Thread(target=data_process)
    t2.start()

    t3 = threading.Thread(target=reading_judge)
    t3.start()

    # web socket
    start_server = websockets.serve(web_socket, "127.0.0.1", 8866)
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()

    """
    while True:
        if (but_ch.supportsRead()):
            time_from_start = time.perf_counter()-time_start
            data.append(
                [int.from_bytes(but_ch.read(), byteorder='little'), time_from_start])
            print("but_supportsRead: ", data[-1])
    """


except KeyboardInterrupt:
    run = False
    t1.join()
    t2.join()
    t3.join()
    dev.disconnect()
    print("KeyboardInterrupt")

finally:
    t1.join()
    dev.disconnect()

# print(data)
