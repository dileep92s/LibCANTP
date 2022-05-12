import time
from typing import overload
import can

from threading import Thread
from threading import Condition
from abc import ABC, abstractmethod

class CANTP(can.Listener):

    class Observer(ABC):
        @abstractmethod
        def update(self, data):
            pass

    # -- can.Listener Overrides --

    def on_error(self, exc: Exception) -> None:
        print(exc)
        return super().on_error(exc)

    def on_message_received(self, msg) -> None:
        print(msg)

        id = msg.arbitration_id
        data = msg.data

        if id == self.rxid:

            if data[0] & 0xF0 == 0x00:
                self.received_blocks = 0
                self.readSingleFrame(data)
                return

            if data[0] & 0xF0 == 0x10:
                self.readFirstFrame(data)
                self.writeFlowControlFrame()
                return
            
            if data[0] & 0xF0 == 0x20:
                self.received_blocks += 1
                self.readConsecutiveFrame(data)
                time.sleep(self.st_min_for_rx/10e3)
                if (self.received_blocks % self.blk_size_for_rx) == 0 and self.rx_data_size:
                    self.writeFlowControlFrame()

            if data[0] & 0xF0 == 0x30:
                self.readFlowControlFrame(data)
                return

    # Init

    def __init__(self, bus, txid, rxid):
        self.tx_cond = Condition()
        self.flow_ctrl_ok = False
        self.st_min_for_tx = 0x64
        self.st_min_for_rx = 0x64
        self.blk_size_for_tx = 4
        self.blk_size_for_rx = 4
        self.rx_data = []
        self.rx_data_size = 0
        self.max_blk_size = 4096
        self.received_blocks = 0
        self.txid, self.rxid = txid, rxid
        self.observers = []

        self.bus = bus
        can.Notifier(self.bus, [self])


    # API for reading data - register as observer

    def addObserver(self, observer:Observer):
        self.observers.append(observer)

    def notify(self):
        for observer in self.observers:
            observer.update(self.rx_data)

    ## Read Data
    def readSingleFrame(self, data):
        size = data[0]
        self.rx_data = [hex(byte) for byte in bytes(data[1: size+1])]
        self.notify()
    
    def readFirstFrame(self, data):
        self.rx_data_size = ((data[0] & 0x0F) << 8) | data[1]
        self.rx_data = [hex(byte) for byte in bytes(data[2: 8])]
        self.rx_data_size -= (8-2)

    def readConsecutiveFrame(self, data):
        if self.rx_data_size >= 7:
            self.rx_data += [hex(byte) for byte in bytes(data[1: 8])]
            self.rx_data_size -= (8-1)
        else:
            self.rx_data += [hex(byte) for byte in bytes(data[1: self.rx_data_size+1])]
            self.rx_data_size -= self.rx_data_size

        if self.rx_data_size == 0:
            self.notify()

    def readFlowControlFrame(self, data):
        self.flow_ctrl_ok = True
        self.st_min_for_tx = (data[2] / 10e3) if (data[2] <= 0x7F) else (self.st_min_for_tx + (data[2] / 10e6))
        self.blk_size_for_tx = data[1] if data[1] else self.max_blk_size
        with self.tx_cond:
            self.tx_cond.notify_all()

    ## Write Data

    def sendMessage(self, msg):
        msg = can.Message(arbitration_id=self.txid, data=msg, is_extended_id=False)
        self.bus.send(msg)
    
    def writeSingleFrame(self, data):
        data_len = len(data)
        msg = [data_len] + data
        msg += [0x00 for i in range(8-len(msg))]
        self.sendMessage(msg)

    def writeFirstFrame(self, data):
        self.seq = 0
        data_len = len(data)
        last_idx = 6
        msg = [0x10 | ((data_len & 0xF00) >> 8)] + [(data_len & 0xFF)] + data[:last_idx]
        self.sendMessage(msg)
        return data[last_idx:]

    def writeConsecutiveFrame(self, data):
        data_len = len(data)
        last_idx = (data_len % 8) if (data_len < 7) else 7
        self.seq = (self.seq + 1) % 16
        msg = [0x20 | self.seq] + data[:last_idx]
        msg += [0x00 for i in range(8-len(msg))]
        self.sendMessage(msg)
        return data[last_idx:]

    def writeFlowControlFrame(self):
        msg = [0x30, self.blk_size_for_rx, self.st_min_for_rx, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.sendMessage(msg)

    def writeMultiFrame(self, data):
        # reset
        block_count = 0
        self.flow_ctrl_ok = False

        # send first frame
        data = self.writeFirstFrame(data)
        data_len = len(data)

        while data_len:
            with self.tx_cond:

                # wait for ctrl flow msg
                if self.tx_cond.wait_for(lambda: self.flow_ctrl_ok, 2):

                    # send consecutive frame
                    data = self.writeConsecutiveFrame(data)
                    data_len = len(data)
                    time.sleep(self.st_min_for_tx)
                    block_count += 1

                    # all blocks sent, wait for ctrl flow again
                    if block_count % self.blk_size_for_tx == 0:
                        self.flow_ctrl_ok  = False
                        block_count = 0
                
                # timeout occurred
                else:
                    print("flow ctrl timeout!")
                    break

    # API for writing data
    def sendData(self, data):
        if len(data) < 8:
            th = Thread(target=self.writeSingleFrame(data))
        else:
            th = Thread(target=self.writeMultiFrame(data))

        th.start()
        th.join()


class DiagRx(CANTP.Observer):
    def update(self, data):
        print("notf", data)

bus2 = can.Bus('test', bustype='virtual')
tp2 = CANTP(bus2, 0x72F, 0x727)
rx = DiagRx()
tp2.addObserver(rx)

bus = can.Bus('test', bustype='virtual')
tp = CANTP(bus, 0x727, 0x72F)

data = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F]
tp.sendData(data)
data = [0x0a, 0x0b, 0x0c]
tp.sendData(data)
data = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F]
tp.sendData(data)

while True:
    time.sleep(1)