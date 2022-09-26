import time
import can

from threading import Thread
from threading import Event
from abc import ABC, abstractmethod

class CANTP(can.Listener):

    # ------------------- OBSERVER ------------------- #
    class Observer(ABC):
        @abstractmethod
        def on_cantp_msg_received(self, data):
            pass

    # ------------------- NOTIFIER ------------------- #

    # -- API for reading data - register as observer --
    def addObserver(self, observer:Observer):
        self.observers.append(observer)

    def notify(self):
        payload = self.rx_data[:self.rx_data_size]
        payload_str = " ".join(["{:02X}".format(byte) for byte in payload])
        print(f"CANTP::notify - {payload_str}")
        for observer in self.observers:
            observer.on_cantp_msg_received(payload)

    # ------------------- LISTENER ------------------- #
    
    # -- can.Listener Overrides --
    def on_error(self, exc: Exception) -> None:
        print(f"CANTP::error : {exc}")
        return super().on_error(exc)

    def on_message_received(self, msg) -> None:
        can_id = msg.arbitration_id
        data = msg.data

        if can_id == self.rxid:
            log_msg = "RX    -    ID: {:04X}    DL: {:02X}    Data: {}".format(can_id, msg.dlc, " ".join(["{:02X} ".format(byte) for byte in msg.data]))
            print(log_msg)

            if data[0] & 0xF0 == 0x00:
                self.readSingleFrame(data)
                return

            if data[0] & 0xF0 == 0x10:
                self.received_blocks = 0
                self.readFirstFrame(data)
                time.sleep(self.st_min_for_rx/10e3)
                self.writeFlowControlFrame()
                return

            if data[0] & 0xF0 == 0x20:
                self.received_blocks += 1
                self.readConsecutiveFrame(data)
                time.sleep(self.st_min_for_rx/10e3)
                if not (self.received_blocks % self.blk_size_for_rx):
                    self.writeFlowControlFrame()

            if data[0] & 0xF0 == 0x30:
                self.readFlowControlFrame(data)
                return
            
    # ------------------- CANTP STUFF ------------------- #
            
    # -- Init --
    def __init__(self, bus, txid, rxid):
        self.flow_ctrl_ok = Event()
        self.st_min_for_tx = 0x14 # 20ms
        self.st_min_for_rx = 0x14 # 20ms
        self.blk_size_for_tx = 4
        self.blk_size_for_rx = 4
        self.rx_data = []
        self.rx_data_size = 0
        self.max_blk_size = 4096
        self.received_blocks = 0
        self.txid, self.rxid = txid, rxid
        self.observers = []
        self.bus = bus

    ## -- Read Data --
    def readSingleFrame(self, data):
        self.rx_data_size = data[0]
        self.rx_data = [byte for byte in bytes(data[1:])]
        self.notify()

    def readFirstFrame(self, data):
        self.rx_data_size = ((data[0] & 0x0F) << 8) | data[1]
        self.rx_data = [byte for byte in bytes(data[2:])]

    def readConsecutiveFrame(self, data):
        self.rx_data += [byte for byte in bytes(data[1:])]
        if len(self.rx_data) >= self.rx_data_size:
            self.notify()

    def readFlowControlFrame(self, data):
        self.st_min_for_tx = (data[2] / 10e3) if (data[2] <= 0x7F) else (self.st_min_for_tx + (data[2] / 10e6))
        self.blk_size_for_tx = data[1] if data[1] else self.max_blk_size
        self.flow_ctrl_ok.set()

    ## -- Write Data --
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
        self.flow_ctrl_ok.clear()
        # send first frame
        data = self.writeFirstFrame(data)
        data_len = len(data)
        timeout = 0

        while data_len:
            # wait for ctrl flow msg
            timeout = 0 if self.flow_ctrl_ok.wait(0.01) else (timeout+1)

            if timeout == 0:
                # send consecutive frame
                data = self.writeConsecutiveFrame(data)
                data_len = len(data)
                time.sleep(self.st_min_for_tx)
                block_count += 1
                # all blocks sent, wait for ctrl flow again
                if block_count % self.blk_size_for_tx == 0:
                    self.flow_ctrl_ok.clear()
                    block_count = 0

            elif timeout > 10:
                print("CANTP::writeMultiFrame : flow ctrl timeout")
                break

    # -- API for writing data --
    def sendData(self, data):
        if len(data) < 8:
            self.writeSingleFrame(data)
        else:
            th = Thread(target=self.writeMultiFrame(data))
            th.start()
            th.join() # remove in production env


# ------------------- LISTENER ------------------- #
class DiagRx(CANTP.Observer):
    def on_cantp_msg_received(self, data):
        # print("notf : " + " ".join([hex(byte) for byte in data]))
        pass


# ------------------- TESTING ------------------- #
rx = DiagRx(    )

bus2 = can.Bus('test', bustype='virtual')
tp2 = CANTP(bus2, 0x72F, 0x727)
can.Notifier(bus2, [tp2])
tp2.addObserver(rx)

bus1 = can.Bus('test', bustype='virtual')
tp1 = CANTP(bus1, 0x727, 0x72F)
can.Notifier(bus1, [tp1])
tp1.addObserver(rx)

data = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F]
tp1.sendData(data)
data = [0x0a, 0x0b, 0x0c]
tp1.sendData(data)

data = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F]
tp2.sendData(data)
data = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
tp2.sendData(data)

while True:
    time.sleep(1)
