import time

class CANTP:
    def singleFrame(self, data):
        data_len = len(data)
        msg = [data_len] + data
        msg += [0x00 for i in range(8-len(msg))]
        self.sendMessage(msg)

    def firstFrame(self, data):
        self.seq = 0
        self.st_min = 0.1
        self.ctrl_frame = True
        self.timeout = False
        
        data_len = len(data)
        last_idx = 6
        msg = [0x10 | ((data_len & 0xF00) >> 8)] + [(data_len & 0xFF)] + data[:last_idx]
        self.sendMessage(msg)
        return data[last_idx:]

    def consecutiveFrame(self, data):
        data_len = len(data)
        last_idx = (data_len % 8) if (data_len < 7) else 7
        self.seq = (self.seq + 1) % 16
        msg = [0x20 | self.seq] + data[:last_idx]
        msg += [0x00 for i in range(8-len(msg))]
        self.sendMessage(msg)
        return data[last_idx:]

    def multiFrame(self, data):
        data = self.firstFrame(data)
        while (self.ctrl_frame == False) or (self.timeout == True):
            time.sleep(0.1)

        if not self.timeout:
            data_len = len(data)
            while data_len:
                time.sleep(self.st_min)
                data = self.consecutiveFrame(data)
                data_len = len(data)

    def sendData(self, data):
        if len(data) < 8:
            self.singleFrame(data)
        else:
            self.multiFrame(data)

    def sendMessage(self, msg):
        # msg = bytes(msg)
        print([hex(i) for i in msg])


tp = CANTP()

data = [0x10, 0x01]
tp.sendData(data)

data = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F]
tp.sendData(data)
