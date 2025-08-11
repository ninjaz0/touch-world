import machine

class accel():
    def __init__(self, i2c, addr=0x68):
        self.iic = i2c
        self.addr = addr
        self.iic.start()
        self.iic.writeto(self.addr, bytearray([107, 0]))
        self.iic.stop()

    def get_raw_values(self):
        self.iic.start()
        a = self.iic.readfrom_mem(self.addr, 0x3B, 14)
        self.iic.stop()
        return a
    
    def get_config(self):
        self.iic.start()
        conf = self.iic.readfrom_mem(self.addr, 0x1B, 2)
        self.iic.stop()
        return conf

    def get_ints(self):
        b = self.get_raw_values()
        c = []
        for i in b:
            c.append(i)
        return c

    def bytes_toint(self, firstbyte, secondbyte):
        if not firstbyte & 0x80:
            return firstbyte << 8 | secondbyte
        return - (((firstbyte ^ 255) << 8) | (secondbyte ^ 255) + 1)

    def get_values(self):
        raw_ints = self.get_raw_values()
        vals = {}
        vals["AcX"] = self.bytes_toint(raw_ints[0], raw_ints[1])
        vals["AcY"] = self.bytes_toint(raw_ints[2], raw_ints[3])
        vals["AcZ"] = self.bytes_toint(raw_ints[4], raw_ints[5])
        vals["Tmp"] = self.bytes_toint(raw_ints[6], raw_ints[7]) / 340.00 + 36.53
        vals["GyX"] = self.bytes_toint(raw_ints[8], raw_ints[9])
        vals["GyY"] = self.bytes_toint(raw_ints[10], raw_ints[11])
        vals["GyZ"] = self.bytes_toint(raw_ints[12], raw_ints[13])
        return vals  # returned in range of Int16
        # -32768 to 32767

    def get_dps(self):
        b = self.get_config()
        raw_ints = self.get_raw_values()
        c = []
        dps = {}
        for i in b:
            c.append(i)
        if c[0] == 0:
            dps["GyX"] = self.bytes_toint(raw_ints[8], raw_ints[9]) / 131
            dps["GyY"] = self.bytes_toint(raw_ints[10], raw_ints[11]) / 131
            dps["GyZ"] = self.bytes_toint(raw_ints[12], raw_ints[13]) / 131
        if c[0] == 1:
            dps["GyX"] = self.bytes_toint(raw_ints[8], raw_ints[9]) / 65.5
            dps["GyY"] = self.bytes_toint(raw_ints[10], raw_ints[11]) / 65.5
            dps["GyZ"] = self.bytes_toint(raw_ints[12], raw_ints[13]) / 65.5
        if c[0] == 2:
            dps["GyX"] = self.bytes_toint(raw_ints[8], raw_ints[9]) / 32.8
            dps["GyY"] = self.bytes_toint(raw_ints[10], raw_ints[11]) / 32.8
            dps["GyZ"] = self.bytes_toint(raw_ints[12], raw_ints[13]) / 32.8
        if c[0] == 3:
            dps["GyX"] = self.bytes_toint(raw_ints[8], raw_ints[9]) / 16.4
            dps["GyY"] = self.bytes_toint(raw_ints[10], raw_ints[11]) / 16.4
            dps["GyZ"] = self.bytes_toint(raw_ints[12], raw_ints[13]) / 16.4
        return dps
    
    def get_mss(self):
        g = 9.8
        b = self.get_config()
        raw_ints = self.get_raw_values()
        c = []
        mss = {}
        for i in b:
            c.append(i)
        if c[1] == 0:
            mss["AcX"] = self.bytes_toint(raw_ints[0], raw_ints[1]) * g / 16384
            mss["AcY"] = self.bytes_toint(raw_ints[2], raw_ints[3]) * g / 16384
            mss["AcZ"] = self.bytes_toint(raw_ints[4], raw_ints[5]) * g / 16384
        if c[1] == 1:
            mss["AcX"] = self.bytes_toint(raw_ints[0], raw_ints[1]) * g / 8192
            mss["AcY"] = self.bytes_toint(raw_ints[2], raw_ints[3]) * g / 8192
            mss["AcZ"] = self.bytes_toint(raw_ints[4], raw_ints[5]) * g / 8192
        if c[1] == 2:
            mss["AcX"] = self.bytes_toint(raw_ints[0], raw_ints[1]) * g / 4096
            mss["AcY"] = self.bytes_toint(raw_ints[2], raw_ints[3]) * g / 4096
            mss["AcZ"] = self.bytes_toint(raw_ints[4], raw_ints[5]) * g / 4096
        if c[1] == 3:
            mss["AcX"] = self.bytes_toint(raw_ints[0], raw_ints[1]) * g / 2048
            mss["AcY"] = self.bytes_toint(raw_ints[2], raw_ints[3]) * g / 2048
            mss["AcZ"] = self.bytes_toint(raw_ints[4], raw_ints[5]) * g / 2048
        return mss
            
    def val_test(self):  # ONLY FOR TESTING! Also, fast reading sometimes crashes IIC
        from time import sleep
        while 1:
            print(self.get_values())
            sleep(0.05)
