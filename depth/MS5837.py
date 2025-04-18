import time

class MS5837:
    def __init__(self, i2c):
        self.__fluidDensity = 1029
        self.i2c = i2c
        self.Pa = 100.0
        self.bar = 0.001
        self.mbar = 1.0
        self.MS5837_30BA = 0
        self.MS5837_02BA = 1
        self.MS5837_ADDR = 0x76
        self.MS5837_RESET = 0x1E
        self.MS5837_ADC_READ = 0x00
        self.MS5837_PROM_READ = 0xA0
        self.MS5837_CONVERT_D1_8192 = 0x4A
        self.MS5837_CONVERT_D2_8192 = 0x5A
        
        self.C = [0, 0, 0, 0, 0, 0, 0, 0]
        
    def init(self):
        buf = bytearray(1)
        buf[0] = self.MS5837_RESET
        self.i2c.writeto(self.MS5837_ADDR, buf)
        time.sleep(0.01)
        for i in range(7):
            #data = self.i2c.readfrom(self.MS5837_ADDR, 2)
            data = self.i2c.readfrom_mem(self.MS5837_ADDR, self.MS5837_PROM_READ+i*2, 2)
            l = list(data)
            #print(l)
            self.C[i] = (l[0] << 8) | l[1]
        crcRead = self.C[0] >> 12
        crcCalculated = self.crc4(self.C)
        if crcCalculated == crcRead:
            return True
        return False
    
    def setModel(self, model):
        self.__model = model
    
    def setFluidDensity(self, density):
        self.__fluidDensity = density
    
    def read(self):
        buf = bytearray(1)
        buf[0] = self.MS5837_CONVERT_D1_8192
        self.i2c.writeto(self.MS5837_ADDR, buf)
        time.sleep(0.02)
        data = self.i2c.readfrom_mem(self.MS5837_ADDR, self.MS5837_ADC_READ, 3)
        l = list(data)
        self.d1 = 0
        self.d1 = l[0]
        self.d1 = (self.d1 << 8) | l[1]
        self.d1 = (self.d1 << 8) | l[2]
        
        buf[0] = self.MS5837_CONVERT_D2_8192
        self.i2c.writeto(self.MS5837_ADDR, buf)
        time.sleep(0.02)
        data = self.i2c.readfrom_mem(self.MS5837_ADDR, self.MS5837_ADC_READ, 3)
        l = list(data)
        self.d2 = 0
        self.d2 = l[0]
        self.d2 = (self.d2 << 8) | l[1]
        self.d2 = (self.d2 << 8) | l[2]
        self.__calculate()

    def pressure(self, conversion = 1.0):
        if self.__model == self.MS5837_02BA:
            return self.P * conversion / 100.0
        else:
            return self.P * conversion / 10.0
        
    def temperature(self):
        return self.TEMP / 100.0
    
    def depth(self):
        return (self.pressure(self.Pa) - 101300) / (self.__fluidDensity * 9.80665)
    
    def altitude(self):
        return (1 - pow((self.pressure() / 1013.25), 0.190284)) * 145366.45 * 0.3048
    
    def __calculate(self):
        dT = 0
        SENS = 0
        OFF = 0
        SENSi = 0
        OFFi = 0
        Ti = 0
        OFF2 = 0
        SENS2 = 0
        
        dT = self.d2 - int(self.C[5]) * 256
        if self.__model == self.MS5837_02BA:
            SENS = int(self.C[1]) * 65536 + (int(self.C[3]) * dT) / 128
            OFF = int(self.C[2]) * 131072 + (int(self.C[4]) * dT) / 64
            self.P = (self.d1 * SENS / (2097152) - OFF) / (32768)
        else:
            SENS = int(self.C[1]) * 32768 + (int(self.C[3]) * dT) / 256
            OFF = int(self.C[2]) * 65536 + (int(self.C[4]) * dT) / 128
            self.P = (self.d1 * SENS / (2097152) - OFF) / (8192)
            
        self.TEMP = 2000 + int(dT) * self.C[6] / 8388608
        
        if self.__model == self.MS5837_02BA:
            if (self.TEMP / 100) < 20:
                Ti = (11 * int(dT) * int(dT)) / (34359738368)
                OFFi = (31 * (self.TEMP - 2000) * (self.TEMP - 2000)) / 8
                SENSi = (63 * (self.TEMP -2000) * (self.TEMP - 2000)) / 32
        else:
            if (self.TEMP / 100) < 20:
                Ti = (3 * int(dT) * int(dT)) / (8589934592)
                OFFi = (3 * (self.TEMP - 2000) * (self.TEMP - 2000)) / 2
                SENSi = (5 * (self.TEMP - 2000) * (self.TEMP - 2000)) / 8
                if (self.TEMP / 100) < -15:
                    OFFi = OFFi + 7 * (self.TEMP + 1500) * (self.TEMP + 1500)
                    SENSi = SENSi + 4 * (self.TEMP + 1500) * (self.TEMP + 1500)
            elif (self.TEMP / 100) >= 20:
                Ti = 2 * (dT * dT) / (137438953472)
                OFFi = (1 * (self.TEMP - 2000) * (self.TEMP - 2000)) / 16
                SENSi = 0
            
        OFF2 = OFF - OFFi
        SENS2 = SENS - SENSi
        
        self.TEMP = (self.TEMP - Ti)
        if self.__model == self.MS5837_02BA:
            self.P = (((self.d1 * SENS2) / 2097152 - OFF2) / 32768)
        else:
            self.P = (((self.d1 * SENS2) / 2097152 - OFF2) / 8192)
            
    def crc4(self, n_prom):
        n_rem = 0
        
        n_prom[0] = ((n_prom[0]) & 0x0FFF)
        n_prom[7] = 0
    
        for i in range(16):
            if i%2 == 1:
                n_rem ^= ((n_prom[i>>1]) & 0x00FF)
            else:
                n_rem ^= (n_prom[i>>1] >> 8)
                
            for n_bit in range(8,0,-1):
                if n_rem & 0x8000:
                    n_rem = (n_rem << 1) ^ 0x3000
                else:
                    n_rem = (n_rem << 1)

        n_rem = ((n_rem >> 12) & 0x000F)
        
        self.n_prom = n_prom
        self.n_rem = n_rem
    
        return n_rem ^ 0x00

