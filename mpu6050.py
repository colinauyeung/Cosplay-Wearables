"""This program handles the communication over I2C
between a Raspberry Pi and a MPU-6050 Gyroscope / Accelerometer combo.
Made by: MrTijn/Tijndagamer
Released under the MIT License
Copyright (c) 2015, 2016, 2017 MrTijn/Tijndagamer
"""

"""
with Modifications by JE Boyd
Sept 2021
Oct 2021
Jan 2022 - changes for Micropython/Pico RP2040/other improvements
Oct 2022 - converted to PiOS smbus device
"""

import smbus
import time
import imufusion
import numpy as np
import csv
import RPi.GPIO as GPIO

import board
import neopixel



class MPU6050:

    # Global Variables
    GRAVITIY_MS2 = 9.80665

    # Scale Modifiers
    #Lower numbers give higher precision, but if you go over the rating, the device will saturate
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0


    #Lower numbers give higher precision, but if you go over the rating, the device will saturate
    #Degrees per second
    GYRO_SCALE_MODIFIER_250DEG = 131.0
    GYRO_SCALE_MODIFIER_500DEG = 65.5
    GYRO_SCALE_MODIFIER_1000DEG = 32.8
    GYRO_SCALE_MODIFIER_2000DEG = 16.4

    # Pre-defined ranges
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18

    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    # MPU-6050 Registers
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F

    TEMP_OUT0 = 0x41

    GYRO_XOUT0 = 0x43
    GYRO_YOUT0 = 0x45
    GYRO_ZOUT0 = 0x47
    
    WHO_AM_I = 0x75

    CONFIG = 0x1A
    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B

    def __init__(self, address, bus=1 ):
        self.address = address
        self.bus = smbus.SMBus(bus)
        
        # verify with WHO_AM_I
        id = self.bus.read_byte_data( self.address,self.WHO_AM_I )
        if id != address:
            print( 'MPU6050 not found at 0x{:02x}'.format(self.address) )
        else :
            print( 'MPU6050 found at 0x{:02x}'.format(self.address) )
        
        # Wake up the MPU-6050 since it starts in sleep mode
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)

        
        # configure digital low-pass filter (DLPF): dlpf_cfg = 3 --> accel 44Hz, gyro 42Hz
        self.writeDLPF( 4 )
        
        # initialize gyro and accelerometer ranges
        self.writeGyroRange( self.GYRO_RANGE_250DEG )
        self.writeAccelRange( self.ACCEL_RANGE_2G )

        time.sleep(0.5)



    # MPU-6050 Methods

    def readTemperature(self) :
        """
        Reads the temperature from the onboard temperature sensor of the MPU-6050.
             -Returns the temperature in degrees Celcius.
        """
        data = self.bus.read_i2c_block_data( self.address, self.TEMP_OUT0, 2 )
        raw_temp = self._pack16( data )

        # Get the actual temperature using the formule given in the
        # MPU-6050 Register Map and Descriptions revision 4.2, page 30
        actual_temp = (raw_temp / 340.0) + 36.53

        return actual_temp
    
    def writeDLPF( self, dlpf_cfg ) :
        """
        Write bits to set DLPF in CONFIG register from
        MPU6050 register map descriptions
        
        
        DLPF_CFG          Accelerometer               Gyroscope
                            (Fs = 1kHz)
                    ----------------------------------------------------------
                     Bandwidth      Delay        Bandwidth   Delay    Fs (kHz)
                     (Hz)           (ms)         (Hz)        (ms)
        ----------------------------------------------------------------------
           0         260               0          256          0.98     8
           1         184             2.0          188          1.9      1
           2          94             3.0           98          2.8      1
           3          44             4.9           42          4.8      1
           4          21             8.5           20          8.3      1
           5          10            13.8           10         13.4      1
           6           5            19.0            5         18.6      1
        ----------------------------------------------------------------------
           7                 RESERVED                      RESERVED     8
        """
        r = self.bus.read_byte_data( self.address, self.CONFIG )
        r &= 0xf8        # zero DLPF bits                                                                 
        r |= dlpf_cfg    # set dlpf_cfg bits
        self.bus.write_byte_data( self.address, self.CONFIG, r )

    def readDLPF( self ) :
        """
        Read DLPF bits in CONFIG register - see writeDLPF above for interpretation
        of bit patterns.
        """
        return self.bus.read_byte_data( self.address, self.CONFIG ) & 0x07
        

    def writeAccelRange(self, accel_range):
        """
        Sets the range of the accelerometer to range.
        """

        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data( self.address, self.ACCEL_CONFIG, accel_range )
        
        if accel_range == self.ACCEL_RANGE_2G:
            self.accelScaleModifier = self.ACCEL_SCALE_MODIFIER_2G
        elif accel_range == self.ACCEL_RANGE_4G:
            self.accelScaleModifier = self.ACCEL_SCALE_MODIFIER_4G
        elif accel_range == self.ACCEL_RANGE_8G:
            self.accelScaleModifier = self.ACCEL_SCALE_MODIFIER_8G
        elif accel_range == self.ACCEL_RANGE_16G:
            self.accelScaleModifier = self.ACCEL_SCALE_MODIFIER_16G
        else:
            print("Unkown range - accel_scale_modifier set to self.ACCEL_SCALE_MODIFIER_2G")
            self.accelScaleModifier = self.ACCEL_SCALE_MODIFIER_2G
        self.accelScaleModifier = 1.0 / self.accelScaleModifier
        time.sleep( 0.001 )

    def readAccelRange(self, raw = False):
        """Reads the range the accelerometer is set to.

        If raw is True, it will return the raw value from the ACCEL_CONFIG
        register
        If raw is False, it will return an integer: -1, 2, 4, 8 or 16. When it
        returns -1 something went wrong.
        """
        raw_data = self.bus.read_byte_data( self.address, self.ACCEL_CONFIG )

        if raw :
            return raw_data
        else :
            raw_data = raw_data & 0x18
            if raw_data == self.ACCEL_RANGE_2G:
                return 2
            elif raw_data == self.ACCEL_RANGE_4G:
                return 4
            elif raw_data == self.ACCEL_RANGE_8G:
                return 8
            elif raw_data == self.ACCEL_RANGE_16G:
                return 16
            else:
                return -1


    def readAccelData( self, g = True ):
        """Gets and returns the X, Y and Z values from the accelerometer.

        If g is True, it will return the data in g
        If g is False, it will return the data in m/s^2
        Returns a dictionary with the measurement results.
        """
        data = self.bus.read_i2c_block_data( self.address, self.ACCEL_XOUT0, 6 )
        x = self._pack16( data[0:2] )
        y = self._pack16( data[2:4] )
        z = self._pack16( data[4:6] )

        x = x * self.accelScaleModifier
        y = y * self.accelScaleModifier
        z = z * self.accelScaleModifier

        if g :
            return ( x, y, z )
        else :
            return (    x * self.GRAVITIY_MS2,
                        y * self.GRAVITIY_MS2,
                        z * self.GRAVITIY_MS2 )

    def writeGyroRange(self, gyro_range):
        """
        Sets the range of the gyroscope to range.
        """
        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data( self.address, self.GYRO_CONFIG, gyro_range )
        
        if gyro_range == self.GYRO_RANGE_250DEG:
            self.gyroScaleModifier = self.GYRO_SCALE_MODIFIER_250DEG
        elif gyro_range == self.GYRO_RANGE_500DEG:
            self.gyroScaleModifier = self.GYRO_SCALE_MODIFIER_500DEG
        elif gyro_range == self.GYRO_RANGE_1000DEG:
            self.gyroScaleModifier = self.GYRO_SCALE_MODIFIER_1000DEG
        elif gyro_range == self.GYRO_RANGE_2000DEG:
            self.gyroScaleModifier = self.GYRO_SCALE_MODIFIER_2000DEG
        else:
            print("Unkown range - gyro_scale_modifier set to self.GYRO_SCALE_MODIFIER_250DEG")
            self.gyroScaleModifier = self.GYRO_SCALE_MODIFIER_250DEG
            
        self.gyroScaleModifier = 1.0 / self.gyroScaleModifier
        time.sleep( 0.001 )


    def readGyroRange(self, raw = False):
        """Reads the range the gyroscope is set to.

        If raw is True, it will return the raw value from the GYRO_CONFIG
        register.
        If raw is False, it will return 250, 500, 1000, 2000 or -1. If the
        returned value is equal to -1 something went wrong.
        """
        raw_data = self.bus.read_byte_data( self.address, self.GYRO_CONFIG )

        if raw :
            return raw_data
        else:
            raw_data = raw_data & 0x18
            if raw_data == self.GYRO_RANGE_250DEG:
                return 250
            elif raw_data == self.GYRO_RANGE_500DEG:
                return 500
            elif raw_data == self.GYRO_RANGE_1000DEG:
                return 1000
            elif raw_data == self.GYRO_RANGE_2000DEG:
                return 2000
            else:
                return -1

    def readGyroData(self):
        """Gets and returns the X, Y and Z values from the gyroscope.

        Returns the read values in a dictionary.
        """
        data = self.bus.read_i2c_block_data( self.address, self.GYRO_XOUT0, 6 )
        x = self._pack16( data[0:2] )
        y = self._pack16( data[2:4] )
        z = self._pack16( data[4:6] )

        x = x * self.gyroScaleModifier
        y = y * self.gyroScaleModifier
        z = z * self.gyroScaleModifier

        return ( x, y, z )


    def _pack16( self, data ) :
        # pack bytes into 16  bits
        value = (data[0] << 8) + data[1]
        # extend sign bit to  get signed integer
        return ((value & 0x0000ffff) ^ 0x00008000) - 0x00008000
        
        
    def readAllData( self ) :
        """read all data registers in one transfer

        returns tuple with ax, ay, az, gx, gy, gz, temperature
        """
        data = self.bus.read_i2c_block_data( self.address, self.ACCEL_XOUT0, 14 )
        
        ax = self._pack16( data[ 0: 2] ) * self.accelScaleModifier
        ay = self._pack16( data[ 2: 4] ) * self.accelScaleModifier
        az = self._pack16( data[ 4: 6] ) * self.accelScaleModifier
        
        temperature = ( self._pack16( data[ 6: 8] ) / 340.0 ) + 36.53

        gx = self._pack16( data[ 8:10] ) * self.gyroScaleModifier
        gy = self._pack16( data[10:12] ) * self.gyroScaleModifier
        gz = self._pack16( data[12:14] ) * self.gyroScaleModifier
        
        return ax,ay,az,gx,gy,gz,temperature
        

if __name__ == "__main__":
    # test module
    # GPIO.setmode(GPIO.BOARD)
    # GPIO.setup(7,GPIO.OUT)
    # GPIO.output(7, True)
    mpu = MPU6050(0x68)
    mpu2 = MPU6050(0x69)
    
    print( "DLPF setting: 0x{:02x}".format( mpu.readDLPF() ) )
    
    print( "temperature = {:5.1f} degrees Celcius".format( mpu.readTemperature() ) )
    
    print( "Accel range: {} (0x{:02x})".format( mpu.readAccelRange(), mpu.readAccelRange(raw=True) ) )
    ax, ay, az = mpu.readAccelData()
    print( "ax: {:5.2f}  az: {:5.2f}  az: {:5.2f}".format( ax, ay, az ) )
    ax, ay, az = mpu.readAccelData( g = False )
    print( "ax: {:5.2f}  az: {:5.2f}  az: {:5.2f}".format( ax, ay, az ) )
    
    mpu.writeAccelRange( mpu.ACCEL_RANGE_16G )
    print( "Accel range: {} (0x{:02x})".format( mpu.readAccelRange(), mpu.readAccelRange(raw=True) ) )
    ax, ay, az = mpu.readAccelData()
    print( "ax: {:5.2f}  az: {:5.2f}  az: {:5.2f}".format( ax, ay, az ) )
    ax, ay, az = mpu.readAccelData( g = False )
    print( "ax: {:5.2f}  az: {:5.2f}  az: {:5.2f}".format( ax, ay, az ) )
    
    print( "Gyro range: {} (0x{:02x})".format( mpu.readGyroRange(), mpu.readGyroRange(raw=True) ) )
    gx, gy, gz = mpu.readGyroData()
    print( "gx: {:8.2f}  gy: {:8.2f}  gz: {:8.2f}".format( gx, gy, gz ) )

    mpu.writeGyroRange( mpu.GYRO_RANGE_2000DEG )
    print( "Gyro range: {} (0x{:02x})".format( mpu.readGyroRange(), mpu.readGyroRange(raw=True) ) )
    gx, gy, gz = mpu.readGyroData()
    print( "gx: {:8.2f}  gy: {:8.2f}  gz: {:8.2f}".format( gx, gy, gz ) )
    
    #reads all the measurement registers at once (as opposed to seperately) to ensure that they all come from the same point in time
    ax, ay, az, gx, gy, gz, temperature = mpu.readAllData()
    print( "ax: {:5.2f}  az: {:5.2f}  az: {:5.2f} gx: {:8.2f}  gy: {:8.2f}  gz: {:8.2f}  T: {:5.1f}".
           format( ax, ay, az, gx, gy, gz, temperature ) )

    # plot measurements


    pixels = neopixel.NeoPixel(board.D10, 6)

	
    pixels[0] = (255, 255, 0)
    pixels[1] = (255, 255, 0)
    pixels[2] = (255, 255, 0)
    pixels[3] = (255, 255, 0)
    pixels[4] = (255, 255, 0)
    pixels[5] = (255, 255, 0)

    ahrs = imufusion.Ahrs()
    sample_rate = 40
    ahrs.settings = imufusion.Settings(0.5,  # gain
                                    10,  # acceleration rejection
                                    20,  # magnetic rejection
                                    5 * sample_rate)  # rejection timeout = 5 seconds

    ahrs2  = imufusion.Ahrs()
    ahrs2.settings = imufusion.Settings(0.5,  # gain
                                    10,  # acceleration rejection
                                    20,  # magnetic rejection
                                    5 * sample_rate)  # rejection timeout = 5 seconds



    with open("test22.csv", mode="w") as file:
        writer = csv.writer(file)
        t1 = time.perf_counter()
        time.sleep( 0.005 )
        for i in range(500):
            t2 = time.perf_counter()
            ax, ay, az, gx, gy, gz, temp = mpu.readAllData()
            ahrs.update_no_magnetometer(np.array([gx, gy, gz]), np.array([ax,ay,az]), 1 / sample_rate )  # 100 Hz sample rate
        
            euler = ahrs.quaternion.to_euler()
            # print(ahrs.quaternion.w, ahrs.quaternion.x ,ahrs.quaternion.y ,ahrs.quaternion.z)
            # print( "gx:{:0.2f} gy:{:0.2f} gz:{:0.2f}".format( euler[0], euler[1], euler[2] ) )

            ax2, ay2, az2, gx2, gy2, gz2, temp = mpu2.readAllData()
            ahrs2.update_no_magnetometer(np.array([gx2, gy2, gz2]), np.array([ax2,ay2,az2]), 1 / sample_rate )  # 100 Hz sample rate
        
            euler2 = ahrs2.quaternion.to_euler()
            # print( "gx2:{:0.2f} gy2:{:0.2f} gz2:{:0.2f}".format( euler2[0], euler2[1], euler2[2] ) )
            print(euler[1], euler2[1])
            writer.writerow([euler[1], euler2[1], i])
            # print( 1 / (t2 - t1) )
            t1 = t2
            print(max( 0, 1 / sample_rate  - (time.perf_counter() - t2) ) )
            time.sleep( max( 0, 1 / sample_rate  - (time.perf_counter() - t2) ) )
    

    pixels[0] = (255, 0, 0)
    pixels[1] = (255, 0, 0)
    pixels[2] = (255, 0, 0)
    pixels[3] = (255, 0, 0)
    pixels[4] = (255, 0, 0)
    pixels[5] = (255, 0, 0)


#     t1 = time.perf_counter_ns()
#     time.sleep( 0.005 )
#     for i in range( 20 ) :
#         t2 = time.perf_counter()
#         ax, ay, az, gx, gy, gz, temp = mpu.get_all_data()
#         print( "Accel: {:6.2f} {:6.2f} {:6.2f}".format( ax, ay, az ) )
#         print( "Gyro:  {:6.2f} {:6.2f} {:6.2f}".format( gx, gy, gz ) )
#         print( "Temperature: {:6.2f}".format( temp ) )
#         print( "Fs:", 1 / (t2 - t1) )
#         t1 = t2
#         time.sleep( max( 0, 0.00985 - (time.perf_counter() - t2) ) )
# #         time.sleep( 0.01 )
