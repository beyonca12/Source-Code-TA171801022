from Crypto.Cipher import AES
import math
import serial
import hashlib
from math import sqrt
from math import atan2
from math import sin
from math import cos
import smbus
import time
import datetime
from time import sleep
import RPi.GPIO as GPIO
import Adafruit_ADS1x15

port = serial.Serial("/dev/ttyAMA0", 38400)
port.close()
port.open()

arduino	=	serial.Serial("/dev/ttyACM0", 38400)

# select the correct i2c bus for this revision of Raspberry Pi
revision = ([l[12:-1] for l in open('/proc/cpuinfo','r').readlines() if l[:8]=="Revision"]+['0000'])[0]
bus = smbus.SMBus(1 if int(revision, 16) >= 4 else 0)

BLOCK_SIZE	=	64
key = b'\xbf\xc0\x85)\x10nc\x94\x02)j\xdf\xcb\xc4\x94\x9d(\x9e[EX\xc8\xd5\xbfI{\xa2$\x05(\xd5\x18'
cipher = AES.new(key)

# ADXL345 constants
EARTH_GRAVITY_MS2   = 9.80665
SCALE_MULTIPLIER    = 0.004

DATA_FORMAT         = 0x31
BW_RATE             = 0x2C
POWER_CTL           = 0x2D

BW_RATE_1600HZ      = 0x0F
BW_RATE_800HZ       = 0x0E
BW_RATE_400HZ       = 0x0D
BW_RATE_200HZ       = 0x0C
BW_RATE_100HZ       = 0x0B
BW_RATE_50HZ        = 0x0A
BW_RATE_25HZ        = 0x09

RANGE_2G            = 0x00
RANGE_4G            = 0x01
RANGE_8G            = 0x02
RANGE_16G           = 0x03

MEASURE             = 0x08
AXES_DATA           = 0x32

#Koefisien Complentary Filter
AA =  0.90

G_GAIN = 0.070
# HMC5883L constants
addrem 				= 0x1e
scale 				= 0.92

x_offset 			= -32
y_offset 			= -186

#PIN LDR
pin1 				= 11
pin2 				= 7



#HMC5883L Function & Setting
def read_byte(adr):
    return bus.read_byte_data(addrem, adr)

def read_word(adr):
    high = bus.read_byte_data(addrem, adr)
    low = bus.read_byte_data(addrem, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def write_byte(adr, value):
    bus.write_byte_data(addrem, adr, value)

write_byte(0, 0b01110000) 
write_byte(1, 0b00100000)
write_byte(2, 0b00000000)

#L3G4200D
bus.write_byte_data(0x69, 0x20, 0x0F)
bus.write_byte_data(0x69, 0x23, 0x30)

#LDR
adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1
	
class ADXL345:

    address = None

    def __init__(self, address = 0x53):        
        self.address = address
        self.setBandwidthRate(BW_RATE_100HZ)
        self.setRange(RANGE_2G)
        self.enableMeasurement()

    def enableMeasurement(self):
        bus.write_byte_data(self.address, POWER_CTL, MEASURE)

    def setBandwidthRate(self, rate_flag):
        bus.write_byte_data(self.address, BW_RATE, rate_flag)

    # set the measurement range for 10-bit readings
    def setRange(self, range_flag):
        value = bus.read_byte_data(self.address, DATA_FORMAT)

        value &= ~0x0F;
        value |= range_flag;  
        value |= 0x08;

        bus.write_byte_data(self.address, DATA_FORMAT, value)
    
    # returns the current reading from the sensor for each axis
    #
    # parameter gforce:
    #    False (default): result is returned in m/s^2
    #    True           : result is returned in gs
    def getAxes(self, gforce = False):
        bytes = bus.read_i2c_block_data(self.address, AXES_DATA, 6)
        
        a = bytes[0] | (bytes[1] << 8)
        if(a & (1 << 16 - 1)):
            a = a - (1<<16)

        b = bytes[2] | (bytes[3] << 8)
        if(b & (1 << 16 - 1)):
            b = b - (1<<16)

        c = bytes[4] | (bytes[5] << 8)
        if(c & (1 << 16 - 1)):
            c = c - (1<<16)

        a = a * SCALE_MULTIPLIER 
        b = b * SCALE_MULTIPLIER
        c = c * SCALE_MULTIPLIER

        if gforce == False:
            a = a * EARTH_GRAVITY_MS2
            b = b * EARTH_GRAVITY_MS2
            c = c * EARTH_GRAVITY_MS2

        x = round(a, 4)
        y = round(b, 4)
        z = round(c, 4)
        return {"x": x, "y": y, "z": z}

def pad(s):
    return s + ((BLOCK_SIZE - len(s)%BLOCK_SIZE)*'a')
    
def encrypt(plaintext):
    global cipher
    return cipher.encrypt(pad(plaintext))
    
def decrypt (ciphertext):
    global cipher
    dec = cipher.decrypt(ciphertext).decode('ISO-8859-1')
    l = dec.count('a')
    return dec[:len(dec)-l]

def Magnetic(state):
	arduino.flushInput()
	arduino.flushOutput()
	# if run directly we'll just create an instance of the class and output 
	# the current readings
	
	#Inisalisasi Parameter Filterfx = 0
	fx = 0
	fy = 0
	fz = 0
	mx = 0
	my = 0
	mz = 0
		
	#Inisialisasi Parameter HPF Gyro
	rate_gyr_x = 0
	rate_gyr_y = 0
	rate_gyr_z = 0
	ogx = 0
	ogy = 0
	ogz = 0
	nroll = 0
	npitch = 0
	nyaw = 0
	last_time = 0
	counter = 10
	count = 10
	
	a = datetime.datetime.now()

	if (state==0): 	#light
		if (port.inWaiting()>0):
			port.flushInput()
			time.sleep(0.3)
			message	= str(0)+"a"+"x"+str(0)+"b"+str(0)+"c"+"0"
			arduino.write(message+"\0")
			time.sleep(0.3)
			message	= str(0)+"a"+"x"+str(0)+"b"+str(0)+"c"+"1"
			arduino.write(message+"\0")
			time.sleep(0.3)
			message	= str(0)+"a"+"x"+str(0)+"b"+str(0)+"c"+"0"
			arduino.write(message+"\0")
			
			print "stopping..."
			main_function()
		port.flushInput()
		arduino.flushInput()
		port.flushOutput()
		arduino.flushOutput()
		
		adxl345 = ADXL345()
		
		#Raw Data Magnetic, Gyrometer, and Accelerometer 
		# L3G4200D address, 0x68(104)
		# Read data back from 0x28(40), 2 bytes, X-Axis LSB first
		data0 = bus.read_byte_data(0x69, 0x28)
		data1 = bus.read_byte_data(0x69, 0x29)

		# Convert the data
		xGyro = data1 * 256 + data0
		if xGyro > 32767 :
			xGyro -= 65536

		# L3G4200D address, 0x68(104)
		# Read data back from 0x2A(42), 2 bytes, Y-Axis LSB first
		data0 = bus.read_byte_data(0x69, 0x2A)
		data1 = bus.read_byte_data(0x69, 0x2B)

		# Convert the data
		yGyro = data1 * 256 + data0
		if yGyro > 32767 :
			yGyro -= 65536

		# L3G4200D address, 0x68(104)
		# Read data back from 0x2C(44), 2 bytes, Z-Axis LSB first
		data0 = bus.read_byte_data(0x69, 0x2C)
		data1 = bus.read_byte_data(0x69, 0x2D)

		# Convert the data
		zGyro = data1 * 256 + data0
		if zGyro > 32767 :
			zGyro -= 65536
		xm = (read_word_2c(3) - x_offset) * scale
   		ym = (read_word_2c(7) - y_offset) * scale
   		zm = (read_word_2c(5)) * scale
		axes = adxl345.getAxes(True)
		
		#Read Sensor Time
		b = datetime.datetime.now() - a
		a = datetime.datetime.now()
		LP = b.microseconds/(1000000*1.0)
		#print "Loop Time | %5.2f|" % ( LP )
		
		#Gain Gyrometer
		rategx =  xGyro * G_GAIN
		rategy =  yGyro * G_GAIN
		rategz =  zGyro * G_GAIN
		
		#HPF Gyro
		rate_gyr_x = 0.4*rate_gyr_x+0.4*(rategx-ogx)
		rate_gyr_y = 0.4*rate_gyr_y+0.4*(rategy-ogy)
		rate_gyr_z = 0.4*rate_gyr_z+0.4*(rategz-ogz)
		
		ogx = rate_gyr_x
		ogy = rate_gyr_y
		ogz = rate_gyr_z
		
		#LPF Magnet & Accelerometer
		fx = axes['x'] * 0.4 + (fx * (1.0 - 0.4))
		fy = axes['y'] * 0.4 + (fy * (1.0 - 0.4))
		fz = axes['z'] * 0.4 + (fz * (1.0 - 0.4))
		mx = xm * 0.4 + (mx * (1.0 - 0.4))
		my = ym * 0.4 + (my * (1.0 - 0.4))
		mz = zm * 0.4 + (mz * (1.0 - 0.4))

		#Pitch & Roll
		pitch = -1*(atan2(fx, sqrt(fy*fy + fz*fz)))
		roll = atan2(fy, fz)
		
		#Yaw
		xh = mx * cos(pitch) + mz * sin(pitch)
		yh = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch)
		yaw = atan2(yh, xh)
		
		#CorrectionYaw
		declinationAngle = (0.0 + (42.0 / 60.0)) / (180.0 / 3.1415)
		yaw += declinationAngle
		
		#Radian to Degrees
		roll = roll*180.0/3.1415
		pitch = pitch*180.0/3.1415
		yaw = yaw*180.0/3.1415

		#Complementary Filter
		nroll=AA*(nroll+rate_gyr_x*LP) +(1 - AA) * roll
		npitch=AA*(npitch+rate_gyr_y*LP) +(1 - AA) * pitch
		nyaw=AA*(nyaw+rate_gyr_z*LP) +(1 - AA) * yaw
			
		if (nyaw-math.floor(nyaw)) >= 0.005:
			nyaw = (math.ceil(nyaw*100))/100
		else:
			nyaw = (math.floor(nyaw*100))/100

		if (nroll-math.floor(nroll)) >= 0.005:
			nroll = (math.ceil(nroll*100))/100
		else:
			nroll = (math.floor(nroll*100))/100

		if (npitch-math.floor(npitch)) >= 0.005:
			npitch = (math.ceil(npitch*100))/100
		else:
			npitch = (math.floor(npitch*100))/100

		print "   Yaw = ", nyaw, "   Roll = ", nroll, "   Pitch = ", npitch
		message	= str(nyaw)+"a"+"x"+str(nroll)+"b"+str(npitch)+"c"+"1"
		arduino.write(message+"\0")
		if (count==counter):
			message2= str(nyaw)+"/"+str(nroll)+"/"+str(npitch)+"/"
			port.write(encrypt(message2))
			counter=0
		counter = counter+1
		time.sleep(0.01)
		
		
	else:	#magnet and bluetooth
		while True:	
			if (port.inWaiting()>0):
				port.flushInput()
				time.sleep(0.3)
				message	= str(0)+"a"+"x"+str(0)+"b"+str(0)+"c"+"0"
				arduino.write(message+"\0")
				time.sleep(0.3)
				message	= str(0)+"a"+"x"+str(0)+"b"+str(0)+"c"+"1"
				arduino.write(message+"\0")
				time.sleep(0.3)
				message	= str(0)+"a"+"x"+str(0)+"b"+str(0)+"c"+"0"
				arduino.write(message+"\0")
				
				print "stopping..."
				break
			port.flushInput()
			arduino.flushInput()
			port.flushOutput()
			arduino.flushOutput()
			
			adxl345 = ADXL345()

			#Raw Data Magnetic, Gyrometer, and Accelerometer 
			# L3G4200D address, 0x68(104)
			# Read data back from 0x28(40), 2 bytes, X-Axis LSB first
			data0 = bus.read_byte_data(0x69, 0x28)
			data1 = bus.read_byte_data(0x69, 0x29)

			# Convert the data
			xGyro = data1 * 256 + data0
			if xGyro > 32767 :
				xGyro -= 65536

			# L3G4200D address, 0x68(104)
			# Read data back from 0x2A(42), 2 bytes, Y-Axis LSB first
			data0 = bus.read_byte_data(0x69, 0x2A)
			data1 = bus.read_byte_data(0x69, 0x2B)

			# Convert the data
			yGyro = data1 * 256 + data0
			if yGyro > 32767 :
				yGyro -= 65536

			# L3G4200D address, 0x68(104)
			# Read data back from 0x2C(44), 2 bytes, Z-Axis LSB first
			data0 = bus.read_byte_data(0x69, 0x2C)
			data1 = bus.read_byte_data(0x69, 0x2D)

			# Convert the data
			zGyro = data1 * 256 + data0
			if zGyro > 32767 :
				zGyro -= 65536
			xm = (read_word_2c(3) - x_offset) * scale
			ym = (read_word_2c(7) - y_offset) * scale
			zm = (read_word_2c(5)) * scale
			axes = adxl345.getAxes(True)
			
			#Read Sensor Time
			b = datetime.datetime.now() - a
			a = datetime.datetime.now()
			LP = b.microseconds/(1000000*1.0)
			print "Loop Time | %5.2f|" % ( LP )
			
			#Gain Gyrometer
			rategx =  xGyro * G_GAIN
			rategy =  yGyro * G_GAIN
			rategz =  zGyro * G_GAIN
			
			#HPF Gyro
			rate_gyr_x = 0.6*rate_gyr_x+0.6*(rategx-ogx)
			rate_gyr_y = 0.6*rate_gyr_y+0.6*(rategy-ogy)
			rate_gyr_z = 0.6*rate_gyr_z+0.6*(rategz-ogz)
			
			ogx = rategx
			ogy = rategy
			ogz = rategy
			
			#LPF Magnet & Accelerometer
			fx = axes['x'] * 0.4 + (fx * (1.0 - 0.4))
			fy = axes['y'] * 0.4 + (fy * (1.0 - 0.4))
			fz = axes['z'] * 0.4 + (fz * (1.0 - 0.4))
			mx = xm * 0.4 + (mx * (1.0 - 0.4))
			my = ym * 0.4 + (my * (1.0 - 0.4))
			mz = zm * 0.4 + (mz * (1.0 - 0.4))

			#Pitch & Roll
			pitch = -1*(atan2(fx, sqrt(fy*fy + fz*fz)))
			roll = atan2(fy, fz)
			
			#Yaw
			xh = mx * cos(pitch) + mz * sin(pitch)
			yh = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch)
			yaw = atan2(yh, xh)
			
			#CorrectionYaw
			declinationAngle = (0.0 + (42.0 / 60.0)) / (180.0 / 3.1415)
			yaw += declinationAngle
			
			#Radian to Degrees
			roll = roll*180.0/3.1415
			pitch = pitch*180.0/3.1415
			yaw = yaw*180.0/3.1415

			#Complementary Filter
			nroll=AA*(nroll+rate_gyr_x*LP) +(1 - AA) * roll
			npitch=AA*(npitch+rate_gyr_y*LP) +(1 - AA) * pitch
			nyaw=AA*(nyaw+rate_gyr_z*LP) +(1 - AA) * yaw

			if (nyaw-math.floor(nyaw)) >= 0.005:
				nyaw = (math.ceil(nyaw*100))/100
			else:
				nyaw = (math.floor(nyaw*100))/100

			if (nroll-math.floor(nroll)) >= 0.005:
				nroll = (math.ceil(nroll*100))/100
			else:
				nroll = (math.floor(nroll*100))/100

			if (pitch-math.floor(npitch)) >= 0.005:
				npitch = (math.ceil(npitch*100))/100
			else:
				npitch = (math.floor(npitch*100))/100

			print "   Yaw = ", nyaw, "   Roll = ", nroll, "   Pitch = ", npitch
			message	= str(nyaw)+"a"+"x"+str(nroll)+"b"+str(npitch)+"c"+"1"
			arduino.write(message+"\0")
			if (count==counter):
				message2=str(nyaw)+"/"+str(nroll)+"/"+str(npitch)+"/"
				port.write(encrypt(message2))
				counter=0
				time.sleep(0.01)
			counter = counter+1

			time.sleep(0.01)
			#time.sleep(0.05)
		main_function()

def rc_time (pin_to_circuit):
    count = 0
  
    #Output on the pin for 
    GPIO.setup(pin_to_circuit, GPIO.OUT)
    GPIO.output(pin_to_circuit, GPIO.LOW)
    time.sleep(0.1)

    #Change the pin back to input
    GPIO.setup(pin_to_circuit, GPIO.IN)
  
    #Count until the pin goes high
    while (GPIO.input(pin_to_circuit) == GPIO.LOW):
        count += 1
    return count
	
def Light():
	port.flushInput()
	arduino.flushInput()
	port.flushOutput()
	arduino.flushOutput()
	
	a = datetime.datetime.now()
	
	#Inisalisasi Parameter Filterfx = 0
	fx = 0
	fy = 0
	fz = 0
	mx = 0
	my = 0
	mz = 0
		
	#Inisialisasi Parameter HPF Gyro
	rate_gyr_x = 0
	rate_gyr_y = 0
	rate_gyr_z = 0
	ogx = 0
	ogy = 0
	ogz = 0
	nroll = 0
	npitch = 0
	nyaw = 0
	last_time = 0
	
	adxl345 = ADXL345()

	#Raw Data Magnetic, Gyrometer, and Accelerometer 
	# L3G4200D address, 0x68(104)
	# Read data back from 0x28(40), 2 bytes, X-Axis LSB first
	data0 = bus.read_byte_data(0x69, 0x28)
	data1 = bus.read_byte_data(0x69, 0x29)

	# Convert the data
	xGyro = data1 * 256 + data0
	if xGyro > 32767 :
		xGyro -= 65536

	# L3G4200D address, 0x68(104)
	# Read data back from 0x2A(42), 2 bytes, Y-Axis LSB first
	data0 = bus.read_byte_data(0x69, 0x2A)
	data1 = bus.read_byte_data(0x69, 0x2B)

	# Convert the data
	yGyro = data1 * 256 + data0
	if yGyro > 32767 :
		yGyro -= 65536

	# L3G4200D address, 0x68(104)
	# Read data back from 0x2C(44), 2 bytes, Z-Axis LSB first
	data0 = bus.read_byte_data(0x69, 0x2C)
	data1 = bus.read_byte_data(0x69, 0x2D)

	# Convert the data
	zGyro = data1 * 256 + data0
	if zGyro > 32767 :
		zGyro -= 65536
	xm = (read_word_2c(3) - x_offset) * scale
	ym = (read_word_2c(7) - y_offset) * scale
	zm = (read_word_2c(5)) * scale
	axes = adxl345.getAxes(True)
	
	#Read Sensor Time
	b = datetime.datetime.now() - a
	a = datetime.datetime.now()
	LP = b.microseconds/(1000000*1.0)
	
	#Gain Gyrometer
	rategx =  xGyro * G_GAIN
	rategy =  yGyro * G_GAIN
	rategz =  zGyro * G_GAIN
	
	#HPF Gyro
	rate_gyr_x = 0.6*rate_gyr_x+0.6*(rategx-ogx)
	rate_gyr_y = 0.6*rate_gyr_y+0.6*(rategy-ogy)
	rate_gyr_z = 0.6*rate_gyr_z+0.6*(rategz-ogz)
	
	ogx = rategx
	ogy = rategy
	ogz = rategy
	
	#LPF Magnet & Accelerometer
	fx = axes['x'] * 0.4 + (fx * (1.0 - 0.4))
	fy = axes['y'] * 0.4 + (fy * (1.0 - 0.4))
	fz = axes['z'] * 0.4 + (fz * (1.0 - 0.4))
	mx = xm * 0.4 + (mx * (1.0 - 0.4))
	my = ym * 0.4 + (my * (1.0 - 0.4))
	mz = zm * 0.4 + (mz * (1.0 - 0.4))

	#Pitch & Roll
	pitch = -1*(atan2(fx, sqrt(fy*fy + fz*fz)))
	roll = atan2(fy, fz)
	
	#Yaw
	xh = mx * cos(pitch) + mz * sin(pitch)
	yh = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch)
	yaw = atan2(yh, xh)
	
	#CorrectionYaw
	declinationAngle = (0.0 + (42.0 / 60.0)) / (180.0 / 3.1415)
	yaw += declinationAngle
	
	#Radian to Degrees
	roll = roll*180.0/3.1415
	pitch = pitch*180.0/3.1415
	yaw = yaw*180.0/3.1415

	#Complementary Filter
	nyaw=AA*(nyaw+rate_gyr_z*LP) +(1 - AA) * yaw
	
	#cahaya
	xp = adc.read_adc(0, gain=GAIN, data_rate=128)
	yp = adc.read_adc(1, gain=GAIN, data_rate=128)
	yn = adc.read_adc(2, gain=GAIN, data_rate=128)
	xn = adc.read_adc(3, gain=GAIN, data_rate=128)
	yp = yp * 21/20
	xn = xn * 26/23
	
	if (xp<(xn+400)):
		x = xp
	else:
		x = -1*xn
	
	if (yp<(yn+400)):
		y = yp
		
	else:
		y = -1*yn
	
	yawl = math.atan2(y, x)*180/3.1415
	
	yawl = yawl+nyaw
	
	if (yawl-math.floor(yawl)) >= 0.005:
		yawl = (math.ceil(yawl*100))/100
	else:
		yawl = (math.floor(yawl*100))/100
	
	if (yawl<-180):
		yawl = yawl+360
	if (yawl>180):
		yawl = yawl-360
	
	print "nyaw = ", nyaw
	print "Yaw = ", yawl
	
	message	= str(yawl)+"a"+"x"+str(0)+"b"+str(0)+"c"+"0"
	arduino.write(message+"\0")
	time.sleep(2)
	
	
def sha256(message):
	hash_object = hashlib.sha256(message)
	hex_dig = hash_object.hexdigest()
	return hex_dig
	
def Bluetooth():
	rcv=port.read(BLOCK_SIZE)
	print "Message:", rcv
	decrypted    =    decrypt(rcv)
	print "Decrypted", decrypted
	data	=	decrypted.split("/")
	for i in range(0,3):
		data[i]	= float(data[i])
		if (data[i]-math.floor(data[i])) >= 0.005:
			data[i] = (math.ceil((data[i])*100))/100
		else:
			data[i] = (math.floor((data[i])*100))/100
	
	message	= str(data[0])+"a"+"x"+str(data[1])+"b"+str(data[2])+"c"+"0"
	arduino.write(message+"\0")
	Magnetic(1)
		
def main_function():
	print "Receiving Data..."
	
	#check data integrity
	rcv=port.read(BLOCK_SIZE)
	rcv2=port.read(BLOCK_SIZE)
	if (sha256(rcv)==rcv2):
		#split data command and timestamp
		decrypted    =    decrypt(rcv)
		data	=	decrypted.split("/")
		command = data[0]
		timestamp = float(data[1])
		
		#get current timestamp
		dt = datetime.datetime.utcnow()
		dt2 = time.mktime(dt.timetuple())
		
		#reject data outside 10 minutes from current time
		if (-600 <= timestamp-dt2 <= 600):
			if command == "1":
				#check data integrity
				rcv3=port.read(BLOCK_SIZE)
				rcv4=port.read(BLOCK_SIZE)
				
				if (sha256(rcv3)==rcv4):
					decrypted2    =    decrypt(rcv3)
					data2	=	decrypted2.split("/")
					timestamp2 = float(data2[3])
				
					#get current timestamp
					dt3 = datetime.datetime.utcnow()
					dt4 = time.mktime(dt.timetuple())
					
					#reject data outside 10 minutes from current time
					if (-600 <= timestamp2-dt4 <= 600):
						for i in range(0,3):
							data2[i]	= float(data2[i])
							if (data2[i]-math.floor(data2[i])) >= 0.005:
								data2[i] = (math.ceil((data2[i])*100))/100
							else:
								data2[i] = (math.floor((data2[i])*100))/100
						
						message	= str(data2[0])+"a"+"x"+str(data2[1])+"b"+str(data2[2])+"c"+"0"
						arduino.write(message+"\0")
						Magnetic(1)
				else:
					print "please try again.."
					
					
			if command == "2":
				message	= str(0)+"a"+"x"+str(0)+"b"+str(0)+"c"+"0"
				arduino.write(message+"\0")
				Magnetic(1)
			else:
				Light()
				Magnetic(1)

while True:
	port.flushInput()
	arduino.flushInput()
	port.flushOutput()
	arduino.flushOutput()
	print "Waiting to authenticate.."
	##authenticate
	#receive R1 data
	R1 = port.read(BLOCK_SIZE)	
	l = R1.count('a')
	R1 = R1[:len(R1)-l]
	R1 = str(R1)

	
	#encrypt R1 to R2 and send to laptop
	R2 = encrypt(R1)
	port.write(R2)
	
	#receive if R1 equals R2
	check = port.read(BLOCK_SIZE)
	check = decrypt(check)
	#if R1 == R2 then proceed
	if (check == "1"):
		main_function()