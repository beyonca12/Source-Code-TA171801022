#import library
import serial
import hashlib
import time
import random
import datetime
from Crypto.Cipher import AES
import xlrd
import xlwt
from xlutils.copy import copy

key = b'\xbf\xc0\x85)\x10nc\x94\x02)j\xdf\xcb\xc4\x94\x9d(\x9e[EX\xc8\xd5\xbfI{\xa2$\x05(\xd5\x18'
cipher = AES.new(key)
raspi = serial.Serial('COM5', 38400)
BLOCK_SIZE	=	64

def authenticate():
	#sending data from laptop
	R1 = random.randint(1,10000)
	raspi.write(pad(str(R1)))
	time.sleep(0.1)
	
	#receive encrypted R1 and decrypted it
	check = raspi.read(BLOCK_SIZE)
	check2 = int(decrypt(check))
	
	#check if R1 equals received after decryption
	if (check2 == R1):
		raspi.write(encrypt("1"))
		return "1"
	else:
		raspi.write(encrypt("0"))
		return "0"

def sha256(message):
	hash_object = hashlib.sha256(message)
	hex_dig = hash_object.hexdigest()
	return hex_dig

##create function
#pad function to pad remaining space until BLOCK_SIZE is fulfilled
def pad(s):
	return s + ((BLOCK_SIZE - len(s)%BLOCK_SIZE)*'a')
	
#encrypt function to encrypt message before sending them
def encrypt(plaintext):
	global cipher
	return cipher.encrypt(pad(plaintext))
	
def decrypt (ciphertext):
    global cipher
    dec = cipher.decrypt(ciphertext).decode('ISO-8859-1')
    l = dec.count('a')
    return dec[:len(dec)-l]

def Data_logging():
	try:
		raspi.flushInput()
		rcv = raspi.read(BLOCK_SIZE)
		time.sleep(0.01)
		decrypted    =    decrypt(rcv)
		data	=	decrypted.split("/")
		time.sleep(0.01)
		for i in range(0,3):
			data[i]	= float(data[i])
		
		print "yaw", data[0], "roll", data[1], "pitch", data[2]
		rb = xlrd.open_workbook('Data.xls',formatting_info=True)
		r_sheet = rb.sheet_by_index(0) 
		r = r_sheet.nrows
		wb = copy(rb) 
		sheet = wb.get_sheet(0)
		sheet.write(r,0,data[0])
		sheet.write(r,1,data[1])
		sheet.write(r,2,data[2])
		wb.save('Data.xls')

	except ValueError:
		pass
	
	
#input_value function to ask for yaw, roll, and pitch input from the user and send the message
def input_value():
	print "Please choose the mode: (type 1 or 2)"
	print "1. Manual Mode : Bluetooth Communication"
	print "2. Automatic Mode"
	command = raw_input("Type something : ");
	print ""
	
	if command == "1":
		#send command and timestamp for bluetooth mode
		dt = datetime.datetime.utcnow()
		dt2 = time.mktime(dt.timetuple())
		message = "1"+"/"+str(dt2)+"/"
		encrypted	=	encrypt(message)
		raspi.write(encrypted)
		time.sleep(0.1)
		raspi.write(sha256(encrypted))
		
		#ask user input and send the message
		yaw		=	raw_input("please write the first coordinate (yaw): ")	
		roll 	=	raw_input("please write the second coordinate (roll): ")	
		pitch	=	raw_input("please write the third coordinate (pitch): ")
		dt = datetime.datetime.utcnow()
		dt2 = time.mktime(dt.timetuple())
		message	=	yaw+"/"+roll+"/"+pitch+"/"+str(dt2)+"/"
		encrypted	=	encrypt(message)
		raspi.write(encrypted)
		time.sleep(0.1)
		raspi.write(sha256(encrypted))
		
	elif command == "2":
		print "Please choose the sensor: (type 1 or 2)"
		print "1. Magnetic Sensor"
		print "2. Light Sensor"
		command2 = raw_input("Type something : ");
		print ""
		if command2 == "1":
			#send command and timestamp for magnetic mode
			dt = datetime.datetime.utcnow()
			dt2 = time.mktime(dt.timetuple())
			message = "2"+"/"+str(dt2)+"/"
			encrypted	=	encrypt(message)
			raspi.write(encrypted)
			time.sleep(0.1)
			raspi.write(sha256(encrypted))
			
		elif command2 == "2":
			#send command and timestamp for light sensor mode
			dt = datetime.datetime.utcnow()
			dt2 = time.mktime(dt.timetuple())
			message = "3"+"/"+str(dt2)+"/"
			encrypted	=	encrypt(message)
			raspi.write(encrypted)
			time.sleep(0.1)
			raspi.write(sha256(encrypted))
			
		else:
			print "Wrong input"
	else:
		print "Wrong input"
	
	#Receive data from raspi
	try:
		while True:
			Data_logging()
	#stopping the program
	except KeyboardInterrupt:
		rb = xlrd.open_workbook('Data.xls',formatting_info=True)
		r_sheet = rb.sheet_by_index(0) 
		r = r_sheet.nrows
		wb = copy(rb) 
		sheet = wb.get_sheet(0)
		sheet.write(r,0,"stop")
		sheet.write(r,1,"stop")
		sheet.write(r,2,"stop")
		wb.save('Data.xls')
		
		print "Stopping.."
		raspi.write("00000000000")
		time.sleep(1)
			
	input_value()
	
	
#main program
while True:
	raspi.flushOutput()
	if (authenticate()=="1"):
		print "Valid Credentials, Please Proceed"
		print ""
		input_value()
	else:
		print "Invalid Credentials."
	time.sleep(1)