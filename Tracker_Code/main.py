import socket
import serial
from time import sleep

#initalize Serial port
ser = serial.Serial('/dev/ttyACM2', 9600, timeout=1, parity=serial.PARITY_EVEN)


TCP_IP = '127.0.0.1'
TCP_PORT = 4533
BUFFER_SIZE = 100

#initalize/connect to TCP Port // data parsing form https://adventurist.me/posts/0136
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)

conn, addr = s.accept()
print('Connection address:', addr)

az = 0.0 #AZ from Pico
el = 0.0 #EL from Pico

mmaz = 0 #AZ from gPredict
mmel = 0 #EL from gPredict

maz = '9999' #AZ to Pico
mel = '9999' #EL to Pico

response = " " # data to gPredict
while 1:
	data = conn.recv(BUFFER_SIZE) #read data fronm gPredict
	response = "{}\n{}\n".format(float(f'{az:.2f}'), float(f'{el:.2f}')) #parse data to gPredict

	if (data.startswith(b'P')):
		values = data.split(b' ')
		#print(values)
		mmaz = int(float(values[1])*10)+1800 #convert to integer and add 180° // 1800 because 0.1° resolution -> uneccessary...
		mmel = int(float(values[2])*10) #convert to interger

		conn.send(bytes(response, 'utf-8')) #send data from Pico to gPredict
	elif data == b'q\n' or data == b'S\n':
		print("close command, shutting down") #closes Program
		conn.close()
		exit()
	elif data == b'p\n':
		conn.send(bytes(response, 'utf-8')) # send data from Pico to gPredict

	print(response)
	print("moving to az:{} el: {}".format( mmaz, mmel));

	# Parse data for Pico:
	# AZ: 180°, EL: 45° -> 18000450

	if(mmaz == 0):
		maz = '000' + str(mmaz)
	if(mmaz < 100):
		maz = '00' + str(mmaz)
	elif(mmaz < 1000):
		maz = '0' + str(mmaz)
	else:
		maz = str(mmaz)

	if(mmel == 0):
		mel = '000' + str(mmel)
	elif(mmel < 100):
		mel = '00' + str(mmel)
	elif(mmel < 1000):
		mel = '0' + str(mmel)
	else:
		mel  = str(mmel)

	#Try to write to the Pico and clear buffer if not print nosend
	try:
		print("az: " + str(mmaz/10) + " el: " + str(mmel/10))
		ser.reset_output_buffer()
		ser.write(bytes(maz + mel +'\r\n', 'ascii'))
		ser.reset_output_buffer()
		
	except:
		print('nosend')
		ser.reset_output_buffer()
		print(str(maz + mel))

	#Try to read to the Pico and clear buffer if not print nor
	try:
		if(ser.in_waiting > 0 ):
			adc = ser.readline()
			ser.reset_input_buffer()
			ser.read_all()
			adc = adc.split(b'/')
			print(adc)
			print("az: " + str(float(adc[0])/10) + " el: " + str(float(adc[1])/10) + " RECEV:" + str(adc[2]))
			az = int(float(adc[0])/10)
			el = int(float(adc[1])/10)
			sleep(0.5)
	except:
		print('nor')

	sleep(1.6) #sleep for clearing buffer.
	

	
