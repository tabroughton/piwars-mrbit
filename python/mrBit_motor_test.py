import time
import grovepi
import bluetooth

#set up bt server

server_sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )

port = 0x1101
server_sock.bind(("",port))
server_sock.listen(1)

client_sock,address = server_sock.accept()
print "Accepted connection from ",address


#set up motor driver pins
motAF = 7
motAR = 8
motBF = 4 
motBR = 3
motAPWM = 5
motBPWM = 6

pwmVal = 0

grovepi.pinMode(motAF,"OUTPUT")
grovepi.pinMode(motAR,"OUTPUT")
grovepi.pinMode(motBF,"OUTPUT")
grovepi.pinMode(motBR,"OUTPUT")
grovepi.pinMode(motAPWM,"OUTPUT")
grovepi.pinMode(motBPWM,"OUTPUT")

try:
     while 1:
        data = client_sock.recv(1024)
        if data:
            print "received [%s]" % data

        if data.isdigit():
            if int(data) >= 0 & int(data) <= 255:
                print ("speed = %s" % data)
                pwmVal =  int(data)

            if data == "F":
                #test forwards both
                grovepi.digitalWrite(motAF,1)
                grovepi.digitalWrite(motBF,1)
                grovepi.digitalWrite(motAR,0)
                grovepi.digitalWrite(motBR,0)
                grovepi.analogWrite(motAPWM, pwmVal)
                grovepi.analogWrite(motBPWM, pwmVal)
                print ("moving fowards")
                     
            if data == "B":
                #test backwards both
                grovepi.digitalWrite(motAF,0)
                grovepi.digitalWrite(motBF,0)
                grovepi.digitalWrite(motAR,1)
                grovepi.digitalWrite(motBR,1)
                grovepi.analogWrite(motAPWM, pwmVal)
                grovepi.analogWrite(motBPWM, pwmVal)
                print ("moving backwards")
                
            if data == "R":
		#test right
                grovepi.digitalWrite(motAF,1)
                grovepi.digitalWrite(motBF,0)
                grovepi.digitalWrite(motAR,0)
                grovepi.digitalWrite(motBR,0)
                grovepi.analogWrite(motAPWM, pwmVal)
                grovepi.analogWrite(motBPWM, pwmVal)               
                print ("turning right")

            if data == "L":
                #test left
                grovepi.digitalWrite(motAF,0)
                grovepi.digitalWrite(motBF,1)
                grovepi.digitalWrite(motAR,0)
                grovepi.digitalWrite(motBR,0)
                grovepi.analogWrite(motAPWM, pwmVal)
                grovepi.analogWrite(motBPWM, pwmVal)
                print ("turning left")

            if data == "S":
                grovepi.digitalWrite(motAF,0)
                grovepi.digitalWrite(motBF,0)
                grovepi.digitalWrite(motAR,0)
                grovepi.digitalWrite(motBR,0)
                grovepi.analogWrite(motAPWM, 0)
                grovepi.analogWrite(motBPWM, 0)
                print ("stop")
            

except Exception as e: print str(e)

except: 	# Turn motors off
    grovepi.digitalWrite(motAF,0)
    grovepi.digitalWrite(motBF,0)
    grovepi.digitalWrite(motAR,0)
    grovepi.digitalWrite(motBR,0)   
    grovepi.analogWrite(motAPWM, 0)
    grovepi.analogWrite(motBPWM, 0)
    client_sock.close()
    server_sock.close()



