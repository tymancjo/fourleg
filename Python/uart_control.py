from sys import is_finalizing
import serial
import time
import math
# ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
# ser.baudrate = 9600

def IK(x,y,A=0.5):
    """
    Inverse kinematic model for the RDL
    """
    Alfa = False
    Beta = False
    M = math.sqrt(x**2 + y**2)
    print(f"Initial module: {M}")
    if M > 2*A:
        ratio = 2*A / M
        x *= ratio
        y *= ratio
        M = math.sqrt(x**2 + y**2)
        print(f"Accomodated: ({x,y}) M:{M}")

    if M <= 2*A:
        Alfa = math.pi - math.asin(x/M) - math.asin(M/(2*A))
        Beta = math.pi + math.asin(M/(2*A)) - math.asin(x/M)

        Alfa = math.degrees(Alfa) - 00
        Beta = math.degrees(Beta) - 120

        Alfa = max(25,min(155,Alfa))
        Beta = max(Alfa-100,min(Alfa+35,max(35,min(165,Beta))))
    
    return Alfa, Beta

try:
    ser = serial.Serial(
        port='/dev/cu.usbserial-120',
        baudrate=115200,
        # parity=serial.PARITY_ODD,
        # stopbits=serial.STOPBITS_TWO,
        # bytesize=serial.EIGHTBITS
    )
    print(ser.name)         # check which port was really used
    is_serial = True
except:
    is_serial = False
    print("No Serial Mode")

time.sleep(2)

# Just to keep it here for now
# list = """
# <41,15,100> <41,8,30>;
# <41,15,125> <41,8,55>;
# <41,15,125> <41,8,115>;
# <41,15,125> <41,8,125>;
# <41,15,90> <41,8,110>;
# <41,15,70> <41,8,100>;
# <41,15,55> <41,8,80>;
# <41,15,50> <41,8,40>;
# """

# step_list = [
#     (100,30),
#     (125,55),
#     (125,115),
#     (125,125),
#     (90,110),
#     (70,100),
#     (55,80),
#     (50,40)
# ]

def serial_snd(msg):
    if is_serial:
        ser.write(msg.encode())
    else:
        print(f"SerialSim: {msg}")


def setLegs(Alfa, Beta):
    """
    Setting up all legs for the given Alfa and Beta servo angles
    """

    zeromsg = "<41"

    korekty = []
    for _ in range(8):
        korekty.append(0)

    korekty[2] = 10
    korekty[6] = 10

    for servo in range(8):
        if servo in [0,2,4,6]:
            Acor = Beta + korekty[servo]
            if servo in [0,1,4,5]:
                Bcor = 180 - Acor
            else:
                Bcor = Acor
            zeromsg += f",{int(Bcor)}"
        else:
            Acor = Alfa + korekty[servo]
            if servo in [0,1,4,5]:
                Bcor = 180 - Acor
            else:
                Bcor = Acor
            zeromsg += f",{int(Bcor)}"
 
    zeromsg +=">"

    return zeromsg



while True:
    A,B = IK(0.1,1)
    if A and B:
        msg = setLegs(A,B)
    serial_snd(msg)

    time.sleep(1)

    A,B = IK(0,1)
    if A and B:
        msg = setLegs(A,B)
    serial_snd(msg)
    
    time.sleep(1)
    

if is_serial:
    ser.close()             # close port
