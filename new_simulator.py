# A simple object oriented tkinter window
import tkinter as tk
from tkinter import ttk
import math
from tkinter.constants import E, W
import serial
import serial.tools.list_ports
import time
 
# -- Windows only configuration --
try:
    from ctypes import windll
    windll.shcore.SetProcessDpiAwareness(1)
except:
    pass
    
def IK2(x,y, L1,L2,A,B,C,D):
    delta = math.sqrt(x**2+y**2)
    dzetta0 = math.acos(x/delta)
    print(f"delta: {delta}")
    delta_max = (L1+L2) 
    if delta > delta_max:
        print("Readjusting...")
        x = delta_max * math.cos(dzetta0)
        y = delta_max * math.sin(dzetta0)
        delta = math.sqrt(x**2+y**2)
        dzetta0 = math.acos(x/delta)
        print(f"delta: {delta}")

    x = max(0,x)
    y = max(L1,y)


    fi_in = (delta**2+L1**2-L2**2)/(2*delta*L1)
    fi_in = min(1,max(-1,fi_in))
    fi = math.acos(fi_in)

    dz_in = (L1**2+L2**2-delta**2)/(2*L1*L2)
    dz_in = min(1,max(-1,dz_in))
    dzetta = 180 - math.degrees(math.acos(dz_in))

    Beta = dzetta0 + fi
    Beta = math.degrees(Beta)

    z = math.sqrt(C**2+B**2-2*B*C*math.cos(math.radians(Beta)))
    print(f"z: {z}")

    A1_in = (z**2+C**2-B**2)/(2*z*C)
    A2_in = (z**2+A**2-D**2)/(2*z*A)
    A1_in = min(1,max(-1,A1_in))
    A2_in = min(1,max(-1,A2_in))

    print(f"A1: {A1_in} . A2: {A2_in}")

    Alfa1 = math.acos(A1_in)
    Alfa2 = math.acos(A2_in)

    Servo1 = 180 - math.degrees(Alfa1) - math.degrees(Alfa2)
    Servo1 = min(150,max(0,Servo1))

    M = math.sqrt(L1**2+B**2-2*L1*B*math.cos(math.radians(dzetta)))
    print(f"M: {M}")

    A1_in = (L1**2+M**2-B**2)/(2*L1*M)
    A2_in = (A**2+M**2-L1**2)/(2*A*M)
    A1_in = min(1,max(-1,A1_in))
    A2_in = min(1,max(-1,A2_in))

    print(f"A1: {A1_in} . A2: {A2_in}")
    Alfa1 = math.acos(A1_in)
    Alfa2 = math.acos(A2_in)

    Servo2 = Servo1 + math.degrees(Alfa1) + math.degrees(Alfa2)
    Servo2 = min(150+120,max(120,Servo2))

    return Servo1, Servo2

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

        Alfa = math.degrees(Alfa) - 10
        Beta = math.degrees(Beta) - 120

        Alfa = max(25,min(155,Alfa))
        Beta = max(Alfa-100,min(Alfa+35,max(35,min(165,Beta))))
    
    return Alfa, Beta

def serial_snd(msg):
    if is_serial:
        ser.write(msg.encode())
    else:
        print(f"SerialSim: {msg}")


def setLegs(Alfa, Beta, legs=[1,2,3,4]):
    """
    Setting up all legs for the given Alfa and Beta servo angles
    """

    servos = []
    if 1 in legs:
        servos.append(0)
        servos.append(1)
    if 2 in legs:
        servos.append(2)
        servos.append(3)
    if 3 in legs:
        servos.append(4)
        servos.append(5)
    if 4 in legs:
        servos.append(6)
        servos.append(7)


    zeromsg = "<42"

    korekty = []
    for _ in range(8):
        korekty.append(0)

    korekty[2] = 10
    korekty[5] = 10

    for servo in range(8):
        if servo in servos:
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
        else:
            zeromsg +=",-1"
 
    zeromsg +=">"

    return zeromsg

    
class HelloWorld(tk.Tk):
    def __init__(self):
        super().__init__()

        self.geometry("640x750")
        self.title("FLR Leg Simulator")

        label = ttk.Label(self, text="FLR IK Simulator v0.2")
        label.config(font=("Arial",20))
        label.pack()

        self.W = 600
        self.H = 600

        self.zeroX = 350
        self.zeroY = 100

        self.klikx = 350
        self.kliky = 100

        self.c = tk.Canvas(self, bg="blue", width=self.W, height=self.H)
        self.c.pack()

        self.c.bind("<Button-1>", self.update)
        self.c.bind("<B1-Motion>", self.update)

        self.c_objects = []


        self.sAlfa = tk.Scale(self, from_=0, to=180, orient=tk.HORIZONTAL, command=self.sliders)
        self.sAlfa.pack()
        self.sAlfa.set(90)
        self.sBeta = tk.Scale(self, from_=0, to=180, orient=tk.HORIZONTAL, command=self.sliders)
        self.sBeta.pack()
        self.sBeta.set(90)

        self.leg1 = tk.IntVar(value=1)
        self.leg2 = tk.IntVar(value=1)
        self.leg3 = tk.IntVar(value=1)
        self.leg4 = tk.IntVar(value=1)

        L1 = tk.Checkbutton(self, text='Leg 1',variable=self.leg1, onvalue=1, offvalue=0)
        L1.pack()
        L2 = tk.Checkbutton(self, text='Leg 2',variable=self.leg2, onvalue=1, offvalue=0)
        L2.pack()
        L3 = tk.Checkbutton(self, text='Leg 3',variable=self.leg3, onvalue=1, offvalue=0)
        L3.pack()
        L4 = tk.Checkbutton(self, text='Leg 4',variable=self.leg4, onvalue=1, offvalue=0)
        L4.pack()

        self.uartBtn = tk.Button(self, text ="UART >>>", command = self.uart)
        self.uartBtn.pack()


        self.redraw()

        # base data for the leg
        skala = 2
        self.skala = skala
        self.A = 17 * skala
        self.B = 35 * skala
        self.C = 55 * skala
        self.D = 60 * skala
        self.L1 = 90 * skala
        self.L2 = (90-35) * skala

        self.servo_R = 17 * skala 
        self.servo_space = 60 * skala
        self.servo_spacer = 60 * skala


        # initial draw
        self.makeLeg(self.sAlfa.get(), self.sBeta.get())

    def uart(self):
        legs = []
        if self.leg1.get():
            legs.append(1)
        if self.leg2.get():
            legs.append(2)
        if self.leg3.get():
            legs.append(3)
        if self.leg4.get():
            legs.append(4)

        msg = setLegs(self.sAlfa.get(), self.sBeta.get(), legs)
        serial_snd(msg)

    def sliders(self, _):
        A = self.sAlfa.get()
        B = self.sBeta.get()
        self.redraw()
        self.makeLeg(A,B)
        ...

    def redraw(self):
        if len(self.c_objects):
            for obj in self.c_objects:
                self.c.delete(obj)

        # drawing axes and point
        r = 5
        tmp = self.c.create_oval(self.zeroX-r,self.zeroY-r,self.zeroX+r,self.zeroY+r,width=1)
        self.c_objects.append(tmp)

        tmp = self.c.create_line(0,self.zeroY,self.W,self.zeroY)
        self.c_objects.append(tmp)

        tmp = self.c.create_line(self.zeroX,0,self.zeroX,self.H)
        self.c_objects.append(tmp)

        tmp = self.c.create_line(self.zeroX, self.zeroY,self.klikx, self.kliky)
        self.c_objects.append(tmp)
        ...

    def drawBar(self, x,y,L,alfa,clr="gray",thk=10):
        # method to draw the bar
        a = math.radians(alfa - 90)
        dy = L * math.cos(a)
        dx = L * math.sin(a)
        
        
        yend = y + dy
        xend = x + dx

        tmp = self.c.create_line(x,y,xend,yend, width=thk, fill=clr)
        self.c_objects.append(tmp)

        return xend, yend
        ...
    
    def makeLeg(self, Alfa, Beta):
        """
        Function to draw the simulated robo-dog-leg
        """
        point_list = []
        point_list.append((self.zeroX,self.zeroY))
        point_list.append((self.zeroX-self.C,self.zeroY))

        # 1st servo arm
        x0,y0 = self.drawBar(self.zeroX - self.C, self.zeroY,self.A,Alfa,clr="red")
        point_list.append((x0,y0))
        # 2nd servo arm
        x1,y1 = self.drawBar(self.zeroX, self.zeroY,self.A,Beta+120,clr="red")
        point_list.append((x1,y1))

        # leg part L1
        in_d = -2*self.A*self.C*math.cos(math.pi-math.radians(Alfa))+self.A**2+self.C**2
        d = math.sqrt(in_d)
        fi1 = math.acos((d**2+self.C**2-self.A**2)/(2*d*self.C)) 
        fi2 = math.acos((d**2+self.B**2-self.D**2)/(2*d*self.B)) 
        Beta1 = fi1 + fi2
        Beta1 = math.degrees(Beta1)

        x2,y2 = self.drawBar(self.zeroX, self.zeroY, self.L1, Beta1)
        point_list.append((x2,y2))

        # leg part 2
        in_w = self.L1**2 + self.A**2 -2*self.L1*self.A*math.cos(math.radians(Beta+120)-math.radians(Beta1))
        w = math.sqrt(in_w)

        fii1 = math.acos((self.L1**2+w**2 - self.A**2)/(2*self.L1*w))
        fii2 = math.acos((self.B**2+w**2-self.L1**2)/(2*self.B*w))

        dzetta = math.degrees(fii1 + fii2)

        psi = Beta1 - dzetta 

        xe,ye = self.drawBar(x2,y2,self.L2,psi)
        point_list.append((xe,ye))

        x4,y4 = self.drawBar(x2,y2,self.B,psi+180)
        point_list.append((x4,y4))

        # calculations for the connection arms
        d_angle = math.acos((self.B**2+self.D**2-d**2)/(2*self.D*self.B))
        d_angle = math.degrees(d_angle)

        d_angle = Beta1 + d_angle
        x5,y5 = self.drawBar(x0,y0,self.D,d_angle,clr="green")
        point_list.append((x5,y5))

        w_angle = math.acos((self.B**2+self.L1**2-w**2)/(2*self.B*self.L1))
        w_angle = math.degrees(w_angle)

        x6,y6 = self.drawBar(x4,y4,self.L1,psi+180+180-w_angle, clr="green")
        point_list.append((x6,y6))

        xe = int(xe)
        ye = int(ye)
        print(f"x,y: ({xe},{xe})")



        # # 1st servo bar
        # x0,y0 = self.drawBar(x0,y0,self.servo_spacer,180,clr="silver")
        # point_list.append((x0,y0))
        # #1st part of leg
        # x0,y0 = self.drawBar(self.zeroX, self.zeroY,self.A,Alfa+10,clr="green")
        # point_list.append((x0,y0))
        # # 2nd part of leg
        # x,y = self.drawBar(x0,y0,self.A,Beta+120-180,clr="magenta")
        # point_list.append((x,y))
        # x0,y0 = self.drawBar(x0,y0,self.servo_R,Beta+120,clr="magenta")
        # point_list.append((x0,y0))
        # # 2nd servo bar
        # x0,y0 = self.drawBar(x0,y0,self.A,Alfa+10-180,clr="silver")
        # point_list.append((x0,y0))

        for pt in point_list:
            x,y = pt
            tmp = self.c.create_oval(x-5,y-5,x+5,y+5, fill="silver")
            self.c_objects.append(tmp)

    def update(self, event):
        if (self.klikx != event.x) or (self.kliky != event.y):
            self.klikx = event.x
            self.kliky = event.y


            x = event.x - self.zeroX
            y = event.y - self.zeroY
            # x = x / (2*self.A)
            # y = y / (2*self.A)
            # print(x,y)

            A,B = IK2(-x,y, self.L1,self.L2,self.A,self.B,self.C,self.D)

            print(A, B-120)

            self.redraw()
            self.makeLeg(A,B-120)
            self.sAlfa.set(int(A))
            self.sBeta.set(int(B-120))
        
        ...


print('Search...')
ports = serial.tools.list_ports.comports(include_links=False)
for port in ports :
    print('Find port '+ port.device)

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
root = HelloWorld()
root.mainloop()

if is_serial:
    ser.close()             # close port
    print("Closing the serial port")