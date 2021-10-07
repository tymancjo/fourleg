# A simple object oriented tkinter window
import tkinter as tk
from tkinter import ttk
import math
from tkinter.constants import E, W
import serial
import time
 
# -- Windows only configuration --
try:
    from ctypes import windll
    windll.shcore.SetProcessDpiAwareness(1)
except:
    pass
    


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

        self.uartBtn = tk.Button(self, text ="UART >>>", command = self.uart)
        self.uartBtn.pack()


        self.redraw()

        # base data for the leg
        skala = 1.5
        self.A = 90 * skala
        self.servo_R = 17 * skala 
        self.servo_space = 60 * skala
        self.servo_spacer = 60 * skala

        # initial draw
        self.makeLeg(self.sAlfa.get(), self.sBeta.get())

    def uart(self):
        msg = setLegs(self.sAlfa.get(), self.sBeta.get())
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
        point_list.append((self.zeroX-self.servo_space,self.zeroY))

        # 1st servo arm
        x0,y0 = self.drawBar(self.zeroX - self.servo_space, self.zeroY,self.servo_R,Alfa,clr="red")
        point_list.append((x0,y0))
        # 2nd servo arm
        x,y = self.drawBar(self.zeroX, self.zeroY,self.servo_R,Beta+120,clr="red")
        point_list.append((x,y))
        # 1st servo bar
        x0,y0 = self.drawBar(x0,y0,self.servo_spacer,180,clr="silver")
        point_list.append((x0,y0))
        #1st part of leg
        x0,y0 = self.drawBar(self.zeroX, self.zeroY,self.A,Alfa+00,clr="green")
        point_list.append((x0,y0))
        # 2nd part of leg
        x,y = self.drawBar(x0,y0,self.A,Beta+120-180,clr="magenta")
        point_list.append((x,y))
        x0,y0 = self.drawBar(x0,y0,self.servo_R,Beta+120,clr="magenta")
        point_list.append((x0,y0))
        # 2nd servo bar
        x0,y0 = self.drawBar(x0,y0,self.A,Alfa+00-180,clr="silver")
        point_list.append((x0,y0))

        for pt in point_list:
            x,y = pt
            tmp = self.c.create_oval(x-10,y-10,x+10,y+10, fill="silver")
            self.c_objects.append(tmp)

    def update(self, event):
        self.klikx = event.x
        self.kliky = event.y


        x = event.x - self.zeroX
        y = event.y - self.zeroY
        x = x / (2*self.A)
        y = y / (2*self.A)

        print(x,y)

        A,B = IK(-x,y)
        print(A, B)
        if A and B:
            self.redraw()
            self.makeLeg(A,B)
            self.sAlfa.set(int(A))
            self.sBeta.set(int(B))
        
        ...


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