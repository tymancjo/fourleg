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
    

# global command lists for the walk learning
last_servos = []
undo_servos = []
undo_servos.append([])
for _ in range(8):
    last_servos.append(90)
moves = []

def IK2(x,y, L1,L2,L3,A,B,C,D):
    delta = math.sqrt(x**2+y**2)
    dzetta0 = math.acos(x/delta)
    # print(f"delta: {delta}")
    delta_max = (L1+L2) * 0.98
    if delta > delta_max:
        # print("Readjusting...")
        x = delta_max * math.cos(dzetta0)
        y = delta_max * math.sin(dzetta0)
        delta = math.sqrt(x**2+y**2)
        dzetta0 = math.acos(x/delta)
        # print(f"delta: {delta}")

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
    # print(f"z: {z}")

    A1_in = (z**2+C**2-B**2)/(2*z*C)
    A2_in = (z**2+A**2-D**2)/(2*z*A)
    A1_in = min(1,max(-1,A1_in))
    A2_in = min(1,max(-1,A2_in))

    # print(f"A1: {A1_in} . A2: {A2_in}")

    Alfa1 = math.acos(A1_in)
    Alfa2 = math.acos(A2_in)

    Servo1 = 180 - math.degrees(Alfa1) - math.degrees(Alfa2)
    Servo1 = min(150,max(0,Servo1))

    M = math.sqrt(L1**2+B**2-2*L1*B*math.cos(math.radians(dzetta)))
    # print(f"M: {M}")

    A1_in = (L1**2+M**2-B**2)/(2*L1*M)
    A2_in = (A**2+M**2-L3**2)/(2*A*M)
    A1_in = min(1,max(-1,A1_in))
    A2_in = min(1,max(-1,A2_in))

    # print(f"A1: {A1_in} . A2: {A2_in}")
    Alfa1 = math.acos(A1_in)
    Alfa2 = math.acos(A2_in)

    Servo2 = Servo1 + math.degrees(Alfa1) + math.degrees(Alfa2)
    Servo2 = min(120+180,max(120-90,Servo2))

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
    global undo_servos
    undo_servos[0] = last_servos[:]

    if is_serial:
        ser.write(msg.encode())
        print(f"Serial: {msg}")
    else:
        print(f"SerialSim: {msg}")
    
    temp_servo = msg[4:-1].split(",")
    for i,s in enumerate(temp_servo):
        s = int(s)
        if s != -1:
            last_servos[i] = s

    print(last_servos)

def undo_last():
    make_moves(mv=undo_servos)
    

def del_last():
    global moves
    moves = moves[:-1]
    ...

def make_moves(mv=moves):
    if len(mv):
        lista = ""
        for move in mv:

            msg = "<42,"
            for s in move:
                msg += f"{s},"
            msg = msg[:-1]
            msg +=">;"

            lista += msg
        
        lista = lista[:-1]
        print(lista)

        serial_lista(lista)

def print_moves():
    if len(moves):
        lista = ""
        for move in moves:

            msg = "<42,"
            for s in move:
                msg += f"{s},"
            msg = msg[:-1]
            msg +=">;\n\r"

            lista += msg
        
        lista = lista[:-1]
        print(lista)

def serial_lista(msg, dt=500, n=1,s=1):
    kroki = msg.split(";")

    if len(kroki):
        for _ in range(n):
            for krok in kroki[::s]:
                krok = krok.strip()
                serial_snd(krok)
                time.sleep(dt/1000)

def setServos(A,B):

    zeromsg = "<42"

    korekty = []
    for _ in range(8):
        korekty.append(0)

    # korekty[2] = 10
    # korekty[5] = 10

    servos = [0,1,2,3,4,5,6,7]

    for servo in range(8):
        the_leg = servo // 2

        Alfa = A[the_leg]
        Beta = B[the_leg] - 120
        
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

        self.geometry("1240x750")
        self.title("FLR Leg Simulator")

        label = ttk.Label(self, text="FLR IK Simulator v0.2")
        label.config(font=("Arial",20))
        # label.pack()
        label.grid(row=1,column=1,columnspan=4)

        self.W = 600
        self.H = 600

        self.zeroX = 350
        self.zeroY = 100

        self.klikx = 350
        self.kliky = 100

        self.c = tk.Canvas(self, bg="blue", width=self.W, height=self.H)
        # self.c.pack()
        self.c.grid(row=2,column=10,columnspan=4, rowspan=20)

        self.c.bind("<Button-1>", self.update)
        self.c.bind("<B1-Motion>", self.update)

        self.c_objects = []


        self.sAlfa = tk.Scale(self, from_=0, to=180, orient=tk.HORIZONTAL, command=self.sliders)
        # self.sAlfa.pack()
        self.sAlfa.grid(row=3,column=1,columnspan=2)
        self.sAlfa.set(90)
        self.sBeta = tk.Scale(self, from_=0, to=180, orient=tk.HORIZONTAL, command=self.sliders)
        # self.sBeta.pack()
        self.sBeta.grid(row=3,column=3,columnspan=2)
        self.sBeta.set(90)

        self.leg1 = tk.IntVar(value=1)
        self.leg2 = tk.IntVar(value=1)
        self.leg3 = tk.IntVar(value=1)
        self.leg4 = tk.IntVar(value=1)

        L1 = tk.Checkbutton(self, text='Leg 1',variable=self.leg1, onvalue=1, offvalue=0)
        L1.grid(row=4,column=1)
        L2 = tk.Checkbutton(self, text='Leg 2',variable=self.leg2, onvalue=1, offvalue=0)
        L2.grid(row=4,column=2)
        L3 = tk.Checkbutton(self, text='Leg 3',variable=self.leg3, onvalue=1, offvalue=0)
        L3.grid(row=5,column=1)
        L4 = tk.Checkbutton(self, text='Leg 4',variable=self.leg4, onvalue=1, offvalue=0)
        L4.grid(row=5,column=2)

        self.uartBtn = tk.Button(self, text ="UART >>>", command = self.uart)
        self.uartBtn.grid(row=6,column=2,columnspan=1)

        self.uartBtn = tk.Button(self, text ="Save>>", command =lambda: moves.append(last_servos[:]))
        self.uartBtn.grid(row=6,column=3,columnspan=1)

        self.uartBtn = tk.Button(self, text ="Show", command =lambda: print(moves))
        self.uartBtn.grid(row=6,column=4,columnspan=1)

        self.uartBtn = tk.Button(self, text ="Del.Last", command =del_last)
        self.uartBtn.grid(row=6,column=5,columnspan=1)

        self.uartBtn = tk.Button(self, text ="Make!", command =make_moves)
        self.uartBtn.grid(row=7,column=3,columnspan=1)

        self.uartBtn = tk.Button(self, text ="Print", command =print_moves)
        self.uartBtn.grid(row=7,column=4,columnspan=1)

        self.uartBtn = tk.Button(self, text ="Undo Lst", command =undo_last)
        self.uartBtn.grid(row=7,column=5,columnspan=1)
        self.redraw()

        # base data for the leg
        skala = 2
        self.skala = skala
        self.A = 25 * skala
        self.B = 35 * skala
        self.C = 55 * skala
        self.D = 60 * skala
        self.L1 = 90 * skala
        self.L3 = 73 * skala
        self.L2 = 80 * skala

        # self.A = 17 * skala
        # self.B = 17 * skala
        # self.C = 55 * skala
        # self.D = 55 * skala
        # self.L1 = 90 * skala
        # self.L2 = 90 * skala

        # tracking the legs x,y
        self.Lxy = []

        # initial draw
        legend = self.makeLeg(self.sAlfa.get(), self.sBeta.get(),self.c)
        self.Lxy.append(legend[:])
        self.Lxy.append(legend[:])
        self.Lxy.append(legend[:])
        self.Lxy.append(legend[:])
        print(self.Lxy)


        self.l1_xp = tk.Button(self, text="(+)", command=lambda: self.legmove(1,1,0))
        self.l1_xp.grid(row=4,column=7)
        self.l1_xp = tk.Button(self, text="(-)", command=lambda: self.legmove(1,-1,0))
        self.l1_xp.grid(row=4,column=5)
        self.l1_xp = tk.Button(self, text="(+)", command=lambda: self.legmove(1,0,1))
        self.l1_xp.grid(row=3,column=6)
        self.l1_xp = tk.Button(self, text="(-)", command=lambda: self.legmove(1,0,-1))
        self.l1_xp.grid(row=5,column=6)

        self.uart2b = tk.Button(self, text="UART>>>>", command=self.uart2)
        self.uart2b.grid(row=6, column=6)

        tekst_komend = """
        <42,180,163,10,17,180,153,0,17>;
        <42,148,51,42,129,148,41,32,129>;
        <42,85,100,105,80,85,90,95,80>;
        <42,64,79,126,101,64,69,116,101>;
        <42,51,53,139,127,51,43,129,127>;
        <42,180,122,10,58,64,69,116,101>;
        <42,141,65,49,115,-1,-1,-1,-1>;
        <42,180,163,68,78,122,92,0,17>;
         <42,127,133,63,47,127,123,53,47>
        """
        nazwy_komend = """
        Lezec;
        Jamnik;
        Trop 0;
        Trop;
        Trop 2;
        Siad;
        Tylne nogi;
        Tymo;
        Malwa
        """

        self.lista_kom = tekst_komend.split(";")
        self.nazwy_kom = nazwy_komend.split(";")

        self.kom_btn = []
        for i,z in enumerate(zip(self.nazwy_kom, self.lista_kom)):
            n,k = z
            self.kom_btn.append(tk.Button(self, text=n.strip(), command=lambda msg=k: serial_snd(f"{msg.strip()}")))
            self.kom_btn[-1].grid(row=8+i,column=1)


        lista_komend = """
        <42,148,51,42,129,148,41,32,129>;
        <42,64,79,126,101,64,69,116,101>;
        <42,121,30,69,150,121,20,59,150>;
        <42,148,58,42,122,121,20,59,150>;
        <42,148,58,42,122,165,170,59,150>;
        <42,148,58,42,122,121,20,59,150>;
        <42,121,30,69,150,121,20,59,150>;
        <42,66,81,124,99,66,71,114,99>;
        <42,148,51,42,129,148,41,32,129>;
        """
        self.kom_btn.append(tk.Button(self,text="Sekwencja 1", command=lambda msg=lista_komend: serial_lista(msg, dt=800)))
        self.kom_btn[-1].grid(row=8,column=2)

        lista_komend = """
        <42,148,51,42,129,148,41,32,129>;
        <42,64,79,126,101,64,69,116,101>;
        <42,121,30,69,150,121,20,59,150>;
        <42,148,58,42,122,121,20,59,150>;

        <42,148,58,42,122,165,170,59,150>;
        <42,165,170,42,122,165,170,59,150>;
        <42,148,58,42,122,121,20,0,0>;
        <42,148,58,0,0,121,20,0,0>;
        <42,148,51,42,129,148,41,32,129>;

        <42,148,58,42,122,165,170,59,150>;
        <42,165,170,42,122,165,170,59,150>;
        <42,148,58,42,122,121,20,0,0>;
        <42,148,58,0,0,121,20,0,0>;
        <42,148,51,42,129,148,41,32,129>;

        <42,148,58,42,122,165,170,59,150>;
        <42,165,170,42,122,165,170,59,150>;
        <42,148,58,42,122,121,20,0,0>;
        <42,148,58,0,0,121,20,0,0>;
        <42,148,51,42,129,148,41,32,129>;
        """
        self.kom_btn.append(tk.Button(self,text="Sekwencja 2", command=lambda msg=lista_komend: serial_lista(msg, dt=200)))
        self.kom_btn[-1].grid(row=9,column=2)

        lista_komend = """
        <42,148,51,42,129,148,41,32,129>;
        <42,148,58,42,122,121,20,59,150>;
        <42,148,58,42,122,180,180,59,150>;
        <42,148,58,42,122,121,20,59,150>;
        <42,148,58,42,122,121,20,0,0>;
        """
        self.kom_btn.append(tk.Button(self,text="Lape daj", command=lambda msg=lista_komend: serial_lista(msg, dt=600)))
        self.kom_btn[-1].grid(row=10,column=2)

        lista_komend = """
        <42,90,61,100,119,90,51,90,119>;
<42,85,100,105,80,85,90,95,80>;
<42,85,100,105,80,85,90,38,122>;
<42,85,100,105,80,85,90,82,60>;
<42,117,65,73,115,117,55,82,60>;
<42,117,65,73,115,156,47,82,60>;
<42,117,65,73,115,137,88,82,60>;
<42,117,65,73,115,119,83,82,60>;
<42,117,65,73,115,144,64,36,106>;
<42,109,64,81,116,144,64,36,106>;
<42,109,64,81,116,144,64,62,109>;
<42,109,64,81,116,144,64,81,104>;
<42,109,64,72,60,144,64,81,104>;
<42,109,64,87,72,144,64,81,104>;
<42,109,64,87,72,144,64,41,86>;
<42,109,64,87,72,144,64,65,85>;
<42,109,64,87,72,114,111,66,59>;
<42,97,103,87,72,114,111,66,59>;
<42,90,61,100,119,90,51,90,119>
        """
        self.kom_btn.append(tk.Button(self,text="pastuch", command=lambda msg=lista_komend: serial_lista(msg, dt=300, n=5)))
        self.kom_btn[-1].grid(row=11,column=2)

        lista_komend = """
        <42,91,61,99,119,91,51,89,119>;
<42,110,104,99,119,91,51,89,119>;
<42,110,104,74,116,116,54,64,116>;
<42,95,116,64,119,116,54,64,116>;
<42,95,116,64,119,109,130,64,116>;
<42,95,116,64,119,129,113,64,116>;
<42,136,95,64,119,129,113,64,116>;
<42,136,95,64,119,129,113,43,42>;
<42,136,95,64,119,129,113,79,56>;
<42,115,122,64,119,115,112,79,56>;
<42,51,53,139,127,51,43,129,127>;
<42,51,53,86,93,51,43,129,127>;
<42,51,53,86,93,105,23,75,147>;
<42,99,92,86,93,105,23,75,147>;
<42,91,61,99,119,91,51,89,119>
        """
        self.kom_btn.append(tk.Button(self,text="pastuch", command=lambda msg=lista_komend: serial_lista(msg, dt=400, n=5)))
        self.kom_btn[-1].grid(row=12,column=2)

        self.kom_btn.append(tk.Button(self,text="pastuch", command=lambda msg=lista_komend: serial_lista(msg, dt=200, n=5,s=-1)))
        self.kom_btn[-1].grid(row=12,column=3)
    def legmove(self, Leg,dx,dy):

        legs = []
        step = 5

        if self.leg1.get():
            legs.append(0)
        if self.leg2.get():
            legs.append(1)
        if self.leg3.get():
            legs.append(2)
        if self.leg4.get():
            legs.append(3) 

        # print(legs)
        
        for l in legs:
            x = self.Lxy[l][0] + dx * step
            y = self.Lxy[l][1] + dy * step
        
            self.Lxy[l][0] = min(405,max(130,x))
            self.Lxy[l][1] = min(455,max(245,y))

        self.uart2()
        print(self.Lxy)
    
    def uart2(self):
        A = []
        B = []

        for l in range(4):
            x,y = self.Lxy[l]
            x = x - self.zeroX
            y = y - self.zeroY

            a,b = IK2(-x,y, self.L1,self.L2,self.L3,self.A,self.B,self.C,self.D)
            A.append(a)
            B.append(b)

        msg = setServos(A,B)
        serial_snd(msg)

        self.redraw()
        legend = self.makeLeg(A[0],B[0]-120,self.c)

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
        self.makeLeg(A,B,self.c)
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

    def drawBar(self, canv, x,y,L,alfa,clr="gray",thk=10):
        # method to draw the bar
        a = math.radians(alfa - 90)
        dy = L * math.cos(a)
        dx = L * math.sin(a)
        
        yend = y + dy
        xend = x + dx

        tmp = canv.create_line(x,y,xend,yend, width=thk, fill=clr)
        self.c_objects.append(tmp)

        return xend, yend
        ...
    
    def makeLeg(self, Alfa, Beta, canv):
        """
        Function to draw the simulated robo-dog-leg
        """
        point_list = []
        point_list.append((self.zeroX,self.zeroY))
        point_list.append((self.zeroX-self.C,self.zeroY))

        # 1st servo arm
        x0,y0 = self.drawBar(canv,self.zeroX - self.C, self.zeroY,self.A,Alfa,clr="red")
        point_list.append((x0,y0))
        # 2nd servo arm
        x1,y1 = self.drawBar(canv,self.zeroX, self.zeroY,self.A,Beta+120,clr="red")
        point_list.append((x1,y1))

        # leg part L1
        in_d = -2*self.A*self.C*math.cos(math.pi-math.radians(Alfa))+self.A**2+self.C**2
        d = math.sqrt(in_d)
        fi1 = math.acos((d**2+self.C**2-self.A**2)/(2*d*self.C)) 
        fi2 = math.acos((d**2+self.B**2-self.D**2)/(2*d*self.B)) 
        Beta1 = fi1 + fi2
        Beta1 = math.degrees(Beta1)

        x2,y2 = self.drawBar(canv,self.zeroX, self.zeroY, self.L1, Beta1)
        point_list.append((x2,y2))

        # leg part 2
        in_w = self.L1**2 + self.A**2 -2*self.L1*self.A*math.cos(math.radians(Beta+120)-math.radians(Beta1))
        w = math.sqrt(in_w)

        fii1 = math.acos((self.L1**2+w**2 - self.A**2)/(2*self.L1*w))
        fii2 = math.acos((self.B**2+w**2-self.L3**2)/(2*self.B*w))

        dzetta = math.degrees(fii1 + fii2)

        psi = Beta1 - dzetta 

        xe,ye = self.drawBar(canv,x2,y2,self.L2,psi)
        point_list.append((xe,ye))

        x4,y4 = self.drawBar(canv,x2,y2,self.B,psi+180)
        point_list.append((x4,y4))

        # calculations for the connection arms
        d_angle = math.acos((self.B**2+self.D**2-d**2)/(2*self.D*self.B))
        d_angle = math.degrees(d_angle)

        d_angle = Beta1 + d_angle
        x5,y5 = self.drawBar(canv,x0,y0,self.D,d_angle,clr="green")
        point_list.append((x5,y5))

        w_angle = math.acos((self.B**2+self.L3**2-w**2)/(2*self.B*self.L3))
        w_angle = math.degrees(w_angle)

        x6,y6 = self.drawBar(canv,x4,y4,self.L3,psi+180+180-w_angle, clr="green")
        point_list.append((x6,y6))

        xe = int(xe)
        ye = int(ye)
        # print(f"x,y: ({xe},{xe})")

        for pt in point_list:
            x,y = pt
            tmp = self.c.create_oval(x-5,y-5,x+5,y+5, fill="silver")
            self.c_objects.append(tmp)

        return [xe, ye]

    def update(self, event):
        if (self.klikx != event.x) or (self.kliky != event.y):
            self.klikx = event.x
            self.kliky = event.y


            x = event.x - self.zeroX
            y = event.y - self.zeroY
            # x = x / (2*self.A)
            # y = y / (2*self.A)
            # print(x,y)

            A,B = IK2(-x,y, self.L1,self.L2,self.L3,self.A,self.B,self.C,self.D)

            # print(A, B-120)

            self.redraw()
            legend = self.makeLeg(A,B-120,self.c)
            self.sAlfa.set(int(A))
            self.sBeta.set(int(B-120))

            for l in range(4):
                self.Lxy[l] = legend[:]
        
        ...


print('Search...')
ports = serial.tools.list_ports.comports(include_links=False)
for port in ports :
    print('Find port '+ port.device)

try:
    ser = serial.Serial(
        port='/dev/cu.usbserial-0001',
        # port='/dev/cu.usbserial-120',
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