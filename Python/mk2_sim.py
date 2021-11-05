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

def my_print(input_msg):
    if debug:
        print(input_msg)
    else:
        ...

def limit(x):
    return min(max(-1,x),1)

def leg_IK_mk2(point, A=17,C=20.54,E=10.25,F=35,G=60,H=90,N=55,D=20.45,B=17,M=35,L=70, be_precise=False):
    """
    Use all dimensions in [mm]
    solwing for the Alpha0, Alpha, Phi - returned in degrees.
    this is the first servo part of the solve. 

    dimension of the leg parts
    please refer to the mk2_leg picture in the doc

    for the 1st part:
    A - servo 1 arm in [mm]
    C - distance from servo 1 to leg axis - in x
    E - distance from servo 2 to leg axis - in y
    F - leg top part - distance from axis to puler point
    G - servo 1 puller length

    H - leg top part length
    N - leg bottom part working length

    input is the point as (x,y) tuple position of the leg tip
    """

    x,y = point
    x = -x
    k = math.sqrt(E**2 + C**2)

    # the reasonable assumption due to geometry 
    # is to ok to have the y > 0
    if y > 0:
          
        h = math.sqrt(x**2 + y**2) 

        dzetta = math.atan2(x,y)
        my_print(f"dzetta={math.degrees(dzetta)}")

        Phi = math.acos(limit((H**2+N**2-h**2)/(2*H*N)))
        my_print(math.degrees(Phi))

        in_0 = (H**2+h**2-N**2)/(2*H*h)
        if not (-1 <= in_0 <= 1):
            return False
        # safety limiting
        in_0 = limit(in_0)
        Psi = math.acos(in_0)
        my_print(f"Psi= {math.degrees(Psi)}")

        Alpha = math.radians(90) + dzetta + Psi
        my_print(math.degrees(Alpha))

        i2  = k**2 + F**2 -2*k*F*math.cos(Alpha + 0.5*math.pi - math.acos(E/k))
        i = math.sqrt(i2)
        my_print(f"{i=}")

        in_1 = (A**2+i2-G**2)/(2*A*i)
        if not (-1 <= in_1 <= 1):
            return False
        my_print(f"{in_1=}")
        in_1 = limit(in_1)

        in_2 = (k**2+i2-F**2)/(2*k*i)
        if not (-1 <= in_2 <= 1):
            return False
        in_2 = limit(in_2)

        k1 = math.acos(in_1)
        k2 = math.acos(in_2)
        k3 = math.atan2(E,C)
        my_print(f"k1={math.degrees(k1)}")
        my_print(f"k2={math.degrees(k2)}")
        my_print(f"k3={math.degrees(k3)}")

        Alpha0 = math.pi - k1 - k2 - k3

        # the part for the second servo
        my_print("2nd servo")
        p2 =  E**2 + D**2
        p = math.sqrt( p2 )
        k1 = math.acos( E/p )
        my_print(f"k1={math.degrees(k1)}")
        eta = math.pi - Psi - dzetta - k1
        my_print(f"eta={math.degrees(eta)}")

        R2 = p2 + H**2 -2*p*H*math.cos(eta)
        R = math.sqrt( R2 )
        my_print(f"{R=}")
        k6 = math.acos(limit((H**2+R2 - p2)/(2*H*R)))
        my_print(f"k6={math.degrees(k6)}")

        k2 = math.pi - Phi
        my_print(f"k2={math.degrees(k2)}")
        k5 = k2 - k6
        my_print(f"k5={math.degrees(k5)}")

        Q2 = R2 + M**2 -2*R*M*math.cos(k5)
        Q = math.sqrt( Q2 )
        my_print(f"{Q=}")
        Hy = E + H*math.cos(Psi+dzetta)
        my_print(f"{Hy=}")

        kd   = math.acos(limit((Q2+B**2-L**2)/(2*Q*B)))
        my_print(f"k.={math.degrees(kd)}")
        kdd  = math.acos(limit((Q2+R2-M**2)/(2*Q*R)))
        my_print(f"k..={math.degrees(kdd)}")
        kddd = math.acos(limit((Hy)/(R)))
        my_print(f"k...={math.degrees(kddd)}")

        Beta = kd + kdd + kddd + math.pi/2
    else:
        return False
    
    if not be_precise:
        return int(math.degrees(Alpha0)),int(math.degrees(Beta)), int(math.degrees(Alpha)-90), int(math.degrees(Phi))

    return (math.degrees(Alpha0)),(math.degrees(Beta)), (math.degrees(Alpha)-90), (math.degrees(Phi))


def leg_DK_mk2(angles, A=17,C=20.54,E=10.25,F=35,G=60,H=90,N=55,D=20.45,B=17,M=35,L=70, be_precise=False):
    my_print("")
    my_print("DirectK")
    Alpha, Beta = angles
    Alpha = math.radians(Alpha)
    Beta = math.radians(Beta)
    
    k2 = E**2 + C**2
    k = math.sqrt(k2)
    kd = math.acos(C/k)
    kdd = math.pi - kd - Alpha

    One2 = A**2 + k2 -2*A*k*math.cos(kdd)
    One = math.sqrt(One2)

    kddd = math.acos(limit((G**2+F**2 - One2)/(2*G*F)))
    k4 = math.acos(limit((One2+F**2-G**2)/(2*One*F)))
    k5 = math.acos(limit((One2+k2-A**2)/(2*One*k)))

    Psi = k4+k5-kd - math.pi/2
    my_print(f"Psi={math.degrees(Psi)}")
    
    # now for the Phi angle
    k2 = D**2 + E**2
    k = math.sqrt(k2)
    kd = math.acos(D/k)
    kdd = math.acos(E/k)

    dzt2 = k2 + B**2 -2*k*B*math.cos(Beta - kd)
    dzt = math.sqrt(dzt2)
    my_print(f"{dzt=}")

    in_0 = (k2+dzt2 - B**2)/(2*k*dzt)
    ks = math.acos(limit(in_0))
    # figuring out the other side
    if Beta > (math.pi + 0.5*math.pi- kdd ):
        ks *= -1

    k1 = math.pi - Psi  - kdd - ks 

    eta2 = H**2 + dzt2 - 2*H*dzt*math.cos(k1)
    eta = math.sqrt(eta2)
    my_print(f"{eta=}")

    in_1 = (H**2+eta2 - dzt2)/(2*H*eta)
    kx1 = math.acos(limit(in_1))
    my_print(f"kx1={math.degrees(kx1)}")
    in_2 = (eta2+M**2-L**2)/(2*eta*M)
    my_print(f"{in_2=}")
    kx2 = math.acos(limit(in_2))
    my_print(f"kx2={math.degrees(kx2)}")

    Phi = math.pi - kx1 - kx2

    # calculation of the end coords x,y
    h2 = H**2 + N**2 -2*H*N*math.cos(Phi)
    h = math.sqrt(h2)
    km = math.acos((h2+H**2-N**2)/(2*h*H))
    k_xy = Psi - km
    x = -h * math.sin(k_xy)
    y = h * math.cos(k_xy)

    return int(math.degrees(Psi)), int(math.degrees(Phi)), x, y


def serial_snd(msg):
    global undo_servos
    undo_servos[0] = last_servos[:]

    if is_serial:
        ser.write(msg.encode())
        print(f"Serial: {msg}")
    else:
        print(f"SerialSim: {msg}")

    
    tmp_command = int(msg[1:3])
    if tmp_command == 42:
    
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

        label = ttk.Label(self, text="FLR IK Simulator v0.3 for mk2")
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


        self.sAlfa = tk.Scale(self, from_=20, to=120, orient=tk.HORIZONTAL, command=self.sliders)
        self.sAlfa.grid(row=22,column=10,columnspan=2)
        self.sAlfa.set(90)
        self.sBeta = tk.Scale(self, from_=0, to=180, orient=tk.HORIZONTAL, command=self.sliders)
        self.sBeta.grid(row=22,column=12,columnspan=2)
        self.sBeta.set(90)

        # self.leg1 = tk.IntVar(value=1)
        # self.leg2 = tk.IntVar(value=1)
        # self.leg3 = tk.IntVar(value=1)
        # self.leg4 = tk.IntVar(value=1)

        # L1 = tk.Checkbutton(self, text='Leg 1',variable=self.leg1, onvalue=1, offvalue=0)
        # L1.grid(row=4,column=1)
        # L2 = tk.Checkbutton(self, text='Leg 2',variable=self.leg2, onvalue=1, offvalue=0)
        # L2.grid(row=4,column=2)
        # L3 = tk.Checkbutton(self, text='Leg 3',variable=self.leg3, onvalue=1, offvalue=0)
        # L3.grid(row=5,column=1)
        # L4 = tk.Checkbutton(self, text='Leg 4',variable=self.leg4, onvalue=1, offvalue=0)
        # L4.grid(row=5,column=2)

        # self.uartBtn = tk.Button(self, text ="UART >>>", command = self.uart)
        # self.uartBtn.grid(row=6,column=2,columnspan=1)

        # self.uartBtn = tk.Button(self, text ="Save>>", command =lambda: moves.append(last_servos[:]))
        # self.uartBtn.grid(row=6,column=3,columnspan=1)

        # self.uartBtn = tk.Button(self, text ="Show", command =lambda: print(moves))
        # self.uartBtn.grid(row=6,column=4,columnspan=1)

        # self.uartBtn = tk.Button(self, text ="Del.Last", command =del_last)
        # self.uartBtn.grid(row=6,column=5,columnspan=1)

        # self.uartBtn = tk.Button(self, text ="Make!", command =make_moves)
        # self.uartBtn.grid(row=7,column=3,columnspan=1)

        # self.uartBtn = tk.Button(self, text ="Print", command =print_moves)
        # self.uartBtn.grid(row=7,column=4,columnspan=1)

        # self.uartBtn = tk.Button(self, text ="Undo Lst", command =undo_last)
        # self.uartBtn.grid(row=7,column=5,columnspan=1)
        self.redraw()

        self.Lxy = []

        # initial draw
        legend = self.makeLeg(self.sAlfa.get(), self.sBeta.get(),self.c)
        self.Lxy.append(legend[:])
        self.Lxy.append(legend[:])
        self.Lxy.append(legend[:])
        self.Lxy.append(legend[:])
        print(self.Lxy)


#         self.l1_xp = tk.Button(self, text="(+)", command=lambda: self.legmove(1,1,0))
#         self.l1_xp.grid(row=4,column=7)
#         self.l1_xp = tk.Button(self, text="(-)", command=lambda: self.legmove(1,-1,0))
#         self.l1_xp.grid(row=4,column=5)
#         self.l1_xp = tk.Button(self, text="(+)", command=lambda: self.legmove(1,0,1))
#         self.l1_xp.grid(row=3,column=6)
#         self.l1_xp = tk.Button(self, text="(-)", command=lambda: self.legmove(1,0,-1))
#         self.l1_xp.grid(row=5,column=6)

#         self.uart2b = tk.Button(self, text="UART>>>>", command=self.uart2)
#         self.uart2b.grid(row=6, column=6)

#         tekst_komend = """
#         <42,180,163,10,17,180,153,0,17>;
#         <42,148,51,42,129,148,41,32,129>;
#         <42,85,100,105,80,85,90,95,80>;
#         <42,64,79,126,101,64,69,116,101>;
#         <42,51,53,139,127,51,43,129,127>;
#         <42,180,122,10,58,64,69,116,101>;
#         <42,141,65,49,115,-1,-1,-1,-1>;
#         <42,180,163,68,78,122,92,0,17>;
#          <42,127,133,63,47,127,123,53,47>
#         """
#         nazwy_komend = """
#         Lezec;
#         Jamnik;
#         Trop 0;
#         Trop;
#         Trop 2;
#         Siad;
#         Tylne nogi;
#         Tymo;
#         Malwa
#         """

#         self.lista_kom = tekst_komend.split(";")
#         self.nazwy_kom = nazwy_komend.split(";")

#         self.kom_btn = []
#         for i,z in enumerate(zip(self.nazwy_kom, self.lista_kom)):
#             n,k = z
#             self.kom_btn.append(tk.Button(self, text=n.strip(), command=lambda msg=k: serial_snd(f"{msg.strip()}")))
#             self.kom_btn[-1].grid(row=8+i,column=1)


#         lista_komend = """
#         <42,148,51,42,129,148,41,32,129>;
#         <42,64,79,126,101,64,69,116,101>;
#         <42,121,30,69,150,121,20,59,150>;
#         <42,148,58,42,122,121,20,59,150>;
#         <42,148,58,42,122,165,170,59,150>;
#         <42,148,58,42,122,121,20,59,150>;
#         <42,121,30,69,150,121,20,59,150>;
#         <42,66,81,124,99,66,71,114,99>;
#         <42,148,51,42,129,148,41,32,129>;
#         """
#         self.kom_btn.append(tk.Button(self,text="Sekwencja 1", command=lambda msg=lista_komend: serial_lista(msg, dt=800)))
#         self.kom_btn[-1].grid(row=8,column=2)

#         lista_komend = """
#         <42,148,51,42,129,148,41,32,129>;
#         <42,64,79,126,101,64,69,116,101>;
#         <42,121,30,69,150,121,20,59,150>;
#         <42,148,58,42,122,121,20,59,150>;

#         <42,148,58,42,122,165,170,59,150>;
#         <42,165,170,42,122,165,170,59,150>;
#         <42,148,58,42,122,121,20,0,0>;
#         <42,148,58,0,0,121,20,0,0>;
#         <42,148,51,42,129,148,41,32,129>;

#         <42,148,58,42,122,165,170,59,150>;
#         <42,165,170,42,122,165,170,59,150>;
#         <42,148,58,42,122,121,20,0,0>;
#         <42,148,58,0,0,121,20,0,0>;
#         <42,148,51,42,129,148,41,32,129>;

#         <42,148,58,42,122,165,170,59,150>;
#         <42,165,170,42,122,165,170,59,150>;
#         <42,148,58,42,122,121,20,0,0>;
#         <42,148,58,0,0,121,20,0,0>;
#         <42,148,51,42,129,148,41,32,129>;
#         """
#         self.kom_btn.append(tk.Button(self,text="Sekwencja 2", command=lambda msg=lista_komend: serial_lista(msg, dt=200)))
#         self.kom_btn[-1].grid(row=9,column=2)

#         lista_komend = """
#         <42,148,51,42,129,148,41,32,129>;
#         <42,148,58,42,122,121,20,59,150>;
#         <42,148,58,42,122,180,180,59,150>;
#         <42,148,58,42,122,121,20,59,150>;
#         <42,148,58,42,122,121,20,0,0>;
#         """
#         self.kom_btn.append(tk.Button(self,text="Lape daj", command=lambda msg=lista_komend: serial_lista(msg, dt=600)))
#         self.kom_btn[-1].grid(row=10,column=2)

#         lista_komend = """
#         <42,90,61,100,119,90,51,90,119>;
# <42,85,100,105,80,85,90,95,80>;
# <42,85,100,105,80,85,90,38,122>;
# <42,85,100,105,80,85,90,82,60>;
# <42,117,65,73,115,117,55,82,60>;
# <42,117,65,73,115,156,47,82,60>;
# <42,117,65,73,115,137,88,82,60>;
# <42,117,65,73,115,119,83,82,60>;
# <42,117,65,73,115,144,64,36,106>;
# <42,109,64,81,116,144,64,36,106>;
# <42,109,64,81,116,144,64,62,109>;
# <42,109,64,81,116,144,64,81,104>;
# <42,109,64,72,60,144,64,81,104>;
# <42,109,64,87,72,144,64,81,104>;
# <42,109,64,87,72,144,64,41,86>;
# <42,109,64,87,72,144,64,65,85>;
# <42,109,64,87,72,114,111,66,59>;
# <42,97,103,87,72,114,111,66,59>;
# <42,90,61,100,119,90,51,90,119>
#         """
#         self.kom_btn.append(tk.Button(self,text="pastuch", command=lambda msg=lista_komend: serial_lista(msg, dt=300, n=5)))
#         self.kom_btn[-1].grid(row=11,column=2)

#         lista_komend = """
#         <42,91,61,99,119,91,51,89,119>;
# <42,110,104,99,119,91,51,89,119>;
# <42,110,104,74,116,116,54,64,116>;
# <42,95,116,64,119,116,54,64,116>;
# <42,95,116,64,119,109,130,64,116>;
# <42,95,116,64,119,129,113,64,116>;
# <42,136,95,64,119,129,113,64,116>;
# <42,136,95,64,119,129,113,43,42>;
# <42,136,95,64,119,129,113,79,56>;
# <42,115,122,64,119,115,112,79,56>;
# <42,51,53,139,127,51,43,129,127>;
# <42,51,53,86,93,51,43,129,127>;
# <42,51,53,86,93,105,23,75,147>;
# <42,99,92,86,93,105,23,75,147>;
# <42,91,61,99,119,91,51,89,119>
#         """
#         self.kom_btn.append(tk.Button(self,text="pastuch", command=lambda msg=lista_komend: serial_lista(msg, dt=400, n=5)))
#         self.kom_btn[-1].grid(row=12,column=2)

#         lista_komend = """
#         <42,119,70,71,110,119,60,61,110>;
# <42,119,70,44,118,119,60,61,110>;
# <42,119,70,44,118,174,64,61,110>;
# <42,119,70,44,118,112,108,61,110>;
# <42,119,70,71,111,112,108,61,110>;
# <42,119,70,71,111,112,108,74,68>;
# <42,103,70,87,110,112,108,74,68>;
# <42,103,70,87,110,150,84,30,86>;
# <42,87,55,103,125,150,84,30,86>;
# <42,87,55,103,125,112,76,68,94>;
# <42,71,58,119,122,71,48,109,122>;
# <42,139,39,119,122,71,48,109,122>;
# <42,97,97,119,122,71,48,109,122>;
# <42,76,83,119,122,110,40,109,122>;
# <42,76,83,92,91,110,40,109,122>;
# <42,76,83,92,91,83,68,109,122>
#         """
#         self.kom_btn.append(tk.Button(self,text="drunk master", command=lambda msg=lista_komend: serial_lista(msg, dt=200, n=1,s=1)))
#         self.kom_btn[-1].grid(row=12,column=3)


#         self.sRamp = tk.Scale(self, from_=50, to=99, orient=tk.HORIZONTAL, command=self.ramp)
#         # self.sBeta.pack()
#         self.sRamp.grid(row=11,column=4,columnspan=2)
#         self.sRamp.set(55)

    def ramp(self, event):
        rap = int(self.sRamp.get() * 10)
        rap = 1490 - rap
        serial_snd(f"<45,{rap},0,0,0,0,0,0,0>")

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
    
    def drawBarP(self, canv, x,y,xe,ye, Lx=1,clr="gray",thk=10):
        # method to draw the bar
        
        yend = ye
        xend = xe

        L2 = (xe-x)**2 + (ye-y)**2
        L = math.sqrt(L2)

        if int(L) > Lx+2:
            clr = "magenta" 

        tmp = canv.create_line(x,y,xend,yend, width=thk, fill=clr)
        self.c_objects.append(tmp)

        return L

    def makeLeg(self, Alfa, Beta, canv):
        """
        Function to draw the simulated robo-dog-leg
        """
        Alfa -= 45
        Beta += 130

        A=17
        C=20.54
        E=10.25
        F=35
        G=60
        H=90
        N=55
        D=20.45
        B=17
        M=35
        L=70

        skala = 2.5
        self.skala = skala

        x0 = self.zeroX
        y0 = self.zeroY

        point_list = []
        point_list.append((x0,y0))

        # 1st servo arm
        x = x0 - C*skala
        y = y0 - E*skala

        point_list.append((x,y))
        R = A * skala

        x_1,y_1 = self.drawBar(canv,x, y, R, Alfa, clr="red")
        point_list.append((x_1,y_1))

        # 2nd servo arm
        x = x0 + D*skala
        y = y0 - E*skala
        point_list.append((x,y))

        R = B * skala

        x_,y_ = self.drawBar(canv,x, y, R, Beta, clr="red")
        point_list.append((x_,y_))

        # grabbing stuff from the dK model
        Psi,Phi,xe,ye = leg_DK_mk2((Alfa,Beta))

        # leg part H
        R = H * skala
        x2,y2 = self.drawBar(canv,x0, y0, R, Psi + 90)
        point_list.append((x2,y2))

        # leg part F
        R = F * skala
        xf,yf = self.drawBar(canv,x0, y0, R, Psi + 90)
        point_list.append((xf,yf))

        self.drawBarP(canv,xf,yf,x_1,y_1,Lx=G*skala)
        # leg part L2
        R = N * skala
        x3,y3 = self.drawBar(canv,x2, y2, R, Psi + 90 -180 + Phi)
        point_list.append((x3,y3))
        
        R = M * skala
        x3,y3 = self.drawBar(canv,x2, y2, R, Psi + 90 + Phi)
        point_list.append((x3,y3))

        self.drawBarP(canv, x3,y3,x_,y_,Lx=L*skala)

        xe = int(xe)
        ye = int(ye)

        for pt in point_list:
            x,y = pt
            tmp = self.c.create_oval(x-5,y-5,x+5,y+5, fill="silver")
            self.c_objects.append(tmp)

        return [xe, ye]

    def update(self, event):
        if (self.klikx != event.x) or (self.kliky != event.y):
            self.klikx = event.x
            self.kliky = event.y


            x = int(-(event.x - self.zeroX)/self.skala)
            y = int((event.y - self.zeroY)/self.skala)
            # x = x / (2*self.A)
            # y = y / (2*self.A)
            # print(x,y)

            # A,B = IK2(-x,y, self.L1,self.L2,self.L3,self.A,self.B,self.C,self.D)
            if leg_IK_mk2((x,y)):
                A,B,_,_ = leg_IK_mk2((x,y))

                print(f"{x=} {y=}")
                # print(f"{A=} {B=}")

                # print(A, B-120)
                # Alfa -= 45
                # Beta += 130

                self.redraw()
                legend = self.makeLeg(A,B,self.c)
                self.sAlfa.set(int(A+45))
                self.sBeta.set(int(B-130))

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
debug = 0
root = HelloWorld()
root.mainloop()

if is_serial:
    ser.close()             # close port
    print("Closing the serial port")