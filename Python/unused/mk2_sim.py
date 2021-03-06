# A simple object oriented tkinter window
import tkinter as tk
from tkinter import ttk
import math
from tkinter.constants import E, W, YES
import serial
import serial.tools.list_ports
from PIL import ImageTk, Image

import asyncio
from bleak import BleakClient, BleakScanner 
from bleak.exc import BleakError
import time
from time import sleep, thread_time
 
# -- Windows only configuration --
try:
    from ctypes import windll
    windll.shcore.SetProcessDpiAwareness(1)
except:
    pass
    
class myBT:

    def __init__(self):
        self.BTsetup();
        self.connect();


    def BTsetup(self):
        self.the_device = ""
        self.the_service = ""
        self.client = False
        self.loop = False

        self.is_serial = False
        self.is_BLE = False


    async def BTsearch(self):
        devices = await BleakScanner.discover()
        for i,d in enumerate(devices):
            print(f"[{i}]\t{d.name}\t{d.address}")
            if "BT05" in d.name:
                print(f"Potenitial robot found @ {d.address}")
                self.the_device = d.address


    async def BTgetServices(self ):
        device = await BleakScanner.find_device_by_address(self.the_device, timeout=20.0)
        
        if not device:
            raise BleakError(f"A device with address {self.the_device} could not be found.")

        async with BleakClient(device) as client:
            svcs = await client.get_services()
            print("Services:")
            for service in svcs:
                print(service)
                if "Vendor specific" in str(service):
                    print(service.characteristics)
                    for inside in service.characteristics:
                        print(f"Sub info properties: {inside.properties}")
                        print(f"Sub info uuid: {inside.uuid}")
                        self.the_service = inside.uuid

    async def BTconnect(self):
        """
        The class method to connecto tot the BT robot
        Based on the blake library for the BLE device operations.
        """

        print("Connecting...")
        print("trying direct BLE connection...")

        self.client = BleakClient(self.the_device,timeout=10)
        await self.client.connect()
        connection_status = self.client.is_connected
        print(f"Connection status: {connection_status}")

        if connection_status:
            print("Connected mode..")
            self.is_serial = False
            self.is_BLE = True
        else:
            self.is_serial = False
            self.is_BLE = False

    async def BTdisconnect(self):
            if self.client:
                if self.client.is_connected:
                    await self.client.disconnect()
                    print("The robot have been disconnected...")
                else:
                    print("No robot to be disconnected!")
            else:
                print("No robot to be disconnected!")

    async def BTwrite(self, the_command, redial=True):
        if self.client.is_connected:
            await self.client.write_gatt_char(self.the_service,bytearray(the_command, "utf-8"), response=not True)
        else:
            print("No devce connected.")
            if redial and self.the_service:
                self.loop.run_until_complete(self.BTconnect()) 

    def connect(self):
        if not self.client:
            print("Scanning for BLE devices...")
            self.loop = asyncio.get_event_loop()
            self.loop.run_until_complete(self.BTsearch())

            if self.the_device:
                print(f"there is Mariola at {self.the_device}")
                self.loop.run_until_complete(self.BTgetServices())
                if self.the_service:
                    print(f"Found Vendor sercvice at {self.the_service}")
                    self.loop.run_until_complete(self.BTconnect())
            else:
                print("No Aron BT05 found :(")
                ...
        else:
            print("Shall be already connected...")



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

def limit(x,min_=-1, max_=1):
    return min(max(min_,x),max_)

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
    msg += "\n\r"

    if is_serial:
        ser.write(msg.encode())
        print(f"Serial: {msg}")
    else:
        print(f"SerialSim: {msg}")

    
    # tmp_command = int(msg[1:3])
    # if tmp_command == 42:
    
    #     temp_servo = msg[4:-1].split(",")
    #     for i,s in enumerate(temp_servo):
    #         s = int(s)
    #         if s != -1:
    #             last_servos[i] = s
    #     print(last_servos)

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

        self.geometry("950x550")
        self.title("FLR Leg Simulator")


        #Setting it up
        img = ImageTk.PhotoImage(Image.open("img/aronmk2.png").resize((150,75)).convert('RGB'))

        label = ttk.Label(self, text=" . ")
        label.config(font=("Arial",20))
        label.grid(row=1,column=1,columnspan=1, sticky="we")
        # label.image = img


        c0 = 1
        r0 = 3

        label = ttk.Label(self, text="FLR IK Simulator v0.3 for mk2")
        label.config(font=("Arial",20))
        label.grid(row=1,column=c0+2,columnspan=10, sticky="we")



        self.pad = tk.Canvas(self, bg="silver", width=200, height=200)
        self.pad.grid(row=r0+11, column=c0+1,rowspan=10, columnspan=4)
        self.pad.bind("<B1-Motion>", self.getpad)
        self.pad_obj = []

        self.sSwing = tk.Scale(self, from_=-50, to=50, orient=tk.HORIZONTAL, command=self.swing)
        self.sSwing.grid(row=r0+4,column=c0+2,columnspan=3, sticky='we')
        self.sSwing.set(0)

        self.sFB = tk.Scale(self, from_=-20, to=40, orient=tk.HORIZONTAL, command=self.swing)
        self.sFB.grid(row=r0+6,column=c0+2,columnspan=3, sticky='we')
        self.sFB.set(0)

        self.sUP = tk.Scale(self, from_=40, to=-40, orient=tk.VERTICAL, command=self.setUp)
        self.sUP.grid(row=r0+11,column=c0+5,columnspan=1, rowspan=10, sticky=tk.N+tk.S)
        self.sUP.set(0)
        self.last_up = 0;

        self.sDrv = tk.Scale(self, from_=150, to=-150, orient=tk.VERTICAL, command=lambda a: self.drive())
        self.sDrv.grid(row=r0+11,column=c0+6,columnspan=1, rowspan=10, sticky=tk.N+tk.S)
        self.sDrv.set(0)
        self.sDrv.bind("<ButtonRelease-1>", self.stopdrive)
        
        self.bHome = tk.Button(text="Home Pose", command=self.homming)
        self.bHome.grid(row=r0+3, column=c0+1, columnspan=2)

        self.bHome = tk.Button(text="Val.Reset", command=self.reset)
        self.bHome.grid(row=r0+3, column=c0+4, columnspan=2)


        # making the controls for the individual legs. 
        label = ttk.Label(self, text="Individual leg's")
        label.config(font=("Arial",20))
        label.grid(row=r0+3,column=15,columnspan=5, sticky="we")

        img = ImageTk.PhotoImage(Image.open("img/aronmk2T.png").resize((200,200)).convert('RGB'))
        button = tk.Button(self, image=img )
        button.grid(row=7, column=20, columnspan=1, rowspan=12)
        button.image = img

        self.legs_up = []
        self.legs_fb = []
        srow = 5
        scol = 13
        for L in range(1,5):
            if L in [1,3]:
                r = srow + (L // 3) * 6
                c = scol 
            else:
                r = srow + ((L-1) // 3) * 6
                c = scol + 10


            self.legs_up.append(tk.Scale(self, from_=40, to=-40, orient=tk.VERTICAL, command=lambda x, a=L: self.moveLeg(a)))
            self.legs_up[-1].grid(row=r0+r,column=c0+c,columnspan=1, rowspan=3, sticky=tk.N+tk.S)
            self.legs_up[-1].set(0)

            self.legs_fb.append(tk.Scale(self, from_=-20, to=20, orient=tk.HORIZONTAL, command=lambda x, a=L: self.moveLeg(a)))
            self.legs_fb[-1].grid(row=r0+r,column=c0+c+1,columnspan=3, rowspan=1, sticky=tk.E+tk.W)
            self.legs_fb[-1].set(0)

        self.getpad();


    def drive(self):
        drv = self.sDrv.get()
        sign = drv/abs(drv)
        drv = drv + sign*50
        drv = limit(drv,min_=-254,max_=254)
        msg = f"drv {drv} {drv}"
        serial_snd(msg)

    def stopdrive(self, *arg):
        msg = "drv 0 0"
        serial_snd(msg)
        self.sDrv.set(0)

    def moveLeg(self,leg):
        up = self.legs_up[leg-1].get()
        fb = self.legs_fb[leg-1].get()

        msg = f"leg {leg} {up} {fb} 0"
        serial_snd(msg)

        print(f"Will move the {leg} by: {msg}")

    def reset(self, *args):
        for p in self.legs_fb:
            p.set(0)
        for p in self.legs_up:
            p.set(0)
        self.last_up = 0
        self.sUP.set(0)
        msg = "reset"
        serial_snd(msg)

    def getpad(self, ev=False):
        
        if len(self.pad_obj):
            for obj in self.pad_obj:
                self.pad.delete(obj)
        
        self.pad_obj.append(self.pad.create_line(0,100,200,100))
        self.pad_obj.append(self.pad.create_line(100,0,100,200))

        if not ev:
            x = 100
            y = 100
        else:
            x = ev.x
            y = ev.y
            
        x = limit(x,min_=0, max_=200)
        y = limit(y,min_=0, max_=200)
        
        dx = x - 100
        dy = y - 100

        self.pad_obj.append(self.pad.create_oval(x-10,y-10,x+10,y+10,fill="red"))
        A = int(50*dx/100)
        B = int(25*dy/100)

        old_A = self.sSwing.get() 
        old_B = self.sFB.get()

        if (abs(A - old_A) > 4):
            self.sSwing.set(A)
        if (abs(B - old_B) > 4):
            self.sFB.set(B)

    def setUp(self, *args):
        UP = self.sUP.get()
        if abs(UP - self.last_up) > 5:
            msg = f"up {int(UP)}"
            serial_snd(msg)
            self.last_up = UP

    def homming(self, *args):
        self.sSwing.set(0)
        self.sFB.set(0)
        self.getpad()

        serial_snd("h")

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

    def swing(self, _):
        A = -self.sSwing.get();
        B = -self.sFB.get();
        # msg = f"swing {A}"
        msg = f"circ {B} {A}"
        serial_snd(msg)



    def twist(self, _):
        A = -self.sTwist.get();
        msg = f"twist {A}"
        serial_snd(msg)


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
        baudrate=9600,
        #baudrate=115200,
        # parity=serial.PARITY_ODD,
        # stopbits=serial.STOPBITS_TWO,
        # bytesize=serial.EIGHTBITS
    )
    print(ser.name)         # check which port was really used
    is_serial = True
except:
    is_serial = False
    print("No Serial Mode")
    print("trying BT...")
    BT = myBT()


time.sleep(2)
debug = 0
root = HelloWorld()
root.mainloop()

if is_serial:
    ser.close()             # close port
    print("Closing the serial port")