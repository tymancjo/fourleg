# A simple object oriented tkinter window
import tkinter as tk
from tkinter import ttk
import math
from tkinter.constants import E, W
 
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
    print(M)
    if M <= 2*A:
        Alfa = math.pi - math.asin(x/M) - math.asin(M/(2*A))
        Beta = math.pi + math.asin(M/(2*A)) - math.asin(x/M)

        Alfa = math.degrees(Alfa)
        Beta = math.degrees(Beta) - 120

        Alfa = max(15,min(165,Alfa))
        Beta = max(15,min(165,Beta))
    
    return Alfa, Beta

    
class HelloWorld(tk.Tk):
    def __init__(self):
        super().__init__()

        self.geometry("640x650")
        self.title("FLR Leg Simulator")

        label = ttk.Label(self, text="FLR IK Simulator v0.1")
        label.config(font=("Arial",20))
        label.pack()

        self.W = 500
        self.H = 500

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


        self.redraw()

        # base data for the leg
        self.A = 120
        self.servo_R = 50
        self.servo_space = 100

        # initial draw
        self.makeLeg(self.sAlfa.get(), self.sBeta.get())

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

    def drawBar(self, x,y,L,alfa,clr="gray",thk=15):
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
        # 1st servo arm
        x0,y0 = self.drawBar(self.zeroX - self.servo_space, self.zeroY,self.servo_R,Alfa,clr="red")
        # 2nd servo arm
        self.drawBar(self.zeroX, self.zeroY,self.servo_R,Beta+120,clr="red")
        # 1st servo bar
        self.drawBar(x0,y0,self.servo_space,180,clr="silver")
        #1st part of leg
        x0,y0 = self.drawBar(self.zeroX, self.zeroY,self.A,Alfa,clr="green")
        # 2nd part of leg
        self.drawBar(x0,y0,self.A,Beta+120-180,clr="magenta")
        x0,y0 = self.drawBar(x0,y0,self.servo_R,Beta+120,clr="magenta")
        # 2nd servo bar
        self.drawBar(x0,y0,self.A,Alfa-180,clr="silver")





        ...

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


root = HelloWorld()
 
root.mainloop()