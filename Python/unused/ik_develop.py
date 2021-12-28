# Helper file for IK work for Aron mk2
# Four Leg Robot
# Tymancjo 10.2021
# MIT license 

import math
import matplotlib.pyplot as plt
import numpy as np

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
        my_print(f"{in_1=}")
        in_1 = limit(in_1)

        in_2 = (k**2+i2-F**2)/(2*k*i)
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

    kddd = math.acos((G**2+F**2 - One2)/(2*G*F))
    k4 = math.acos((One2+F**2-G**2)/(2*One*F))
    k5 = math.acos((One2+k2-A**2)/(2*One*k))

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


# testing the 1st servo alpha0
debug = 0

# print(leg_IK_mk2((27.43,137.44)))
# print(leg_IK_mk2((-87,95)))
a,b,c,d = leg_IK_mk2((-12,117))
print(leg_DK_mk2((a,b)))
print(a,b,c,d)

a,b,c,d = leg_IK_mk2((-87,95))
print(leg_DK_mk2((a,b)))
print(a,b,c,d)

a,b,c,d = leg_IK_mk2((2,122))
print(leg_DK_mk2((a,b)))
print(a,b,c,d)


x = [100*math.sin(math.radians(k)) for k in range(100,-100,-1)]
y = [120 + 30*math.sin(3*math.radians(k)) for k in x]

a1 = []
a2 = []
b1 = []
b2 = []
_xx = []
_yy = []

for xx,yy in zip(x,y):
   a,b,c,d = leg_IK_mk2((xx,yy),be_precise=1) 
   a1.append(c)
   b1.append(d)
   c,d,_x,_y = leg_DK_mk2((a,b))
   a2.append(c)
   b2.append(d)
   _xx.append(_x)
   _yy.append(_y)

# plt.plot(x,a1,label="ik A")
# plt.plot(x,a2,label="dk A")
# plt.plot(x,b1,label="ik B")
# plt.plot(x,b2,label="dk A")
plt.plot(x,y,label="in Y")
plt.plot(x,x,label="in X")
plt.plot(x,_xx,label="X")
plt.plot(x,_yy,label="Y")
plt.grid()
plt.legend()
plt.show()

x = [k for k in range(100,-100,-1)]
# y = [k for k in range(50,150,1)]

# # creating the pre-calculated solution matrix
# the_array_of_Alpha = []
# the_array_of_Beta = []
# for yy in y:
#     the_row_A = []
#     the_row_B = []
#     for xx in x:
#         a,b,_,_ = leg_IK_mk2((xx,yy))
#         the_row_A.append(a) 
#         the_row_B.append(b)
#     the_array_of_Alpha.append(the_row_A)
#     the_array_of_Beta.append(the_row_B)

# Alpha = np.array(the_array_of_Alpha)
# Beta = np.array(the_array_of_Beta)

# print(f"Size: {Alpha.shape, Beta.shape}")
# X, Y = np.meshgrid(x, y) 

# # Create the figure
# fig = plt.figure()

# # Add an axes
# ax1 = fig.add_subplot(121,projection='3d')
# ax2 = fig.add_subplot(122,projection='3d')

# fig.suptitle("Aron Mk2 IK model solutions space", fontsize=16)

# # plot the surface
# ax1.plot_surface(X, Y, Alpha, alpha=1)
# ax2.plot_surface(X, Y, Beta, alpha=1)

# ax1.set_xlabel("x [mm]")
# ax1.set_ylabel("y [mm]")
# ax1.set_zlabel("Servo1 [deg]")
# ax1.set_title("Servo 1")

# ax2.set_xlabel("x [mm]")
# ax2.set_ylabel("y [mm]")
# ax2.set_zlabel("Servo2 [deg]")
# ax2.set_title("Servo 2")


# plt.show()

# fig1 = plt.figure()
# plt.plot(x,alfa)
# plt.plot(x,beta)
# plt.plot(x,y)
# plt.xlabel("x [mm]")
# plt.ylabel("Angle [deg]")
# plt.grid()
# plt.show()
