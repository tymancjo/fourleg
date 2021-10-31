# Helper file for IK work for Aron mk2
# Four Leg Robot
# Tymancjo 10.2021
# MIT license 

import math
import matplotlib.pyplot as plt

def my_print(input_msg):
    if debug:
        print(input_msg)
    else:
        ...

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

        Phi = math.acos((H**2+N**2-h**2)/(2*H*N))
        my_print(math.degrees(Phi))

        in_0 = (H**2+h**2-N**2)/(2*H*h)
        Psi = math.acos(in_0)
        my_print(f"Psi= {math.degrees(Psi)}")

        Alpha = math.radians(90) + dzetta + Psi
        my_print(math.degrees(Alpha))

        i2  = k**2 + F**2 -2*k*F*math.cos(Alpha + 0.5*math.pi - math.acos(E/k))
        i = math.sqrt(i2)
        my_print(f"{i=}")

        in_1 = (A**2+i2-G**2)/(2*A*i)
        my_print(f"{in_1=}")
        in_2 = (k**2+i2-F**2)/(2*k*i)
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
        k6 = math.acos((H**2+R2 - p2)/(2*H*R))
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

        kd   = math.acos((Q2+B**2-L**2)/(2*Q*B))
        my_print(f"k.={math.degrees(kd)}")
        kdd  = math.acos((Q2+R2-M**2)/(2*Q*R))
        my_print(f"k..={math.degrees(kdd)}")
        kddd = math.acos((Hy)/(R))
        my_print(f"k...={math.degrees(kddd)}")

        Beta = kd + kdd + kddd + math.pi/2





    else:
        return False
    
    if not be_precise:
        return int(math.degrees(Alpha0)),int(math.degrees(Beta)), int(math.degrees(Alpha)), int(math.degrees(Phi))
    return (math.degrees(Alpha0)),(math.degrees(Beta)), (math.degrees(Alpha)), (math.degrees(Phi))



# testing the 1st servo alpha0
debug = 0

# print(leg_IK_mk2((27.43,137.44)))
print(leg_IK_mk2((-33.3,124.6)))
print(leg_IK_mk2((10,130)))
print(leg_IK_mk2((0,130)))

x = [k for k in range(50,-50,-5)]
y = [120 + 10 * math.sin(math.radians(4*k)) for k in x]

print(x, y)
alfa = []
beta = []

for xx,yy in zip(x,y):
    a,b,c,d = leg_IK_mk2((xx,yy))
    alfa.append(a)
    beta.append(b-(90+30))

# fig1 = plt.figure()
plt.plot(x,alfa)
plt.plot(x,beta)
plt.plot(x,y)
plt.xlabel("x [mm]")
plt.ylabel("Angle [deg]")
plt.grid()
plt.show()
