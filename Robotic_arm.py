# 6DOF Robotic Arm
import math
import numpy as np

#Initial Parameters
a1z = 650
a2x = 400
a2z = 680
a3z = 1100
a4z = 230
a4x = 766
a5x = 345
a6x = 244 #The offset of WP and TCP

b4x = math.sqrt( math.pow(a4z,2) + math.pow((a4x + a5x),2) )

TCPx = 1500
TCPy = 1000
TCPz = 2000
TCPa = 0
TCPb = 0
TCPc = 0

def Joint1(WPx, WPy):
    J1 = math.atan2(WPy, WPx)
    J1 = math.degrees(J1)
    print("J1=" + str(round(J1,2)))
    return J1
  
def Joint2(alpha, B):
    J2 = round((math.pi/2),2) - alpha - B #B is positive if in down state
    J2 = math.degrees(J2)
    print("J2=" + str(round(J2,2)))
    return J2

def Joint3(delta, y):
    J3 = math.pi - y - delta
    J3 = math.degrees(J3)
    print("J3=" + str(round(J3,2)))
    return J3

def Joint4(R21, R31):
    J4 = math.atan2(R21, R31)
    J4 = math.degrees(J4)
    print("J4=" + str(round(J4,2)))
    return J4

def Joint5(R11):
    J5 = math.atan2(math.sqrt(1 - math.pow(R11, 2)), R11)
    J5 = math.degrees(J5)
    print("J5=" + str(round(J5,2)))
    return J5

def Joint6(R12, R13):
    J6 = math.atan2(R12, R13)
    J6 = math.degrees(J6)
    print("J6=" + str(round(J6,2)))
    return J6

A = TCPa;
B = TCPb;
C = TCPc;

R = np.array([[math.cos(C)*math.cos(B), math.cos(C)*math.sin(B)*math.sin(A)-math.sin(C)*math.cos(A), math.sin(C)*math.sin(A)+math.cos(C)*math.sin(B)*math.cos(A)],
              [math.sin(C)*math.cos(B), math.cos(C)*math.cos(A)+math.sin(C)*math.sin(B)*math.sin(A), math.sin(C)*math.sin(B)*math.cos(A)-math.cos(C)*math.sin(A)],
              [-math.sin(B), math.cos(B)*math.sin(A), math.cos(B)*math.cos(A)]])

print(R)

R11= R[0][0]
R21= R[1][0]
R31= R[2][0]

R12= R[0][1]
R13= R[0][2]

WPx = TCPx - a6x * R11
print("WPx " + str(WPx))

WPy = TCPy - a6x * R21
print("WPy " + str(WPy))

WPz = TCPz - a6x * R31
print("WPz " + str(WPz))

WPxy = math.sqrt(math.pow(WPx,2) + math.pow(WPy,2))

l = WPxy - a2x #could be positive or negative. (see original calcs)
h = WPz - a1z - a2z 
alpha = math.atan2(h,l)
p = round(math.sqrt(math.pow(h,2) + math.pow(l,2)),0)

print("Rho " + str(p))

# p < a3z + b4x
pCond1 = a3z + b4x
print("Condition 1 = Rho < a3z + b4x " + str(pCond1))

# p > |a3z - b4x|
pCond2 = abs(a3z - b4x)
print("Condition 2 = Rho > |a3z - b4x|" + str(pCond2))

B = np.arccos(( math.pow(p,2) + math.pow(a3z,2) - math.pow(b4x,2)) /( 2*p*a3z ))
y = np.arccos((math.pow(a3z,2) + math.pow(b4x,2) - math.pow(p,2)) / ( 2*a3z*b4x))
delta = math.atan2(a4x + a5x, a4z)

J1 = math.radians(Joint1(WPx, WPy))
J2 = math.radians(Joint2(alpha, B))
J3 = math.radians(Joint3(delta, y))

R_arm = np.array([[math.cos(J1)*math.cos(J2+J3), -math.sin(J1), math.cos(J1)*math.sin(J2+J3)],
                  [math.sin(J1)*math.cos(J2+J3), math.cos(J1), math.sin(J1)*math.sin(J2+J3)],
                  [-math.sin(J2+J3), 0, math.cos(J2+J3)]])

print(R_arm)

RTarm = R_arm.transpose()
print(RTarm)

Rwrist = R.dot(RTarm)
print(Rwrist)

J4 = math.atan2(Rwrist[0][1], Rwrist[0][2])
J4 = math.degrees(J4)
print(J4)

J5 = math.atan2(math.sqrt(1-pow(Rwrist[0][0],2)), Rwrist[0][0])
J5 = math.degrees(J5)
print(J5)

J6 = math.atan2(Rwrist[1][0],Rwrist[2][0])
J6 = math.degrees(J6)
print(J6)

#Expected Output
#J4 = -124.7
#J5 = 49.2
#J6 = 136.6