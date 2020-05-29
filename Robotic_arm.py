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
    print("J1=" + str(round(J1,2)))
  
def Joint2(alpha, B):
    J2 = round((math.pi/2),2) - alpha - B #B is positive if in down state
    print("J2=" + str(round(J2,2)))

def Joint3(delta, y):
    J3 = math.pi - y - delta
    print("J3=" + str(round(J3,2)))

def Joint4(R21, R31):
    J4 = math.atan2(R21, R31)
    print("J4=" + str(round(J4,2)))

def Joint5(R11):
    J5 = math.atan2(math.sqrt(1 - math.pow(R11, 2)), R11)
    print("J5=" + str(round(J5,2)))

def Joint6(R12, R13):
    J6 = math.atan2(R12, R13)
    print("J6=" + str(round(J6,2)))

#Rx = np.array([[1, 0, 0], [0, math.cos(TCPa), -math.sin(TCPa)], [0, math.sin(TCPa), math.cos(TCPa)]])
#Ry = np.array([[math.cos(TCPb), 0, math.sin(TCPb)], [0, 1, 0], [-math.sin(TCPb), 0, math.cos(TCPb)]])
#Rz = np.array([[math.cos(TCPc), -math.sin(TCPc), 0], [math.sin(TCPc), math.cos(TCPc), 0], [0, 0, 1]])

#R = Rx(C)Ry(B)Rz(A)
#RXY = Rx.dot(Ry)
#RXYZ = RXY.dot(Rz)

# x=A; y =B; z=C)
A = TCPa;
B = TCPb;
C = TCPc;

R = np.array([[math.cos(C)*math.cos(B), math.cos(C)*math.sin(B)*math.sin(A)-math.sin(C)*math.cos(A), math.sin(C)*math.sin(A)+math.cos(C)*math.cos(B)*math.cos(A)],
              [math.sin(C)*math.cos(B), math.cos(C)*math.cos(A)+math.sin(C)*math.sin(B)*math.sin(A), math.sin(C)*math.sin(B)*math.cos(A)-math.cos(C)*math.sin(A)],
              [-math.sin(B), math.cos(B)*math.sin(A), math.cos(B)*math.cos(A)]])

print(R)

#print(RXYZ)

R11= R[0][0]
R21= R[1][0]
R31= R[2][0]

print(R11)
print(R21)
print(R31)

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

Bcalc = ( math.pow(p,2) + math.pow(a3z,2) - math.pow(b4x,2) )/( 2*p*a3z )
B = np.arccos(Bcalc)

yCalc = (math.pow(a3z,2) + math.pow(b4x,2) - math.pow(p,2)) / (2*(a3z * b4x))
y = np.arccos(yCalc)
delta = math.atan2(a4x + a5x, a4z)

Joint1(WPx, WPy)
Joint2(alpha, B)
Joint3(delta, y)
#Joint4(R21, R31)
#Joint5(R11)
#Joint6(R12, R13)