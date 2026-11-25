import numpy as np 
import math as mt 
import casadi as ca 


Kv= 980 #KV [rpm/V], sin fuente: Parrot no lo publica; con 3S (11.1 V) y 12000 rpm en vuelo debe ser > 1080, medirlo
Kt=60/(2*np.pi*Kv) #constante de torque [N·m/A]; Kv va en rpm/V, por eso no es 1/Kv
tf=0.0 #torque de friccion [N·m], medir (corriente sin hélice)
R=0.0 #resistencia del embobinado [ohm], medir entre fases / 2
Ke=Kt #constante de voltaje [V·s/rad], igual a Kt en SI
Df = 0.0 #friccion viscosa [N·m·s/rad], medir (corriente sin hélice)
kappa_tau = 1.9e-8 #arrastre aerodinamico [N·m·s²], sigma*kappa = 0.01 m * 1.9e-6 (Sun et al. 2020, Bebop 2)
J=8.0e-6 #inercia del rotor [kg·m²] (Sun et al. 2020, Bebop 2)

m=0.503 #masa del dron [kg] (modelo Gazebo; Sun et al. usan 0.41 sin cámara y con batería ligera)
g=9.81 #aceleracion de la gravedad
L=0.145 #distancia del centro de masa al motor [m] (Sun et al. 2020; el SDF da 0.146)


 

c1 = (R * tf**2) / (Kt**2)  # costo estatico de friccion 
c2 = (tf / Kt) * (((2 * R * Df) / Kt) + Ke)  # costo del fluido interno 
c3 = (Df / Kt) * (((R * Df) / Kt) + Ke) + ((2 * R * tf * kappa_tau) / (Kt**2))  # eficiencia electromagnetica
c4 = (kappa_tau / Kt) * (((2 * R * Df) / Kt) + Ke)  # costo del arrastre aerodinamico 
c5 = (R * kappa_tau**2) / (Kt**2)  # costo aerodinamico cuadratico
c6 = (2 * R * J * tf) / (Kt**2)  # costo de la inercia del rotor (opcional calcularlo)
c7 = (R * J**2) / (Kt**2)  # costo de la agresividad termica

opti = ca.Opti()  # Create an optimization problem
N=50  # Number of time steps
dt=0.1

#Matriz de estados 
A=opti.variable(16,N+1) # [x,y,z,phi,theta,psi,vx,vy,vz,p,q,r,w1,w2,w3,w4]
D=opti.variable(4,N) # [w1,w2,w3,w4]

pos_x = A[0,:]
pos_y = A[1,:]
pos_z = A[2,:]
roll = A[3,:]
pitch = A[4,:]
yaw = A[5,:]
vx=A[6,:]
vy=A[7,:]
vz=A[8,:]
p = A[9,:]
q = A[10,:]
r = A[11,:]
w1 = A[12,:]
w2 = A[13,:]
w3 = A[14,:]
w4 = A[15,:]

#alpha aceleraciones angulares 
ap1 = D[0,:]
ap2 = D[1,:]
ap3 = D[2,:]
ap4 = D[3,:]

costo_inicial=0.0
costo_m1=0.0
costo_m2=0.0
costo_m3=0.0
costo_m4=0.0

for k in range(N):
    costo_m1 = c1+(c2*w1[k])+(c3*w1[k]**2)+(c4*w1[k]**3)+(c5*w1[k]**4)+(c7*ap1[k]**2)
    costo_m2 = c1+(c2*w2[k])+(c3*w2[k]**2)+(c4*w2[k]**3)+(c5*w2[k]**4)+(c7*ap2[k]**2)
    costo_m3 = c1+(c2*w3[k])+(c3*w3[k]**2)+(c4*w3[k]**3)+(c5*w3[k]**4)+(c7*ap3[k]**2)
    costo_m4 = c1+(c2*w4[k])+(c3*w4[k]**2)+(c4*w4[k]**3)+(c5*w4[k]**4)+(c7*ap4[k]**2)
    costo_total = (costo_m1 + costo_m2 + costo_m3 + costo_m4)*dt
    costo_inicial += costo_total

for k in range(N):
    pos_x[k+1] = pos_x[k] + vx[k]*dt
    pos_y[k+1] = pos_y[k] + vy[k]*dt
    pos_z[k+1] = pos_z[k] + vz[k]*dt
    roll[k+1] = roll[k] + p[k]*dt
    pitch[k+1] = pitch[k] + q[k]*dt
    yaw[k+1] = yaw[k] + r[k]*dt
    



opti.minimize(costo_inicial)  # Minimize the total cost
opti.subject_to




