import numpy as np
import matplotlib.pyplot as plt

K=1
tau=0.5
dt=0.01
t_max=1


Kp=25
Ki=50
Kd=0.075

t=np.arange(0,t_max,dt)


integrale=0
erreur_precedente=0
sortie=[]
commande=100
reponse=0


for e in t:
    erreur=commande-reponse
    integrale+=erreur*dt
    derivee=(erreur-erreur_precedente)/dt
    erreur_precedente=erreur
    
    consigne=Kp*erreur+Ki*integrale+Kd*derivee
    
    reponse+=(K*consigne-reponse)*(dt/tau)
    sortie.append(reponse)
    
plt.plot(t, sortie)
plt.axhline(commande, color="r")
plt.show()