import numpy as np
import matplotlib.pyplot as plt

K=1
tau=0.5
dt=0.01
t_max=1


Kp=4
Ki=5
Kd=0.0735

t=np.arange(0,t_max,dt)


integrale=0
erreur_precedente=0
sortie=[]
commande=100
perturbations=0
reponse=0


for e in t:
    erreur=commande-reponse
    integrale+=erreur*dt
    derivee=(erreur-erreur_precedente)/dt
    erreur_precedente=erreur
    
    consigne=Kp*erreur+Ki*integrale+Kd*derivee
    
    reponse+=(K*(consigne-perturbations)-reponse)*(dt/tau)
    sortie.append(reponse) 
    
plt.plot(t, sortie)
plt.axhline(commande, color="r")
plt.show()