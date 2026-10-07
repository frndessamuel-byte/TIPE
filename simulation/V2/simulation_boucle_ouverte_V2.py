import numpy as np
import matplotlib.pyplot as plt
import math

# Paramètres système
K_g, K_d = 2.3, 2.3       # gain statique
tau_g, tau_d = 0.25, 0.25     # constante de temps
dt = 0.01     # pas de temps
t_max = 50   # durée de la simulation

# Consigne
consigne_g = 12.56 #val en rad/s
consigne_d = 12.56 #val en rad/s
perturbations=0
t = np.arange(0, t_max, dt)
reponse_g, reponse_d = 0, 0
sortie_g, sortie_d, sortie_r = [], [], []
Perturbations=[]
Cg, Cr = [], []
x=0

for e in t:
    # Stocke la consigne du moteur gauche pour la courbe et calcule la consigne du robot
    Cg.append(consigne_g)
    Cr.append(0.5 * (consigne_g + consigne_d))
    
    # Mise à jour de la réponse dynamique des moteurs
    reponse_g +=  (consigne_g + perturbations - reponse_g) * (dt / tau_g)
    reponse_d +=  (consigne_d + perturbations - reponse_d) * (dt / tau_d)

    # Conversion des réponses dynamiques en vitesses linéaires à l'aide du rayon des roues (0.03 m)
    vg = 0.03 * reponse_g 
    vd = 0.03 * reponse_d
    vr = (vg + vd) * 0.5
    
    # Intégration de la vitesse pour obtenir la position x
    x += vr * dt
    
    # Sauvegarde des données pour affichage
    sortie_g.append(reponse_g)
    sortie_d.append(reponse_d)
    sortie_r.append(vr)
    
    # Mise à jour de la consigne en fonction de la position x modifiée
    # (Utilisation du modulo pour définir différents intervalles de consigne)
    if x % (4 + math.pi) > 1:
        consigne_g = 8.11
    if x % (4 + math.pi) > (1 + 0.5 * math.pi):
        consigne_g = 12.56
    if x % (4 + math.pi) > (3 + 0.5 * math.pi):
        consigne_g = 8.11
    if x % (4 + math.pi) > (3 + math.pi):
        consigne_g = 12.56
    
    # Calcul des perturbations pour simuler un environnement irrégulier :
    perturbations = (0.2 * np.random.normal() +           # bruit aléatoire
                     (-2 if 1.25 + 0.5 * np.pi < x % (4 + np.pi) < 1.75 + 0.5 * np.pi else 0)  
                     + (2 if 2.25 + 0.5 * np.pi < x % (4 + np.pi) < 2.75 + 0.5 * np.pi else 0)  
                     + 0.1 * np.sin(2 * np.pi * 0.3 * e)   # irrégularité de l'environnement 
                    )
    Perturbations.append(perturbations)

Cr = np.array(Cr)

# Affichage
plt.figure(figsize=(20, 3))
plt.subplot(2,2,1)
plt.plot(t, sortie_g, label="Réponse du moteur gauche")
plt.plot(t, Cg, color='r', linestyle='--', label="Consigne", )
plt.xlabel("Temps (s)")
plt.ylabel("Vitesse angulaire (rad/s)")
plt.grid(True)
plt.legend()
plt.subplot(2,2,2)
plt.plot(t, sortie_d, label="Réponse du moteur droit")
plt.axhline(y=12.56, color='r', linestyle='--', label="Consigne")
plt.xlabel("Temps (s)")
plt.ylabel("Vitesse angulaire (rad/s)")
plt.grid(True)
plt.legend()
plt.subplot(2,1,2)
plt.plot(t, sortie_r, label="Réponse du robot")
plt.plot(t, Cr*0.03, color='r', linestyle='--', label="Consigne")
plt.xlabel("Temps (s)")
plt.ylabel("Vitesse (m/s)")
plt.grid(True)
plt.legend()

plt.subplots_adjust(top=0.9)
plt.suptitle("Figure 9 : Boucle ouverte - Réponse des moteurs et du robot sans régulation")
plt.show()