import numpy as np
import matplotlib.pyplot as plt
import math

# Paramètres système
K_g = 2.3       # gain statique du moteur gauche
tau_g = 0.25     # constante de temps du moteur gauche
K_d = 2.3      # gain statique du moteur droit
tau_d = 0.25     # constante de temps du moteur droit
dt = 0.01     # pas de temps
t_max = 50   # durée de la simulation

# PID (proportionnel seulement)
Kp_g, Kp_d = 10, 10

# Temps et initialisation
consigne_g = 12.56 #val en rad/s
consigne_d = 12.56 #val en rad/s
perturbations=0
t = np.arange(0, t_max, dt)
reponse_g = 0
reponse_d = 0
sortie_g = []
sortie_d = []
sortie_r = []
X=[]
Cg=[]
Cr=[] #np.vect
x=0

def calcul_réponse(reponse, K, consigne, Kp, perturbations, tau):
        erreur= consigne - (reponse + np.random.normal(0, 0.1)) # ajout de bruit de mesure
        commande = Kp * erreur
        commande = np.clip(commande, -20, 20)
        return reponse + (K * commande + perturbations - reponse) * (dt / tau)

for e in t:
    # Stocker la consigne du moteur gauche et la consigne moyenne pour le robot
    Cg.append(consigne_g)
    Cr.append(0.5 * (consigne_g + consigne_d))
    
    # Mise à jour de la réponse des moteurs selon un modèle dynamique de premier ordre
    reponse_g = calcul_réponse(reponse_g, K_g, consigne_g, Kp_g, perturbations, tau_g)
    reponse_d = calcul_réponse(reponse_d, K_d, consigne_d, Kp_d, perturbations, tau_d)
    
    # Calcul des vitesses issues des réponses des moteurs
    vg = 0.03 * reponse_g  # Vitesse du moteur gauche
    vd = 0.03 * reponse_d  # Vitesse du moteur droit
    vr = 0.5 * (vg + vd)   # Vitesse du robot (moyenne des deux moteurs)
    
    # Mise à jour de la position du robot par intégration
    x += vr * dt
    
    # Stockage des valeurs pour tracé ultérieur
    X.append(x)
    sortie_g.append(reponse_g)
    sortie_d.append(reponse_d)
    sortie_r.append(0.5 * (reponse_g + reponse_d))
    
    # Modification de la consigne du moteur gauche en fonction de la position du robot
    if x % (4 + math.pi) > 1:
        consigne_g = 8.11
    if x % (4 + math.pi) > (1 + 0.5 * math.pi):
        consigne_g = 12.56
    if x % (4 + math.pi) > (3 + 0.5 * math.pi):
        consigne_g = 8.11
    if x % (4 + math.pi) > (3 + math.pi):
        consigne_g = 12.56
    
    # Calcul des perturbations incluant :
    perturbations = (
        0.2 * np.random.normal() +
        (-2 if 1.25 + 0.5 * np.pi < x % (4 + np.pi) < 1.75 + 0.5 * np.pi else 0) +
        (2 if 2.25 + 0.5 * np.pi < x % (4 + np.pi) < 2.75 + 0.5 * np.pi else 0) +
        0.1 * np.sin(2 * np.pi * 0.3 * e)
    )

# Affichage
plt.figure(figsize=(20, 3))
plt.subplot(2, 2,2)
plt.plot(t, sortie_g, label="Réponse du moteur gauche")
plt.plot(t, Cg, color='r', linestyle='--', label="Consigne")
plt.xlabel("Temps (s)")
plt.ylabel("Vitesse angulaire (rad/s)")
plt.grid(True)
plt.legend()
plt.subplot(2, 2,1)
plt.plot(t, sortie_d, label="Réponse du moteur droit")
plt.axhline(y=12.56, color='r', linestyle='--', label="Consigne")
plt.xlabel("Temps (s)")
plt.ylabel("Vitesse angulaire (rad/s)")
plt.grid(True)
plt.legend()
plt.subplot(2, 1,2)
plt.plot(t, sortie_r, label="Réponse du robot")
plt.plot(t, Cr, color='r', linestyle='--', label="Consigne")
plt.xlabel("Temps (s)")
plt.ylabel("Vitesse angulaire (rad/s)")
plt.grid(True)
plt.legend()

plt.subplots_adjust(top=0.9)
plt.suptitle("Figure 10 : Boucle fermée - Réponse des moteurs et du robot avec régulation Proportionnelle")
plt.show()