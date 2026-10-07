import numpy as np
import matplotlib.pyplot as plt

K_g, K_d = 2.3, 2.3        # gains statiques des moteurs gauche et droit
tau_g, tau_d = 0.15, 0.25  # constantes de temps des moteurs gauche et droit      
dt = 0.01                  # pas de temps
t_max = 50                 # durée de la simulation

Kp_g,  Ki_g, Kd_g, Kp_d,  Ki_d, Kd_d = 9, 2, 0.04, 9, 2, 0.04  # coefficients du PID
#9,2,0.04     k/tau base
#4.968327, 9.966255, 0.005116    k/tau robot
# 2, 6, 0.005116, 2, 6, 0.005116 
consigne_g = consigne_d = 12.56637061         # initialisation des consignes de vitesse angulaire en rad/s
perturbations = 0                             # initialisation des perturbations

t = np.arange(0, t_max, dt) 
reponse_g = reponse_d = 0 # initialisation des réponses des moteurs gauche et droit

# initialisation des listes pour stocker les réponses et les consignes
sortie_g = [] 
sortie_d = []
sortie_r = []

X = [] 
Cg = []
Cr = [] # consignes du robot
x = 0 # position du robot

integrale_g = integrale_d = 0 # initialisation des intégrales pour le PID
erreur_precedente_g = erreur_precedente_d = 0

def calcul_réponse(reponse, K, consigne, integrale, erreur_precedente, Kp, Ki, Kd, perturbations, tau):
        erreur= consigne - (reponse + np.random.normal(0, 0.1)) # ajout de bruit de mesure
        integrale += erreur * dt
        integrale = np.clip(integrale, -10, 10)
        derivee = (erreur - erreur_precedente) / dt
        erreur_precedente= erreur
        commande = Kp * erreur + Ki * integrale + Kd * derivee
        commande = np.clip(commande, -20, 20)
        return reponse + (K * commande + perturbations - reponse) * (dt / tau), integrale, erreur_precedente

for e in t: 
    # Stockage des consignes pour le moteur gauche et la commande moyenne pour le robot
    Cg.append(consigne_g) 
    Cr.append(0.03*0.5 * (consigne_g + consigne_d)) 
    
    reponse_g, integrale_g, erreur_precedente_g = calcul_réponse(reponse_g, K_g, consigne_g, integrale_g, erreur_precedente_g, Kp_g, Ki_g, Kd_g, perturbations, tau_g)
    reponse_d, integrale_d, erreur_precedente_d = calcul_réponse(reponse_d, K_d, consigne_d, integrale_d, erreur_precedente_d, Kp_d, Ki_d, Kd_d, perturbations, tau_d)
    
    # Calcul de la vitesse en m/s pour chaque moteur et pour le robot
    vg = 0.03 * reponse_g  # vitesse du moteur gauche
    vd = 0.03 * reponse_d  # vitesse du moteur droit
    vr = (vg + vd) * 0.5   # vitesse moyenne du robot
    x += vr * dt           # mise à jour de la position du robot (intégration de la vitesse)
    
    # Stockage de la position et des réponses pour analyse ultérieure
    X.append(x) 
    sortie_g.append(reponse_g) 
    sortie_d.append(reponse_d)
    sortie_r.append(vr)
    
    # Ajustement de la consigne en fonction de la position du robot selon un cycle périodique
    if x % (4 + np.pi) > 1: 
        consigne_g = 8.11  # consigne inférieure en rad/s
    if x % (4 + np.pi) > (1 + 0.5 * np.pi):
        consigne_g = 12.56 # consigne supérieure en rad/s
    if x % (4 + np.pi) > (3 + 0.5 * np.pi):
        consigne_g = 8.11  # repasse à la consigne inférieure
    if x % (4 + np.pi) > (3 + np.pi):
        consigne_g = 12.56 # consigne supérieure en rad/s

    # Ajout de perturbations aléatoires et événements ponctuels
    perturbations = (0.2 * np.random.normal() +                           # bruit aléatoire
                    (-2 if 1.25 + 0.5 * np.pi < x % (4 + np.pi) < 1.75 + 0.5 * np.pi  else 0) + 
                    (2 if 2.25 + 0.5 * np.pi < x % (4 + np.pi) < 2.75 + 0.5 * np.pi  else 0) +  # événement ponctuel
                    0.1 * np.sin(2 * np.pi * 0.3 * e)  # irrégularité de l'environnement
                    )
# Conversion des listes en tableaux NumPy pour les calculs ultérieurs des critères de performance
sortie_r = np.array(sortie_r) 
Cr = np.array(Cr)
t = np.array(t)

# Erreur totale robot
erreur_r = Cr - sortie_r 

# Critères d’évaluation :
ISE = np.trapezoid(erreur_r**2, t)           # Integral of Squared Error
IAE = np.trapezoid(np.abs(erreur_r), t)      # Integral of Absolute Error
ITSE = np.trapezoid(t * erreur_r**2, t)      # Integral of Time * Squared Error
ITAE = np.trapezoid(t * np.abs(erreur_r), t) # Integral of Time * Absolute Error

print("\n=== Évaluation des performances PID avec Kalman ===") 
print(f"IAE   (∫|e|dt)     = {IAE:.4f}") 
print(f"ISE   (∫e²dt)      = {ISE:.4f}")
print(f"ITAE  (∫t|e|dt)    = {ITAE:.4f}")
print(f"ITSE  (∫t·e²dt)    = {ITSE:.4f}")

# Affichage des résultats
plt.figure(figsize=(20, 3))
# plt.subplot(2, 2, 1)
# plt.plot(t, sortie_g, label="Réponse")
# plt.plot(t, Cg, color='r', linestyle='--', label="Consigne" )
# plt.xlabel("Temps (s)")
# plt.ylabel("Vitesse angulaire (rad/s)")
# plt.title("Moteur gauche")
# plt.grid(True)
# plt.legend()

# plt.subplot(2, 2, 2)
# plt.title("Moteur droit", loc='center')
# plt.plot(t, sortie_d, label="Réponse")
# plt.axhline(y=12.56, color='r', linestyle='--', label="Consigne")
# plt.xlabel("Temps (s)")
# plt.ylabel("Vitesse angulaire (rad/s)")
# plt.grid(True)
# plt.legend()

plt.subplot(2, 1, 2)
plt.title("Robot", loc='center')
plt.plot(t, sortie_r, label="Réponse")
plt.plot(t, Cr, color='r', linestyle='--', label="Consigne")
plt.xlabel("Temps (s)")
plt.ylabel("Vitesse (m/s)")
plt.grid(True)
plt.legend()

plt.subplots_adjust(top=0.9)
plt.suptitle("Figure 15 : Boucle fermée - Réponse du robot avec régulation PID (4.902, 19.872, 0.019)", fontsize=16)
plt.show()
