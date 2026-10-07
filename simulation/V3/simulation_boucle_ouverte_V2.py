import numpy as np
import matplotlib.pyplot as plt
import math

# Paramètres système
K_g = 2.3       # gain statique
tau_g = 0.25     # constante de temps
K_d = 2.3      # gain statique
tau_d = 0.25     # constante de temps
dt = 0.01     # pas de temps
t_max = 50   # durée de la simulation

# Consigne
t_ramp = 1  # Durée du démarrage progressif en secondes
consigne_max = 12.56
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
Cr=[] 
x=0

def perturbations_fonction(x, t):
    """Perturbation dépendant de la position et du temps"""
    return (
        0.2 * np.random.normal() +
        (-2 if 1.25 + 0.5*np.pi < x % (4 + np.pi) < 1.75 + 0.5*np.pi else 0) +
        (2 if 2.25 + 0.5*np.pi < x % (4 + np.pi) < 2.75 + 0.5*np.pi else 0) +
        0.1 * np.sin(2 * np.pi * 0.3 * t)
    )

def f_reponse(reponse, K, commande, perturb, tau):
    """Équation différentielle du moteur : dr/dt = (K*u + perturb - r)/tau"""
    return (K * commande + perturb - reponse) / tau

def rk4_update(reponse, K, tau, commande_fn, perturb_fn, t, dt):
    """Intégration numérique Runge-Kutta ordre 4"""
    k1 = f_reponse(reponse, K, commande_fn(t), perturb_fn(t), tau)
    k2 = f_reponse(reponse + 0.5*dt*k1, K, commande_fn(t + 0.5*dt), perturb_fn(t + 0.5*dt), tau)
    k3 = f_reponse(reponse + 0.5*dt*k2, K, commande_fn(t + 0.5*dt), perturb_fn(t + 0.5*dt), tau)
    k4 = f_reponse(reponse + dt*k3, K, commande_fn(t + dt), perturb_fn(t + dt), tau)
    return reponse + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)

for e in t:
    if e < t_ramp:
        # Augmentation linéaire de la consigne (Rampe)
        consigne_actuelle = consigne_max * (e / t_ramp)
        consigne_g = consigne_actuelle
        consigne_d = consigne_actuelle
        
    # Stocke la consigne du moteur gauche pour la courbe et calcule la consigne du robot comme moyenne des deux moteurs
    Cg.append(consigne_g)
    Cr.append(0.5 * (consigne_g + consigne_d))
    
    # # Mise à jour de la réponse dynamique du moteur gauche (filtre du premier ordre)
    # reponse_g += (consigne_g + perturbations - reponse_g) * (dt / tau_g)
    # # Mise à jour de la réponse dynamique du moteur droit (filtre du premier ordre)
    # reponse_d += (consigne_d + perturbations - reponse_d) * (dt / tau_d)
    
    commande_g = consigne_g / K_g 
    commande_d = consigne_d / K_d 
   
    reponse_g = rk4_update(
        reponse_g, K_g, tau_g,
        lambda t_: commande_g,
        lambda t_: perturbations_fonction(x, t_),
        e, dt
    )
    
    # Moteur Droit
    reponse_d = rk4_update(
        reponse_d, K_d, tau_d,
        lambda t_: commande_d,
        lambda t_: perturbations_fonction(x, t_),
        e, dt
    )
    # Conversion des réponses dynamiques en vitesses linéaires à l'aide d'un coefficient de conversion
    vg = 0.03 * reponse_g 
    vd = 0.03 * reponse_d
    
    # Calcul de la vitesse du robot comme la moyenne des vitesses des deux moteurs
    vr = (vg + vd) * 0.5
    
    # Intégration de la vitesse pour obtenir la position x
    x += vr * dt
    
    # Sauvegarde des données pour affichage
    X.append(x)
    sortie_g.append(reponse_g)
    sortie_d.append(reponse_d)
    sortie_r.append(0.5 * (reponse_g + reponse_d))
    
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
                     (-2 if 1.25 + 0.5 * np.pi < x % (4 + np.pi) < 1.75 + 0.5 * np.pi else 0) + 
                     (2 if 2.25 + 0.5 * np.pi < x % (4 + np.pi) < 2.75 + 0.5 * np.pi else 0) + # événement ponctuel
                     0.1 * np.sin(2 * np.pi * 0.3 * e)  # composante sinusoïdale pour perturbation périodique
                    )
sortie_r = np.array(sortie_r) 
Cr = np.array(Cr)
t = np.array(t)

# Erreur totale robot
erreur_r = Cr - sortie_r        
    
ISE  = np.trapezoid(erreur_r**2, t)
IAE  = np.trapezoid(np.abs(erreur_r), t)
ITSE = np.trapezoid(t * erreur_r**2, t)
ITAE = np.trapezoid(t * np.abs(erreur_r), t)

print("\n=== Évaluation des performances en boucle ouverte avec intégration RK4 ===")
print(f"IAE   (∫|e|dt)     = {IAE:.4f}")
print(f"ISE   (∫e²dt)      = {ISE:.4f}")
print(f"ITAE  (∫t|e|dt)    = {ITAE:.4f}")
print(f"ITSE  (∫t·e²dt)    = {ITSE:.4f}")        
    
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
plt.plot(t, Cr, color='r', linestyle='--', label="Consigne")
plt.xlabel("Temps (s)")
plt.ylabel("Vitesse angulaire (rad/s)")
plt.grid(True)
plt.legend()

plt.subplots_adjust(top=0.9)
plt.suptitle("Figure 18 : Boucle ouverte - Réponse réelle des moteurs et du robot sans perturbations")
plt.show()