import numpy as np
import matplotlib.pyplot as plt

# Paramètres du simulateur et du PID
K_g, K_d = 2.3, 2.3
tau_g, tau_d = 0.15, 0.25
dt = 0.01
t_max = 50
# Aucun code relatif à l'animation du robot n'est ajouté.
Kp_g, Ki_g, Kd_g = 2, 6, 0.005116
Kp_d, Ki_d, Kd_d = 2, 6, 0.005116

consigne_g = consigne_d = 12.56637061
perturbations = 0

t = np.arange(0, t_max, dt)
reponse_g = reponse_d = 0

# Initialisation des listes pour stocker les valeurs
sortie_g, sortie_d, sortie_r = [], [], []
mesures_g, mesures_d = [], []
kalman_g, kalman_d = [], []
X = []
Cg, Cr = [], []
x = 0

integrale_g = integrale_d = 0
erreur_precedente_g = erreur_precedente_d = 0

# Paramètres du filtre de Kalman pour chaque moteur
Q = 0.001  # bruit du modèle
R = 0.01  # bruit de mesure

# Initialisation des estimations et covariances au démarrage
x_hat_g, P_g = 0, 1.0
x_hat_d, P_d = 0, 1.0

def kalman_prediction(x_hat, P, commande, K, tau, dt, Q):
    """Extrapolation de l'état et de la covariance selon le modèle moteur"""
    # Prédiction de la vitesse attendue (modèle du moteur)
    x_hat = x_hat + (K * commande + perturbations- x_hat) * (dt / tau)
    # Prédiction de la covariance
    P = P + Q
    return x_hat, P

def kalman_mise_a_jour(x_hat, P, mesure, R):
    """
    Correction de l'estimation avec la mesure bruitée.
    """
    K_gain = P / (P + R)
    x_hat = x_hat + K_gain * (mesure - x_hat)
    P = (1 - K_gain) * P
    return x_hat, P

for e in t:
    Cg.append(consigne_g)
    Cr.append(0.5 * (consigne_g + consigne_d))
    
    # Simulation d'une mesure bruitée pour chaque moteur
    mesure_g = reponse_g + np.random.normal(0, np.sqrt(R))
    mesure_d = reponse_d + np.random.normal(0, np.sqrt(R))
    mesures_g.append(mesure_g)
    mesures_d.append(mesure_d)

    
    # Utilisation des estimations filtrées pour le calcul de l'erreur du PID
    erreur_g = consigne_g - x_hat_g
    erreur_d = consigne_d - x_hat_d
    integrale_g += erreur_g * dt 
    integrale_d += erreur_d * dt 
    integrale_g = np.clip(integrale_g, -10, 10)
    integrale_d = np.clip(integrale_d, -10, 10)

    derivee_g = (erreur_g - erreur_precedente_g) / dt 
    derivee_d = (erreur_d - erreur_precedente_d) / dt
    erreur_precedente_g = erreur_g 
    erreur_precedente_d = erreur_d 
    
    commande_g = Kp_g * erreur_g + Ki_g * integrale_g + Kd_g * derivee_g 
    commande_d = Kp_d * erreur_d + Ki_d * integrale_d + Kd_d * derivee_d 
    commande_g = np.clip(commande_g, -20, 20) 
    commande_d = np.clip(commande_d, -20, 20)

    # --- Filtre de Kalman avec extrapolation ---
    # Prédiction (extrapolation)
    x_hat_g, P_g = kalman_prediction(x_hat_g, P_g, commande_g, K_g, tau_g, dt, Q)
    x_hat_d, P_d = kalman_prediction(x_hat_d, P_d, commande_d, K_d, tau_d, dt, Q)
    # Correction (mise à jour avec la mesure)
    x_hat_g, P_g = kalman_mise_a_jour(x_hat_g, P_g, mesure_g, R)
    x_hat_d, P_d = kalman_mise_a_jour(x_hat_d, P_d, mesure_d, R)
    kalman_g.append(x_hat_g)
    kalman_d.append(x_hat_d)
    
    # Simulation de la réponse du moteur
    reponse_g += (K_g * commande_g + perturbations - reponse_g) * (dt / tau_g) 
    reponse_d += (K_d * commande_d + perturbations - reponse_d) * (dt / tau_d)
    
    # Calcul des vitesses et position
    vg = 0.03 * reponse_g 
    vd = 0.03 * reponse_d 
    vr = 0.5 * (vg + vd)
    x += vr * dt
    
    X.append(x)
    sortie_g.append(reponse_g)
    sortie_d.append(reponse_d)
    sortie_r.append(0.5 * (reponse_g + reponse_d))
    
    # Mise à jour de la consigne en fonction de la position du robot
    if x % (4 + np.pi) > 1:
        consigne_g = 8.11
    if x % (4 + np.pi) > (1 + 0.5 * np.pi):
        consigne_g = 12.56
    if x % (4 + np.pi) > (3 + 0.5 * np.pi):
        consigne_g = 8.11
    if x % (4 + np.pi) > (3 + np.pi):
        consigne_g = 12.56

    # Ajout de perturbations aléatoires et événements ponctuels    
    perturbations = (0.2 * np.random.normal() +
                     (-2 if 1.25 + 0.5 * np.pi < x % (4 + np.pi) < 1.75 + 0.5 * np.pi else 0) + 
                     (2 if 2.25 + 0.5 * np.pi < x % (4 + np.pi) < 2.75 + 0.5 * np.pi else 0) +
                     0.1 * np.sin(2 * np.pi * 0.3 * e)
                    )

# Calcul des critères de performance
sortie_r = np.array(sortie_r)
Cr = np.array(Cr)
t = np.array(t)
erreur_r = Cr - sortie_r 

ISE = np.trapz(erreur_r**2, t)
IAE = np.trapz(np.abs(erreur_r), t)
ITSE = np.trapz(t * erreur_r**2, t)
ITAE = np.trapz(t * np.abs(erreur_r), t)

print("\n=== Évaluation des performances PID avec Kalman ===")
print(f"IAE   (∫|e|dt)     = {IAE:.4f}")
print(f"ISE   (∫e²dt)      = {ISE:.4f}")
print(f"ITAE  (∫t|e|dt)    = {ITAE:.4f}")
print(f"ITSE  (∫t·e²dt)    = {ITSE:.4f}")

# Affichage des résultats
plt.figure(figsize=(18, 8))

plt.subplot(2, 2, 1)
plt.plot(t, sortie_g, label="Réponse réelle moteur gauche")
plt.plot(t, mesures_g, label="Mesure bruitée gauche", alpha=0.5)
plt.plot(t, kalman_g, label="Kalman gauche", linestyle='--')
plt.plot(t, Cg, color='r', linestyle='--', label="Consigne")
plt.axhline(y=12.56, color='r', linestyle='--', label="Consigne")
plt.xlabel("Temps (s)")
plt.ylabel("Vitesse angulaire (rad/s)")
plt.title("Moteur gauche")
plt.grid(True)
plt.legend()

plt.subplot(2, 2, 2)
plt.plot(t, sortie_d, label="Réponse réelle moteur droit")
plt.plot(t, mesures_d, label="Mesure bruitée droit", alpha=0.5)
plt.plot(t, kalman_d, label="Kalman droit", linestyle='--')
plt.axhline(y=12.56, color='r', linestyle='--', label="Consigne")
plt.xlabel("Temps (s)")
plt.ylabel("Vitesse angulaire (rad/s)")
plt.title("Moteur droit")
plt.grid(True)
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(t, sortie_r, label="Réponse du robot")
plt.plot(t, Cr, color='r', linestyle='--', label="Consigne")
plt.xlabel("Temps (s)")
plt.ylabel("Vitesse (m/s)")
plt.title("Réponse du robot")
plt.grid(True)
plt.legend()

plt.suptitle("Figure 15 : Boucle fermée - Réponse des moteurs et du robot avec régulation PID avec filtre Kalman 1d (2, 6, 0.005116)", fontsize=16)
plt.show()
