import numpy as np
import matplotlib.pyplot as plt

# PARAMÈTRES DU SYSTÈME ET DU PID
K_g, K_d = 2.3, 2.3
tau_g, tau_d = 0.25, 0.25
dt = 0.01
t_max = 50
Kp_g, Ki_g, Kd_g, Kp_d, Ki_d, Kd_d = 	18.9820,1.1087,0.006280, 18.9820,1.1087,0.006280

t_ramp = 0.2  # Durée du démarrage progressif en secondes
consigne_max = 12.56
consigne_g, consigne_d = 12.56, 12.56
perturbations = 0.0

# INITIALISATIONS
t = np.arange(0, t_max, dt)
reponse_g = reponse_d = 0.0
integrale_g = integrale_d = 0.0
erreur_precedente_g = erreur_precedente_d = 0.0
x = 0.0

sortie_g, sortie_d, sortie_r = [], [], []
mesures_g, mesures_d = [], []
kalman_g, kalman_d = [], []
Cg, Cr, X = [], [], []

# PARAMÈTRES DU FILTRE DE KALMAN
Q = 0.001  # bruit du modèle
R = 0.01   # bruit de mesure

x_hat_g, P_g = 0.0, 1.0
x_hat_d, P_d = 0.0, 1.0

# DÉFINITIONS DE FONCTIONS UTILES
def perturbations_fonction(x, t):
    """Perturbation dépendant de la position et du temps"""
    return (
        0.2 * np.random.normal() +
        (-2 if 1.25 + 0.5*np.pi < x % (4 + np.pi) < 1.75 + 0.5*np.pi else 0) +
        (2 if 2.25 + 0.5*np.pi < x % (4 + np.pi) < 2.75 + 0.5*np.pi else 0) +
        0.1 * np.sin(2 * np.pi * 0.3 * t)
    )

def rk4_calcul(reponse, K, tau, commande, perturb, dt):
    def f(r):
        return (K * commande + perturb - r) / tau
 
    k1 = f(reponse)
    k2 = f(reponse + 0.5*dt*k1)
    k3 = f(reponse + 0.5*dt*k2)
    k4 = f(reponse + dt*k3)
    return reponse + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4)

def kalman_predict(x_hat, P, commande, K, tau, dt, Q, perturb):
    """Étape de prédiction du filtre de Kalman"""
    x_hat = x_hat + (K *commande + perturb - x_hat) * (dt / tau)
    P = P + Q
    return x_hat, P

def kalman_correct(x_hat, P, mesure, R):
    """Étape de correction du filtre de Kalman"""
    K_gain = P / (P + R)
    x_hat = x_hat + K_gain * (mesure - x_hat)
    P = (1 - K_gain) * P
    return x_hat, P

def calcul_commande(consigne, integrale, erreur_precedente, Kp, Ki, Kd, x_hat):
    """Calcul de la réponse du système avec PID"""
    erreur = consigne - x_hat
    integrale += erreur * dt
    integrale = np.clip(integrale, -10, 10)
    derivee = (erreur - erreur_precedente) / dt
    erreur_precedente = erreur
    commande = Kp * erreur + Ki * integrale + Kd * derivee
    return np.clip(commande, -20, 20), integrale, erreur_precedente


# BOUCLE PRINCIPALE DE SIMULATION
for e in t:
    perturbations = perturbations_fonction(x, e)
    if e < t_ramp:
        # Augmentation linéaire de la consigne (Rampe)
        consigne_actuelle = consigne_max * (e / t_ramp)
        consigne_g = consigne_actuelle
        consigne_d = consigne_actuelle
    
    Cg.append(consigne_g)
    Cr.append(0.03*0.5 * (consigne_g + consigne_d))

    mesure_g = reponse_g + np.random.normal(0, np.sqrt(R))
    mesure_d = reponse_d + np.random.normal(0, np.sqrt(R))
    mesures_g.append(mesure_g)
    mesures_d.append(mesure_d)

    commande_g, integrale_g, erreur_precedente_g = calcul_commande(consigne_g, integrale_g, erreur_precedente_g, Kp_g, Ki_g, Kd_g, x_hat_g)
    commande_d, integrale_d, erreur_precedente_d = calcul_commande(consigne_d, integrale_d, erreur_precedente_d, Kp_d, Ki_d, Kd_d, x_hat_d)

    # Prédiction Kalman
    x_hat_g, P_g = kalman_predict(x_hat_g, P_g, commande_g, K_g, tau_g, dt, Q, perturbations)
    x_hat_d, P_d = kalman_predict(x_hat_d, P_d, commande_d, K_d, tau_d, dt, Q, perturbations)

    # Correction Kalman
    x_hat_g, P_g = kalman_correct(x_hat_g, P_g, mesure_g, R)
    x_hat_d, P_d = kalman_correct(x_hat_d, P_d, mesure_d, R)
    kalman_g.append(x_hat_g)
    kalman_d.append(x_hat_d)

    # Dynamique moteur intégrée par RK4
    reponse_g = rk4_calcul(reponse_g, K_g, tau_g, commande_g, perturbations, dt)
    reponse_d = rk4_calcul(reponse_d, K_d, tau_d, commande_d, perturbations, dt)
    # Perturbations évolutives
    perturbations = perturbations_fonction(x, e)

    # Vitesse et position du robot
    vg, vd = 0.03 * reponse_g, 0.03 * reponse_d
    vr = 0.5 * (vg + vd)
    x += vr * dt

    # Sauvegarde
    X.append(x)
    sortie_g.append(reponse_g)
    sortie_d.append(reponse_d)
    sortie_r.append(vr)

    # Mise à jour de la consigne
    consigne_g = consigne_max
    if x % (4 + np.pi) > 1:
        consigne_g = 8.11
    if x % (4 + np.pi) > (1 + 0.5 * np.pi):
        consigne_g = 12.56
    if x % (4 + np.pi) > (3 + 0.5 * np.pi):
        consigne_g = 8.11
    if x % (4 + np.pi) > (3 + np.pi):
        consigne_g = 12.56


# CRITÈRES DE PERFORMANCE
sortie_r = np.array(sortie_r)
Cr = np.array(Cr)
erreur_r = Cr - sortie_r

ISE  = np.trapezoid(erreur_r**2, t)
IAE  = np.trapezoid(np.abs(erreur_r), t)
ITSE = np.trapezoid(t * erreur_r**2, t)
ITAE = np.trapezoid(t * np.abs(erreur_r), t)

print("\n=== Évaluation des performances PID + Kalman avec intégration RK4 ===")
print(f"IAE   (∫|e|dt)     = {IAE:.4f}")
print(f"ISE   (∫e²dt)      = {ISE:.4f}")
print(f"ITAE  (∫t|e|dt)    = {ITAE:.4f}")
print(f"ITSE  (∫t·e²dt)    = {ITSE:.4f}")

# AFFICHAGE DES RÉSULTATS
plt.figure(figsize=(18, 8))

plt.subplot(2, 2, 1)
plt.plot(t, sortie_g, label="Réponse réelle moteur gauche")
plt.plot(t, mesures_g, label="Mesure bruitée gauche", alpha=0.5)
plt.plot(t, kalman_g, label="Estimation Kalman gauche", linestyle='--')
plt.plot(t, Cg, 'r--', label="Consigne")
plt.xlabel("Temps (s)")
plt.ylabel("Vitesse angulaire (rad/s)")
plt.title("Moteur gauche")
plt.grid(True)
plt.legend()

plt.subplot(2, 2, 2)
plt.plot(t, sortie_d, label="Réponse réelle moteur droit")
plt.plot(t, mesures_d, label="Mesure bruitée droit", alpha=0.5)
plt.plot(t, kalman_d, label="Estimation Kalman droit", linestyle='--')
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
plt.title("Robot (PID + Kalman + RK4)")
plt.grid(True)
plt.legend()

plt.suptitle("Figure 17 : ", fontsize=16)
plt.show()
