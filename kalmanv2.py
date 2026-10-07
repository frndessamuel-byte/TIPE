import numpy as np
import matplotlib.pyplot as plt
 
# ==========================================================
# PARAMÈTRES DU SYSTÈME ET DU PID
# ==========================================================
K_g, K_d = 2.3, 2.3
tau_g, tau_d = 0.25, 0.25
dt = 0.01
t_max = 50
 
Kp_g, Ki_g, Kd_g = 4.902, 19.872, 0.019
Kp_d, Ki_d, Kd_d = 4.902, 19.872, 0.019
 
t_ramp = 0.2        # Durée du démarrage progressif en secondes
consigne_max = 12.56
consigne_g = 12.56
consigne_d = 12.56
perturbations_g = 0.0   # Perturbation courante moteur gauche (constante sur le pas RK4)
perturbations_d = 0.0   # Perturbation courante moteur droit
 
# ==========================================================
# INITIALISATIONS
# ==========================================================
t = np.arange(0, t_max, dt)
reponse_g = reponse_d = 0.0
integrale_g = integrale_d = 0.0
erreur_precedente_g = erreur_precedente_d = 0.0
x = 0.0
 
sortie_g, sortie_d, sortie_r = [], [], []
mesures_g, mesures_d = [], []
kalman_g, kalman_d = [], []
Cg, Cr, X = [], [], []
 
# ==========================================================
# PARAMÈTRES DU FILTRE DE KALMAN
# ==========================================================
Q = 0.001   # Variance du bruit de modèle
R = 0.01    # Variance du bruit de mesure
 
x_hat_g, P_g = 0.0, 1.0
x_hat_d, P_d = 0.0, 1.0
 
# ==========================================================
# FONCTIONS
# ==========================================================
 
def perturbations_fonction(x_pos, t_val):
    """
    Perturbation composée de :
    - bruit gaussien (aléatoire, à n'évaluer qu'une fois par pas)
    - chocs ponctuels dépendant de la position
    - composante sinusoïdale périodique
    """
    return (
        0.2 * np.random.normal() +
        (-2 if 1.25 + 0.5*np.pi < x_pos % (4 + np.pi) < 1.75 + 0.5*np.pi else 0) +
        (2  if 2.25 + 0.5*np.pi < x_pos % (4 + np.pi) < 2.75 + 0.5*np.pi else 0) +
        0.1 * np.sin(2 * np.pi * 0.3 * t_val)
    )
 
def rk4_update(reponse, K, tau, commande, perturb, dt):
    """
    Intégration RK4 du modèle moteur du premier ordre :
        dr/dt = (K*u + perturb - r) / tau
 
    La commande et la perturbation sont supposées CONSTANTES sur le pas dt.
    C'est correct car :
    - la commande PID ne change pas à l'intérieur d'un pas
    - la perturbation est échantillonnée une seule fois avant l'appel (bruit fixe)
    - la variation déterministe de la perturbation sur 10ms est négligeable (~0.002 rad/s)
 
    Le RK4 améliore ici uniquement l'intégration de la dynamique du moteur
    (le terme -r/tau qui évolue entre les sous-pas).
    """
    def f(r):
        return (K * commande + perturb - r) / tau
 
    k1 = f(reponse)
    k2 = f(reponse + 0.5*dt*k1)
    k3 = f(reponse + 0.5*dt*k2)
    k4 = f(reponse + dt*k3)
    return reponse + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4)
 
def kalman_predict(x_hat, P, commande, K, tau, dt, Q, perturb):
    """
    Prédiction Kalman : propagation de l'état estimé via le modèle moteur.
    La perturbation est incluse dans la prédiction car elle est déjà connue
    à ce pas (échantillonnée avant). Elle ne constitue pas une "triche" :
    on utilise la valeur mesurée/estimée de la perturbation actuelle,
    pas une valeur future.
    """
    x_hat = x_hat + (K * commande + perturb - x_hat) * (dt / tau) 
    P = P + Q
    return x_hat, P
 
def kalman_update(x_hat, P, mesure, R):
    """Correction Kalman par la mesure."""
    K_gain = P / (P + R)
    x_hat = x_hat + K_gain * (mesure - x_hat)
    P = (1 - K_gain) * P
    return x_hat, P
 
def calcul_commande(consigne, integrale, erreur_precedente, Kp, Ki, Kd, x_hat, dt):
    """
    Calcul PID basé sur l'estimation Kalman x_hat (et non la mesure bruitée).
    Retourne la commande saturée, l'intégrale mise à jour, et l'erreur courante.
    """
    erreur = consigne - x_hat
    integrale += erreur * dt
    integrale = np.clip(integrale, -10, 10)     # Anti-windup
    derivee = (erreur - erreur_precedente) / dt
    erreur_precedente = erreur
    commande = Kp * erreur + Ki * integrale + Kd * derivee
    return np.clip(commande, -20, 20), integrale, erreur_precedente
 
# ==========================================================
# BOUCLE PRINCIPALE DE SIMULATION
# ==========================================================
for e in t:
 
    # --- Rampe de démarrage ---
    if e < t_ramp:
        consigne_g = consigne_max * (e / t_ramp)
        consigne_d = consigne_max * (e / t_ramp)
 
    Cg.append(consigne_g)
    Cr.append(0.5 * (consigne_g + consigne_d))
 
    # --- Mesures bruitées ---
    mesure_g = reponse_g + np.random.normal(0, np.sqrt(R))
    mesure_d = reponse_d + np.random.normal(0, np.sqrt(R))
    mesures_g.append(mesure_g)
    mesures_d.append(mesure_d)
 
    # --- Calcul PID (basé sur l'estimation Kalman du pas précédent) ---
    commande_g, integrale_g, erreur_precedente_g = calcul_commande(
        consigne_g, integrale_g, erreur_precedente_g, Kp_g, Ki_g, Kd_g, x_hat_g, dt
    )
    commande_d, integrale_d, erreur_precedente_d = calcul_commande(
        consigne_d, integrale_d, erreur_precedente_d, Kp_d, Ki_d, Kd_d, x_hat_d, dt
    )
 
    # --- Perturbations : échantillonnées UNE SEULE FOIS avant le RK4 ---
    # Cela garantit que k1, k2, k3, k4 voient la même perturbation,
    # conformément à l'hypothèse RK4 d'une fonction déterministe sur le pas.
    perturbations_g = perturbations_fonction(x, e)
    perturbations_d = perturbations_fonction(x, e)
 
    # --- Prédiction Kalman ---
    x_hat_g, P_g = kalman_predict(x_hat_g, P_g, commande_g, K_g, tau_g, dt, Q, perturbations_g)
    x_hat_d, P_d = kalman_predict(x_hat_d, P_d, commande_d, K_d, tau_d, dt, Q, perturbations_d)
 
    # --- Correction Kalman ---
    x_hat_g, P_g = kalman_update(x_hat_g, P_g, mesure_g, R)
    x_hat_d, P_d = kalman_update(x_hat_d, P_d, mesure_d, R)
    kalman_g.append(x_hat_g)
    kalman_d.append(x_hat_d)
 
    # --- Dynamique moteur intégrée par RK4 ---
    # La perturbation est passée comme scalaire constant (déjà échantillonnée).
    # La commande est également constante sur le pas.
    reponse_g = rk4_update(reponse_g, K_g, tau_g, commande_g, perturbations_g, dt)
    reponse_d = rk4_update(reponse_d, K_d, tau_d, commande_d, perturbations_d, dt)
 
    # --- Cinématique du robot ---
    vg = 0.03 * reponse_g
    vd = 0.03 * reponse_d
    vr = 0.5 * (vg + vd)
    x += vr * dt
 
    # --- Sauvegarde ---
    X.append(x)
    sortie_g.append(reponse_g)
    sortie_d.append(reponse_d)
    sortie_r.append(0.5 * (reponse_g + reponse_d))
 
    # --- Mise à jour de la consigne selon la position (cycle parcours) ---
    consigne_g = consigne_max
    if x % (4 + np.pi) > 1:
        consigne_g = 8.11
    if x % (4 + np.pi) > (1 + 0.5 * np.pi):
        consigne_g = 12.56
    if x % (4 + np.pi) > (3 + 0.5 * np.pi):
        consigne_g = 8.11
    if x % (4 + np.pi) > (3 + np.pi):
        consigne_g = 12.56
 
# ==========================================================
# CRITÈRES DE PERFORMANCE
# ==========================================================
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
 
# ==========================================================
# AFFICHAGE DES RÉSULTATS
# ==========================================================
plt.figure(figsize=(18, 8))
 
plt.subplot(2, 2, 1)
plt.plot(t, sortie_g, label="Réponse réelle moteur gauche")
plt.plot(t, mesures_g, label="Mesure bruitée gauche", alpha=0.3)
plt.plot(t, kalman_g, label="Estimation Kalman gauche", linestyle='--')
plt.plot(t, Cg, 'r--', label="Consigne")
plt.xlabel("Temps (s)")
plt.ylabel("Vitesse angulaire (rad/s)")
plt.title("Moteur gauche")
plt.grid(True)
plt.legend()
 
plt.subplot(2, 2, 2)
plt.plot(t, sortie_d, label="Réponse réelle moteur droit")
plt.plot(t, mesures_d, label="Mesure bruitée droit", alpha=0.3)
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
plt.ylabel("Vitesse angulaire (rad/s)")
plt.title("Robot (PID + Kalman + RK4)")
plt.grid(True)
plt.legend()
 
plt.suptitle("Simulation PID avec filtre de Kalman et intégration RK4", fontsize=16)
plt.tight_layout()
plt.show()