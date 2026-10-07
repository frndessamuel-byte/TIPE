import numpy as np
import matplotlib.pyplot as plt

# ==========================================================
# PARAMÈTRES DU SYSTÈME
# ==========================================================
K_g, K_d = 2.3, 2.3        # gains statiques des moteurs gauche et droit
tau_g, tau_d = 0.25, 0.25  # constantes de temps des moteurs gauche et droit      
dt = 0.01                  # pas de temps
t_max = 50                 # durée de la simulation

# Coefficients PID
Kp_g = 9
Kp_d = 9

# ==========================================================
# CONDITIONS INITIALES
# ==========================================================
t = np.arange(0, t_max, dt)
reponse_g = reponse_d = 0.0
consigne_g = consigne_d = 12.56637061  # rad/s initialement
perturbations = 0.0

# Variables de stockage
sortie_g, sortie_d, sortie_r = [], [], []
Cg, Cr, X = [], [], []
x = 0.0

# États PID
integrale_g = integrale_d = 0.0
erreur_precedente_g = erreur_precedente_d = 0.0

# ==========================================================
# DÉFINITIONS DES FONCTIONS UTILES
# ==========================================================
def perturbations_fonction(x, t):
    """Renvoie la perturbation actuelle selon la position et le temps."""
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
    """Intégration RK4 avec réévaluation dynamique de la commande et des perturbations."""
    k1 = f_reponse(reponse, K, commande_fn(t), perturb_fn(t), tau)
    k2 = f_reponse(reponse + 0.5*dt*k1, K, commande_fn(t + 0.5*dt), perturb_fn(t + 0.5*dt), tau)
    k3 = f_reponse(reponse + 0.5*dt*k2, K, commande_fn(t + 0.5*dt), perturb_fn(t + 0.5*dt), tau)
    k4 = f_reponse(reponse + dt*k3, K, commande_fn(t + dt), perturb_fn(t + dt), tau)
    return reponse + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)

def calcul_commande(reponse, K, consigne, Kp):
        erreur= consigne - (reponse + np.random.normal(0, 0.1)) # ajout de bruit de mesure
        commande = Kp * erreur
        return np.clip(commande, -20, 20)
# ==========================================================
# BOUCLE PRINCIPALE DE SIMULATION
# ==========================================================
for e in t:
    # Sauvegarde des consignes actuelles
    Cg.append(consigne_g)
    Cr.append(0.5 * (consigne_g + consigne_d))

    commande_g = calcul_commande(reponse_g, K_g, consigne_g, Kp_g)
    commande_d = calcul_commande(reponse_d, K_d, consigne_d, Kp_d)

    # Perturbations dynamiques
    perturbations = perturbations_fonction(x, e)

    # Intégration dynamique avec RK4
    reponse_g = rk4_update(
        reponse_g, K_g, tau_g,
        lambda t_: commande_g,       # ici commande actuelle (varie lentement)
        lambda t_: perturbations_fonction(x, t_),
        e, dt
    )
    reponse_d = rk4_update(
        reponse_d, K_d, tau_d,
        lambda t_: commande_d,
        lambda t_: perturbations_fonction(x, t_),
        e, dt
    )

    # Calcul des vitesses et de la position du robot
    vg, vd = 0.03 * reponse_g, 0.03 * reponse_d
    vr = 0.5 * (vg + vd)
    x += vr * dt

    # Sauvegarde des sorties
    X.append(x)
    sortie_g.append(reponse_g)
    sortie_d.append(reponse_d)
    sortie_r.append(0.5 * (reponse_g + reponse_d))

    # Mise à jour de la consigne du moteur gauche (cycle périodique)
    if x % (4 + np.pi) > 1:
        consigne_g = 8.11
    if x % (4 + np.pi) > (1 + 0.5 * np.pi):
        consigne_g = 12.56
    if x % (4 + np.pi) > (3 + 0.5 * np.pi):
        consigne_g = 8.11
    if x % (4 + np.pi) > (3 + np.pi):
        consigne_g = 12.56

# ==========================================================
# CALCUL DES CRITÈRES DE PERFORMANCE
# ==========================================================
sortie_r = np.array(sortie_r)
Cr = np.array(Cr)
erreur_r = Cr - sortie_r

ISE  = np.trapezoid(erreur_r**2, t)
IAE  = np.trapezoid(np.abs(erreur_r), t)
ITSE = np.trapezoid(t * erreur_r**2, t)
ITAE = np.trapezoid(t * np.abs(erreur_r), t)

print("\n=== Évaluation des performances PID avec intégration RK4 ===")
print(f"IAE   (∫|e|dt)     = {IAE:.4f}")
print(f"ISE   (∫e²dt)      = {ISE:.4f}")
print(f"ITAE  (∫t|e|dt)    = {ITAE:.4f}")
print(f"ITSE  (∫t·e²dt)    = {ITSE:.4f}")

# ==========================================================
# AFFICHAGE DES RÉSULTATS
# ==========================================================
plt.figure(figsize=(20, 3))

plt.subplot(2, 2, 1)
plt.plot(t, sortie_g, label="Réponse (moteur gauche)")
plt.plot(t, Cg, 'r--', label="Consigne")
plt.xlabel("Temps (s)")
plt.ylabel("Vitesse angulaire (rad/s)")
plt.title("Moteur gauche")
plt.grid(True)
plt.legend()

plt.subplot(2, 2, 2)
plt.plot(t, sortie_d, label="Réponse (moteur droit)")
plt.axhline(y=12.56, color='r', linestyle='--', label="Consigne")
plt.xlabel("Temps (s)")
plt.ylabel("Vitesse angulaire (rad/s)")
plt.title("Moteur droit")
plt.grid(True)
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(t, sortie_r, label="Réponse robot")
plt.plot(t, Cr, 'r--', label="Consigne")
plt.xlabel("Temps (s)")
plt.ylabel("Vitesse angulaire (rad/s)")
plt.title("Robot (boucle fermée PID - méthode RK4)")
plt.grid(True)
plt.legend()

plt.subplots_adjust(top=0.9)
plt.suptitle("Figure 12 : Boucle fermée - Réponse des moteurs et du robot avec régulation Proportionnelle Kp=9", fontsize=16)
plt.show()
