import numpy as np
import matplotlib.pyplot as plt

# Paramètres initiaux
K_base = 1.0
tau_base = 0.5
dt = 0.01
t_max = 2

# PID
Kp = 25
Ki = 50
Kd = 0.075

# Vecteurs de temps et initialisation
t = np.arange(0, t_max, dt)
commande = np.piecewise(t, [t < 1, t >= 1], [100, 150])  # consigne changeante

# Variables
integrale = 0
erreur_precedente = 0
reponse = 0

sortie = []
erreurs = []
commandes_pid = []
valeurs_K = []
valeurs_tau = []

# Simulation
for i in range(len(t)):
    erreur = commande[i] - reponse
    integrale += erreur * dt

    # Anti-windup (limiter l’intégrale si besoin)
    integrale = max(min(integrale, 500), -500)

    derivee = (erreur - erreur_precedente) / dt
    erreur_precedente = erreur

    # Commande PID
    consigne = Kp * erreur + Ki * integrale + Kd * derivee

    # K et tau dynamiques
    K = K_base + 0.0015 * consigne
    tau = max(tau_base - 0.002 * abs(consigne), 0.05)

    # Réponse du système (1er ordre)
    reponse += (K * consigne - reponse) * (dt / tau)

    # Sauvegarde des données
    sortie.append(reponse)
    erreurs.append(erreur)
    commandes_pid.append(consigne)
    valeurs_K.append(K)
    valeurs_tau.append(tau)

# Affichage
plt.figure(figsize=(12, 8))

plt.subplot(2, 1, 1)
plt.plot(t, sortie, label="Réponse du système")
plt.plot(t, commande, 'r--', label="Consigne")
plt.title("Simulation PID avec K et tau variables")
plt.xlabel("Temps (s)")
plt.ylabel("Valeur de sortie")
plt.legend()
plt.grid(True)

plt.subplot(2, 2, 3)
plt.plot(t, commandes_pid, label="Commande PID")
plt.xlabel("Temps (s)")
plt.ylabel("Effort de commande")
plt.grid(True)
plt.legend()

plt.subplot(2, 2, 4)
plt.plot(t, valeurs_K, label="K dynamique")
plt.plot(t, valeurs_tau, label="Tau dynamique")
plt.xlabel("Temps (s)")
plt.ylabel("Valeurs")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()