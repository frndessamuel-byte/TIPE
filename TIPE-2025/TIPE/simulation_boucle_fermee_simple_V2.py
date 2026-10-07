import numpy as np
import matplotlib.pyplot as plt

# Paramètres système
K = 1.0
tau = 0.5
dt = 0.01
t_max = 2

# PID (proportionnel seulement)
Kp = 2.5

# Temps et initialisation
t = np.arange(0, t_max, dt)
commande = 100
reponse = 0
sortie = []

for _ in t:
    erreur = commande - reponse
    signal = Kp * erreur              # régulation proportionnelle
    reponse += (K * signal - reponse) * (dt / tau)
    sortie.append(reponse)

# Affichage
plt.figure(figsize=(8, 4))
plt.plot(t, sortie, label=f"Réponse (Kp = {Kp})")
plt.axhline(commande, color="r", linestyle="--", label="Consigne")
plt.title("Boucle fermée proportionnelle - Réponse avec Kp")
plt.xlabel("Temps (s)")
plt.ylabel("Sortie")
plt.grid(True)
plt.legend()
plt.show()