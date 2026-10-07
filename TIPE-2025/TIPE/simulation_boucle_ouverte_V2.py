import numpy as np
import matplotlib.pyplot as plt

# Paramètres système
K = 1.0       # gain statique
tau = 0.5     # constante de temps
dt = 0.01     # pas de temps
t_max = 2     # durée de la simulation

# Consigne fixe (ex: PWM simulé)
commande = 100  

t = np.arange(0, t_max, dt)
reponse = 0
sortie = []

for _ in t:
    reponse += (K * commande - reponse) * (dt / tau)
    sortie.append(reponse)

# Affichage
plt.figure(figsize=(8, 4))
plt.plot(t, sortie, label="Réponse (boucle ouverte)")
plt.axhline(commande * K, color="r", linestyle="--", label="Consigne * K")
plt.title("Boucle ouverte - Réponse d’un moteur sans régulation")
plt.xlabel("Temps (s)")
plt.ylabel("Sortie")
plt.grid(True)
plt.legend()
plt.show()