import matplotlib.pyplot as plt
import numpy as np

# Génération du temps
t = np.linspace(0, 10, 200)

# Simulation d'une réponse sous PID fixe
error_pid = np.exp(-t) * (1 + 0.3 * np.sin(2 * np.pi * t))

# Simulation d'une réponse avec LSTM adaptatif (moins oscillatoire)
error_lstm = np.exp(-t)

plt.figure(figsize=(10, 6))
plt.plot(t, error_pid, label='PID fixe', linestyle='--', color='red')
plt.plot(t, error_lstm, label='LSTM adaptatif', color='green')
plt.xlabel('Temps (s)')
plt.ylabel("Erreur d'asservissement")
plt.title("Comparaison de l'erreur de suivi")
plt.legend()
plt.grid(True)
plt.show()


# Simulation de l'évolution des gains PID ajustés par le LSTM
Kp_lstm = 1.5 + 0.2 * np.sin(0.5 * np.pi * t)
Ki_lstm = 0.5 + 0.1 * np.sin(0.3 * np.pi * t)
Kd_lstm = 0.1 + 0.05 * np.cos(0.6 * np.pi * t)

plt.figure(figsize=(10, 6))
plt.plot(t, Kp_lstm, label='Kp (LSTM)', color='blue')
plt.plot(t, Ki_lstm, label='Ki (LSTM)', color='orange')
plt.plot(t, Kd_lstm, label='Kd (LSTM)', color='purple')
plt.xlabel('Temps (s)')
plt.ylabel("Valeur des gains")
plt.title("Évolution des gains PID ajustés par le LSTM")
plt.legend()
plt.grid(True)
plt.show()
