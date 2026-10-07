import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import pandas as pd
from datetime import datetime

# ==========================================================
# 1. CONFIGURATION
# ==========================================================
# Chemin vers votre fichier de mesures
FICHIER_MESURES = "C:/Users/ferna/OneDrive/Bureau/TIPE/simulation/V4_test_programme/mes.ouv.sanspert.txt" 

# La commande que vous avez envoyée pour obtenir ces mesures
# Si c'était analogWrite(155) sur 255 -> U0 = 155
# Si c'était en Volts (6V) -> U0 = 6.0
# IMPORTANT : K sera calculé par rapport à cette valeur.
U0_COMMANDE = 155.0 

# ==========================================================
# 2. LECTURE ET NETTOYAGE DES DONNÉES
# ==========================================================
def charger_donnees(chemin):
    temps = []
    vitesse_g = []
    vitesse_d = []
    
    # Format de date dans votre fichier : HH:MM:SS.mmm
    format_date = "%H:%M:%S.%f"
    
    with open(chemin, 'r') as f:
        for ligne in f:
            try:
                # Structure: "20:53:24.985 0 , 0"
                parties = ligne.replace(',', ' ').split()
                if len(parties) < 3: continue
                
                # Parsing du temps
                t_obj = datetime.strptime(parties[0], format_date)
                # Conversion en secondes totales pour calcul relatif
                t_sec = t_obj.hour * 3600 + t_obj.minute * 60 + t_obj.second + t_obj.microsecond / 1e6
                
                temps.append(t_sec)
                vitesse_g.append(float(parties[1])) # Moteur Gauche
                vitesse_d.append(float(parties[2])) # Moteur Droit
                
            except ValueError:
                continue

    return np.array(temps), np.array(vitesse_g), np.array(vitesse_d)

t_raw, v_g_raw, v_d_raw = charger_donnees(FICHIER_MESURES)

# --- Détection du démarrage (t=0) ---
# On coupe tout ce qui est avant que le moteur commence à bouger
seuil_demarrage = 1.0 # On commence dès que la vitesse > 1
indices_mouvement = np.where(v_g_raw > seuil_demarrage)[0]

if len(indices_mouvement) == 0:
    print("Erreur : Aucun mouvement détecté dans le fichier.")
    exit()

start_index = indices_mouvement[0] - 1 # On prend juste un point avant pour voir la montée
if start_index < 0: start_index = 0

# Recalage temporel (t commence à 0)
t_data = t_raw[start_index:] - t_raw[start_index]
v_data = v_g_raw[start_index:] # On travaille sur le Moteur Gauche (ou faire la moyenne)

# On filtre la fin si le robot s'arrête ou tape un mur (optionnel)
# Ici on prend tout le reste du fichier

# ==========================================================
# 3. MODÉLISATION ET OPTIMISATION
# ==========================================================

# Modèle du premier ordre : V(t) = K * U0 * (1 - exp(-t/tau))
def reponse_echelon(t, K, tau):
    return K * U0_COMMANDE * (1 - np.exp(-t / tau))

print("Optimisation en cours...")

# Estimation initiale [K, tau]
# K approx = Vitesse_max / Commande
# Tau approx = Temps pour arriver à 63%
p0 = [np.max(v_data)/U0_COMMANDE, 0.2]

# curve_fit trouve les paramètres qui minimisent l'erreur carrée
params_optimaux, covariance = curve_fit(reponse_echelon, t_data, v_data, p0=p0)

K_estime, tau_estime = params_optimaux

# ==========================================================
# 4. RÉSULTATS ET AFFICHAGE
# ==========================================================
print("-" * 40)
print(f"RÉSULTATS IDENTIFICATION (Moteur Gauche)")
print("-" * 40)
print(f"Commande envoyée (U0) : {U0_COMMANDE}")
print(f"Gain Statique (K)     : {K_estime:.5f} (RPM/UnitéCommande)")
print(f"Constante de temps (τ): {tau_estime:.5f} secondes")
print("-" * 40)
print("Copiez ces valeurs dans votre simulation Python !")

# Génération de la courbe modèle pour le graphique
v_modele = reponse_echelon(t_data, K_estime, tau_estime)

plt.figure(figsize=(10, 6))
plt.scatter(t_data, v_data, color='blue', s=10, label='Mesures Réelles (Bruitées)', alpha=0.6)
plt.plot(t_data, v_modele, color='red', linewidth=2, label=f'Modèle Identifié\nK={K_estime:.3f}, $\\tau$={tau_estime:.3f}s')

plt.title("Identification des paramètres du robot (Boucle Ouverte)")
plt.xlabel("Temps (s)")
plt.ylabel("Vitesse (RPM)") # D'après vos données (valeurs ~18), ce sont sûrement des RPM à la roue
plt.legend()
plt.grid(True, which='both', linestyle='--')
plt.tight_layout()
plt.show()