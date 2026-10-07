import sys
import os
sys.path.append(r'C:\Users\ferna\OneDrive\Bureau\TIPE\simulation\V2')
import fonction_pid_V1
import numpy as np
import time

N = 10 # Nombre d'essais
# Initialisation des variables pour conserver le meilleur score et les paramètres PID correspondants
meilleur_score = float("inf")
meilleur_PID = None
meilleure_sortie = None
meilleure_consigne = None 

# Début du chronométrage de l'exécution
c = time.time()

for _ in range(N):
    # Génération aléatoire de coefficients PID pour gauche et droite
    Kp_g = np.random.uniform(7, 10)
    Kp_d = Kp_g  # Assure que les coefficients sont identiques pour les deux côtés
    Ki_g = np.random.uniform(1, 3)
    Ki_d = Ki_g  
    Kd_g = np.random.uniform(0.03, 0.04)
    Kd_d = Kd_g 

    # Simulation PID avec les coefficients générés
    score_itae, sortie_r, Cr = fonction_pid_V1.simuler_pid(Kp_g, Ki_g, Kd_g, Kp_d, Ki_d, Kd_d)
    

    erreur_r = Cr - sortie_r
    
    # Calcul du dépassement 
    depassement = np.max(sortie_r) - np.max(Cr)
    # Calcul de l'oscillation sur les 20% finaux de la trajectoire
    oscillation = np.std(erreur_r[-int(len(erreur_r) * 0.2):])
    
    # Calcul d'un score total combinant score ITAE, dépassement et oscillation
    score_total = score_itae + 10 * depassement + 20 * oscillation

    # Si le système est trop instable (dépassement ou oscillation trop élevés), on rejette la solution
    if depassement > 1 or oscillation > 1:
        continue

    # Affichage du score total pour la configuration actuelle
    print(score_total)

    # Mise à jour du meilleur score et des paramètres PID si le score total actuel est inférieur
    if score_total < meilleur_score:
        meilleur_score = score_total
        meilleur_PID = (Kp_g, Ki_g, Kd_g, Kp_d, Ki_d, Kd_d) 
        meilleure_sortie = sortie_r

    # Enregistrement progressif des meilleurs PID trouvés dans un fichier texte
    with open('C:/Users/ferna/OneDrive/Bureau/TIPE/simulation/V2/scores.txt', "a") as s:
        s.write("\n" + str(meilleur_PID[0]) + " " + str(meilleur_PID[1]) + " " + str(meilleur_PID[2]) + "\t" + str(meilleur_score))

# Affichage du meilleur PID trouvé et de son score après toutes les tentatives
print("\n=== Meilleur PID trouvé ===")
print("Kp_g, Ki_g, Kd_g = " + str(meilleur_PID[0]) + str(meilleur_PID[1]) + str(meilleur_PID[2]))
print("\nScore = " + str(meilleur_score))

# Affichage du temps total d'exécution de la simulation
print(time.time() - c)