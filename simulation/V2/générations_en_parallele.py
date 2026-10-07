import numpy as np
import time
import concurrent.futures
import heapq
import sys
import os
sys.path.append(r'C:\Users\ferna\OneDrive\Bureau\TIPE\simulation\V2')
import fonction_pid_V1  # Ce fichier doit contenir uniquement des fonctions

def tester_combinaison(_):
    # Crée un générateur local par appel (ou pense à le créer une seule fois par processus via un setup)
    rng = np.random.default_rng()
    
    # Génération aléatoire des coefficients PID
    Kp = rng.uniform(0, 10)    # Intervalle pour Kp
    Ki = rng.uniform(0, 10)   # Intervalle pour Ki
    Kd = rng.uniform(0, 0.2) # Intervalle pour Kd

    # Appel de la simulation avec les coefficients PID générés et récupération des résultats
    score_itae, sortie_r, Cr = fonction_pid_V1.simuler_pid(Kp, Ki, Kd, Kp, Ki, Kd)
    # Calcul de l'erreur entre la sortie prévue et la référence
    erreur_r = Cr - sortie_r

    # Calcul du dépassement et de l'oscillation (écart-type sur la dernière portion du signal)
    depassement = np.max(sortie_r) - np.max(Cr)
    oscillation = np.std(erreur_r[-int(len(erreur_r)*0.2):])

    # Score total combinant l'ITAE et les pénalités pour le dépassement et l'oscillation
    score_total = score_itae + 10 * depassement + 20 * oscillation

    # Filtrer les résultats qui dépassent les seuils fixés
    if depassement > 1 or oscillation > 1:
        return None
    return (score_total, Kp, Ki, Kd)

if __name__ == "__main__":
    # Nombre total d'essais
    N = 5000         
    # Nombre de meilleurs résultats à conserver
    TOP_N = 5    
    start = time.time()

    print("Lancement")

    # Détermination du nombre maximum de processus à utiliser
    max_workers = min(8, os.cpu_count() or 1)
    # Calcul d'une taille de chunk adaptée pour optimiser le traitement en parallèle
    chunk_size = max(N // (max_workers * 4), 1)

    # Utilisation de ProcessPoolExecutor pour lancer en parallèle les simulations
    with concurrent.futures.ProcessPoolExecutor(max_workers=max_workers) as executor:
        resultats = list(executor.map(tester_combinaison, range(N), chunksize=chunk_size))

    # Filtrer les résultats non valides (ceux pour lesquels la fonction retourne None)
    resultats = [r for r in resultats if r is not None]

    # Sélectionner les TOP_N meilleurs résultats en fonction du score total (le plus petit est le meilleur)
    resultats_tries = heapq.nsmallest(TOP_N, resultats, key=lambda x: x[0])

    # Chemin d'accès au fichier de scores
    chemin_scores = os.path.join('C:/Users/ferna/OneDrive/Bureau/TIPE/simulation/V2', 'scores.txt')
    
    # Écriture des scores et des valeurs PID dans le fichier scores.txt
    with open(chemin_scores, "a") as f:
        for score, kp, ki, kd in resultats_tries:
            f.write(f"{kp:.6f}\t{ki:.6f}\t{kd:.6f}\t{score:.6f}\n")

    # Affichage des résultats +: meilleur score et ses coefficients PID correspondants
    if resultats_tries:
        meilleur = resultats_tries[0]
        print(f"\nTerminé en {time.time() - start:.2f}s")
        print(f"Meilleur PID : Kp = {meilleur[1]:.3f}, Ki = {meilleur[2]:.3f}, Kd = {meilleur[3]:.3f}")
        print(f"Score total : {meilleur[0]:.3f}")
    else:
        print("Aucune combinaison PID valide trouvée.")