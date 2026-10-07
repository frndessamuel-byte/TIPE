import numpy as np
import time
import concurrent.futures
import heapq
import sys
import os
sys.path.append(r'C:\Users\ferna\OneDrive\Bureau\TIPE\simulation\V4_test_programme')
import sim # On importe le nouveau fichier fonction

def tester_combinaison(_):
    # Générateur aléatoire local
    rng = np.random.default_rng()
    
    # Plages de recherche (A ajuster selon vos intuitions précédentes)
    Kp = rng.uniform(1, 10)    
    Ki = rng.uniform(0, 20)   
    Kd = rng.uniform(0, 0.1) 

    try:
        # Appel de la simulation KALMAN
        score, kp_out, ki_out, kd_out = sim.simuler_systeme_complet(Kp, Ki, Kd)
        return (score, kp_out, ki_out, kd_out)
    except Exception as e:
        return None

if __name__ == "__main__":
    # Paramètres
    N = 50  # Nombre d'essais (Réduire car RK4 est plus lent qu'Euler)
    TOP_N = 5
    
    print(f"Lancement de l'optimisation sur {N} simulations (Modèle RK4 + Kalman)...")
    start = time.time()

    # Utilisation de tous les coeurs CPU
    max_workers = os.cpu_count() or 4
    # plus d'équilibrage (plus d'overhead)
    chunk_size = 1

    # compromis classique : une fenêtre par coeur
    chunk_size = max(N // max_workers, 1)

    # moins d'overhead (groupe plus gros)
    chunk_size = max(N // (max_workers * 8), 1)

    with concurrent.futures.ProcessPoolExecutor(max_workers=max_workers) as executor:
        # Lancement parallèle
        resultats = list(executor.map(tester_combinaison, range(N), chunksize=chunk_size))

    # Nettoyage des résultats None
    resultats = [r for r in resultats if r is not None]

    # Tri des meilleurs résultats (Score le plus bas est le mieux pour ITAE)
    meilleurs = heapq.nsmallest(TOP_N, resultats, key=lambda x: x[0])

    print(f"\nTerminé en {time.time() - start:.2f} secondes")
    print("-" * 50)
    print("TOP 3 DES MEILLEURS REGLAGES PID (Robustes au bruit) :")
    print("-" * 50)
    
    for i, (score, kp, ki, kd) in enumerate(meilleurs):
        print(f"#{i+1} | Score: {score:.2f} | Kp: {kp:.4f} | Ki: {ki:.4f} | Kd: {kd:.5f}")

    # Sauvegarde
    chemin_scores = r'C:\Users\ferna\OneDrive\Bureau\TIPE\simulation\V4_test_programme\scores_kalman.txt'
    with open(chemin_scores, "a") as f:
        for score, kp, ki, kd in meilleurs:
            f.write(f"{score:.4f}\t{kp:.3f}\t{ki:.3f}\t{kd:.3f}\n")
    
    print(f"\nRésultats sauvegardés dans {chemin_scores}")