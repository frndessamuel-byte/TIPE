import numpy as np
import time
import concurrent.futures
import heapq
import sys
import os
sys.path.append(r'C:\Users\ferna\OneDrive\Bureau\TIPE\simulation\V4_test_programme')
import sim # On importe le nouveau fichier fonction

def tester_combinaison(_):
    rng = np.random.default_rng()
    Kp = rng.uniform(1, 10)
    Ki = rng.uniform(0, 20)
    Kd = rng.uniform(0, 0.1)
    try:
        score, kp_out, ki_out, kd_out = sim.simuler_systeme_complet(Kp, Ki, Kd)
        return (score, kp_out, ki_out, kd_out)
    except Exception:
        return None

if __name__ == "__main__":
    # Paramètres
    N = 50   # Nombre d'essais
    N_meilleurs = 5
    
    print(f"Lancement de l'optimisation sur {N} simulations (Modèle RK4 + Kalman)...")
    start = time.time()

    max_workers = os.cpu_count() or 4
    # chunk_size : compromise, pour tâches homogènes on peut augmenter pour réduire l'overhead
    chunk_size = max(1, N // (max_workers * 4))

    # On soumet N tâches et on traite les résultats au fur et à mesure
    top_heap = []  # heap de (-score, score, kp, ki, kd) pour garder N_meilleurs (scores les plus petits)
    with concurrent.futures.ProcessPoolExecutor(max_workers=max_workers) as executor:
        futures = [executor.submit(tester_combinaison, i) for i in range(N)]
        for i, fut in enumerate(concurrent.futures.as_completed(futures), 1):
            res = fut.result()
            if res is None:
                continue
            score, kp, ki, kd = res
            # on stocke en tant que -score pour obtenir un max-heap simulé avec heapq (on garde les N_meilleurs plus petits scores)
            entry = (-score, score, kp, ki, kd)
            if len(top_heap) < N_meilleurs:
                heapq.heappush(top_heap, entry)
            else:
                # si le nouveau est meilleur que le pire du tas, on remplace
                if entry > top_heap[0]:
                    heapq.heapreplace(top_heap, entry)
            # affichage de progression léger
            if i % 500 == 0 or i == N:
                elapsed = time.time() - start
                print(f"Traité {i}/{N}  —  elapsed {elapsed:.1f}s")

    # Récupère les meilleurs triés par score croissant
    meilleurs = sorted([ (entry[1], entry[2], entry[3], entry[4]) for entry in top_heap ], key=lambda x: x[0])
    print(f"\nTerminé en {time.time() - start:.2f} secondes")
    print("-" * 50)
    print("TOP " + str(N_meilleurs) + " :")
    print("-" * 50)
    for i, (score, kp, ki, kd) in enumerate(meilleurs, 1):
        print(f"#{i} | Score: {score:.2f} | Kp: {kp:.4f} | Ki: {ki:.4f} | Kd: {kd:.5f}")

    # chemin_scores = r'C:\Users\ferna\OneDrive\Bureau\TIPE\simulation\V4_test_programme\scores_kalman.txt'
    # with open(chemin_scores, "a") as f:
    #     for score, kp, ki, kd in meilleurs:
    #         f.write(f"{score:.4f}\t{kp:.3f}\t{ki:.3f}\t{kd:.3f}\n")

    # print(f"\nRésultats sauvegardés dans {chemin_scores}")