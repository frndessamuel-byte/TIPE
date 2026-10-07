import sys
import os
sys.path.append(r'C:\Users\ferna\OneDrive\Bureau\TIPE\simulation\V2')
import fonction_pid_V2
import time
import torch


device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"🖥️ Dispositif actif : {device}")

N = 1  # Nombre d'essais
TOP_N = 1  # Meilleurs PID à conserver

meilleur_score = float("inf")
meilleur_PID = None
meilleure_sortie = None

historique = []

start = time.time()

for _ in range(N):
    print(N-_)
    # Génération de coefficients PID directement sur le GPU
    Kp = torch.rand(1, device=device) * (10 - 7) + 7
    Ki = torch.rand(1, device=device) * (3 - 1) + 1
    Kd = torch.rand(1, device=device) * (0.08 - 0.03) + 0.03

    # Simulation PID sur GPU
    itae, sortie_r, Cr = fonction_pid_V2.simuler_pid(Kp, Ki, Kd, Kp, Ki, Kd, device=device)

    # Analyse de la réponse avec torch
    sortie_r_torch = torch.tensor(sortie_r, device=device)
    Cr_torch = torch.tensor(Cr, device=device)
    erreur_r = Cr_torch - sortie_r_torch

    depassement = torch.max(sortie_r_torch) - torch.max(Cr_torch)
    oscillation = torch.std(erreur_r[-int(len(erreur_r) * 0.2):])

    # Score combiné
    score_total = itae + 10 * depassement.item() + 20 * oscillation.item()

    if depassement > 1 or oscillation > 1:
        continue  # Trop instable → rejeté

    historique.append((score_total, Kp.item(), Ki.item(), Kd.item()))

    # Mettre à jour le meilleur
    if score_total < meilleur_score:
        meilleur_score = score_total
        meilleur_PID = (Kp.item(), Ki.item(), Kd.item())
        meilleure_sortie = sortie_r

# Trier les meilleurs PID
historique_trie = sorted(historique, key=lambda x: x[0])[:TOP_N]

# Écriture dans un fichier
with open("C:/Users/ferna/OneDrive/Bureau/TIPE/simulation/V2/scores_tries_torch.txt", "a") as f:
    for score, kp, ki, kd in historique_trie:
        f.write(f"{kp:.6f}\t{ki:.6f}\t{kd:.6f}\t{score:.6f}\n")

# Affichage final
print(f"\n✅ Meilleur PID : Kp={meilleur_PID[0]:.3f}, Ki={meilleur_PID[1]:.3f}, Kd={meilleur_PID[2]:.3f}")
print(f"🔢 Score total : {meilleur_score:.3f}")
print(f"⏱️ Temps écoulé : {time.time() - start:.2f} s")
