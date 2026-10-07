import numpy as np
import time
from numba import njit
from scipy.optimize import differential_evolution
import matplotlib.pyplot as plt

# ==========================================================
# 1. LE MOTEUR DE SIMULATION (Compilé en C++ via Numba)
# ==========================================================
@njit(fastmath=True) # C'est cette ligne qui accélère tout x100
def simulation_rapide(params):
    Kp, Ki, Kd = params
    
    # --- Paramètres ---
    K_g, tau_g = 2.3, 0.15
    dt = 0.01
    t_max = 20.0
    n_steps = int(t_max / dt)
    
    consigne_base = 12.56
    
    # Kalman Init
    x_hat = 0.0
    P = 1.0
    Q = 0.001
    R = 0.01
    
    # État système
    reponse = 0.0
    integrale = 0.0
    erreur_prec = 0.0
    x_pos = 0.0
    
    # Score accumulator
    score_itae = 0.0
    max_depassement = 0.0
    sum_oscillation = 0.0
    
    # Boucle temporelle (Optimisée sans listes Python)
    for i in range(n_steps):
        t = i * dt
        
        # Consigne & Perturbation (Scénario mixte "hardcore")
        
        
        # Scénario : Rampe -> Choc -> Pente
        consigne = consigne_base
        if t < 1.0: consigne = consigne_base * (t / 1.0)
        pert = 0.2 * np.random.normal() + \
        (-2 if 1.25 + 0.5*np.pi < x_pos % (4 + np.pi) < 1.75 + 0.5*np.pi else 0) + \
        (2 if 2.25 + 0.5*np.pi < x_pos % (4 + np.pi) < 2.75 + 0.5*np.pi else 0) + \
        0.1 * np.sin(2 * np.pi * 0.3 * t)
        
        # 1. Mesure bruitée
        mesure = reponse + np.random.normal(0, np.sqrt(R))
        
        # 2. Kalman (Scalaire)
        # Prédiction
        x_hat = x_hat + (K_g * (Kp*(consigne-x_hat)) + pert - x_hat) * (dt / tau_g)
        P = P + Q
        # Correction
        K_gain = P / (P + R)
        x_hat = x_hat + K_gain * (mesure - x_hat)
        P = (1 - K_gain) * P
        
        # 3. PID
        erreur = consigne - x_hat
        integrale += erreur * dt
        # Clamp intégral (manuellement car np.clip est parfois lent)
        if integrale > 10: integrale = 10
        elif integrale < -10: integrale = -10
            
        derivee = (erreur - erreur_prec) / dt
        erreur_prec = erreur
        
        commande = Kp * erreur + Ki * integrale + Kd * derivee
        if commande > 20: commande = 20
        elif commande < -20: commande = -20
            
        # 4. Physique RK4 (Inline pour vitesse)
        # k1
        acc1 = (K_g * commande + pert - reponse) / tau_g
        v1 = reponse
        # k2
        acc2 = (K_g * commande + pert - (v1 + 0.5*dt*acc1)) / tau_g
        # k3
        acc3 = (K_g * commande + pert - (v1 + 0.5*dt*acc2)) / tau_g
        # k4
        acc4 = (K_g * commande + pert - (v1 + dt*acc3)) / tau_g
        
        reponse += (dt/6.0) * (acc1 + 2*acc2 + 2*acc3 + acc4)
        
        # 5. Calcul Score en direct (pour éviter de stocker des tableaux)
        x_pos += 0.03 * reponse * dt
        
        err_abs = abs(consigne - reponse)
        score_itae += (t * err_abs) * dt
        
        # Pénalité dépassement
        dep = reponse - consigne
        if dep > max_depassement: max_depassement = dep
        
        # Pénalité oscillation (simplifiée : variation de l'erreur)
        if t > 15.0: # On regarde la stabilité à la fin
            sum_oscillation += abs(dep)

    # Score Final
    score_total = score_itae
    if max_depassement > 1.0: score_total += 500 * max_depassement
    score_total += 10 * sum_oscillation # Pénalité instabilité fin de course
    
    return score_total

# ==========================================================
# 2. L'OPTIMISATEUR (Scipy)
# ==========================================================
def lancer_optimisation():
    print("Compilation du code Numba (quelques secondes)...")
    # Appel à vide pour forcer la compilation JIT avant le chronomètre
    simulation_rapide(np.array([2.0, 5.0, 0.01])) 
    
    print("Lancement de l'Évolution Différentielle...")
    start = time.time()
    
    # Bornes de recherche : [(Kp_min, Kp_max), (Ki_min, Ki_max), (Kd_min, Kd_max)]
    bounds = [(1, 10), (0, 20), (0.0, 0.2)]
    
    # L'algorithme magique
    resultat = differential_evolution(
        simulation_rapide, 
        bounds, 
        strategy='best1bin', 
        maxiter=50000,      # Nombre de générations
        popsize=15000,      # Taille population (15 * 3 paramètres = 45 individus)
        tol=0.01,        # Arrêt quand on ne trouve plus mieux
        mutation=(0.1, 1), 
        recombination=0.2,
        workers=-1,      # UTILISE TOUS LES COEURS DU CPU (-1)
        disp=True        # Affiche la progression
    )
    
    end = time.time()
    
    print("-" * 50)
    print(f"OPTIMISATION TERMINÉE en {end - start:.2f} secondes")
    print("-" * 50)
    print(f"Meilleur Score : {resultat.fun:.4f}")
    print(f"Meilleurs Gains : Kp={resultat.x[0]:.4f} | Ki={resultat.x[1]:.4f} | Kd={resultat.x[2]:.6f}")
    
    return resultat.x

if __name__ == "__main__":
    meilleurs_gains = lancer_optimisation()
    
    # Sauvegarde
    with open("meilleurs_pid_numba.txt", "a") as f:
        f.write(f"Kp={meilleurs_gains[0]}\tKi={meilleurs_gains[1]}\tKd={meilleurs_gains[2]}")