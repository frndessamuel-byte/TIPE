import numpy as np
import time
from numba import njit
from scipy.optimize import differential_evolution
import matplotlib.pyplot as plt


@njit(fastmath=True)
def simulation_rapide(params):
    Kp, Ki, Kd = params
    
    K_g = 2.3
    tau_g = 0.25      
    dt = 0.01
    t_max = 50.0      
    n_steps = int(t_max / dt)
    
    consigne_max = 12.56
    t_ramp = 0.2     
    
    # Kalman Init
    x_hat = 0.0
    P = 1.0
    Q = 0.001
    R = 0.01
    
    # État initial
    reponse = 0.0
    integrale = 0.0
    erreur_prec = 0.0
    x_pos = 0.0
    consigne = 0.0
    
    # Score 
    score_itae = 0.0
    max_depassement = 0.0
    sum_oscillation = 0.0
    
    # Boucle temporelle 
    for i in range(n_steps):
        t = i * dt
        
        # Scénario de Consigne
        if t < t_ramp:
            consigne = consigne_max * (t / t_ramp)
        else:
            modulo_x = x_pos % (4.0 + np.pi)
            if modulo_x > 1.0:
                consigne = 8.11
            if modulo_x > (1.0 + 0.5 * np.pi):
                consigne = 12.56
            if modulo_x > (3.0 + 0.5 * np.pi):
                consigne = 8.11
            if modulo_x > (3.0 + np.pi):
                consigne = 12.56

        perturbations = 0.2 * np.random.normal(0.0, 1.0) + \
               (-5.0 if 1.25 + 0.5*np.pi < x_pos % (4.0 + np.pi) < 1.75 + 0.5*np.pi else 0.0) + \
               (5.0 if 2.25 + 0.5*np.pi < x_pos % (4.0 + np.pi) < 2.75 + 0.5*np.pi else 0.0) + \
               0.1 * np.sin(2.0 * np.pi * 0.3 * t)
        
        mesure = reponse + np.random.normal(0.0, np.sqrt(R))
        
        # PID
        erreur = consigne - x_hat
        integrale += erreur * dt
        
        if integrale > 10.0: integrale = 10.0
        elif integrale < -10.0: integrale = -10.0
            
        derivee = (erreur - erreur_prec) / dt
        erreur_prec = erreur
        
        commande = Kp * erreur + Ki * integrale + Kd * derivee
        if commande > 20.0: commande = 20.0
        elif commande < -20.0: commande = -20.0

        # Kalman 1d
        # Prédiction
        x_hat = x_hat + (K_g * commande + perturbations - x_hat) * (dt / tau_g)
        P = P + Q
        # Correction
        K_gain = P / (P + R)
        x_hat = x_hat + K_gain * (mesure - x_hat)
        P = (1.0 - K_gain) * P
            
        # RK4
        k1 = (K_g * commande + perturbations - reponse) / tau_g
        k2 = (K_g * commande + perturbations - (reponse + 0.5*dt*k1)) / tau_g
        k3 = (K_g * commande + perturbations - (reponse + 0.5*dt*k2)) / tau_g
        k4 = (K_g * commande + perturbations - (reponse + dt*k3)) / tau_g
        
        
        reponse += (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4)
        
        # Calcul Position et Score
        x_pos += 0.03 * reponse * dt
        
        err_abs = abs(consigne - reponse)
        score_itae += (t * err_abs) * dt
        
        # Pénalité dépassement
        depassement = reponse - consigne
        if depassement > max_depassement: 
            max_depassement = depassement
        
        # Pénalité oscillation
        if t > 40.0: 
            sum_oscillation += abs(depassement)

    # Score Final
    score_total = score_itae
    if max_depassement > 2.0: 
        score_total += 500.0 * max_depassement
    score_total += 10.0 * sum_oscillation 
    
    return score_total


def lancer_optimisation():
    print("Compilation du code Numba")
    
    print("Lancement de l'Évolution Différentielle")
    c = time.time()
    
    # Bornes de recherche : [(Kp_min, Kp_max), (Ki_min, Ki_max), (Kd_min, Kd_max)]
    bounds = [(0, 100), (0, 200), (0, 2)]
    
    # L'algorithme
    resultat = differential_evolution(
        simulation_rapide, 
        bounds, 
        strategy='best1bin', 
        maxiter=5000,      # Nombre de générations
        popsize=150,      # Taille population (15 * 3 paramètres = 45 individus)
        tol=0.01,        # Arrêt quand on ne trouve plus mieux
        mutation=(0.1, 1), 
        recombination=0.2,
        workers=-1,      # UTILISE TOUS LES COEURS DU CPU (-1)
        disp=True        # Affiche la progression
    )
    
    print("-" * 50)
    print(f"OPTIMISATION TERMINÉE en {time.time() - c:.2f} secondes")
    print("-" * 50)
    print(f"Meilleur Score : {resultat.fun:.4f}")
    print(f"Meilleurs Gains : {resultat.x[0]:.4f},{resultat.x[1]:.4f},{resultat.x[2]:.6f}")
    
    return resultat.x

if __name__ == "__main__":
    meilleurs_gains = lancer_optimisation()
    
    # Sauvegarde
    with open("meilleurs_pid_numba.txt", "a") as f:
        f.write(f"{meilleurs_gains[0]},\t{meilleurs_gains[1]},\t{meilleurs_gains[2]}\n")