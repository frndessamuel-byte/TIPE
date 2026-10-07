import numpy as np

# ==========================================================
# FONCTION DE SIMULATION (PID + KALMAN + RK4)
# ==========================================================
def simuler_systeme_complet(Kp, Ki, Kd):

    
    # --- 1. Paramètres Système ---
    K_g, tau_g = 2.3, 0.15
    dt = 0.01
    t_max = 20 # On peut réduire le temps pour l'optimisation (gagner du temps de calcul)
    
    # Consignes
    consigne_base = 12.56
    t_ramp = 1.0
    
    # Paramètres Kalman
    Q = 0.001 
    R = 0.01
    x_hat, P = 0.0, 1.0

    # Initialisation
    t = np.arange(0, t_max, dt)
    reponse = 0.0
    integrale = 0.0
    erreur_precedente = 0.0
    x_pos = 0.0
    
    # Listes pour résultats
    sortie_reelle = [] # Ce que fait le moteur physiquement
    sortie_filtree = [] # Ce que voit le PID (après Kalman)
    consignes = []
    
    # --- 2. Fonctions internes (Optimisées pour la vitesse) ---
    # On définit les perturbations ici pour éviter les appels globaux
    def get_perturbation(x_p, t_p):
        return (0.2 * np.random.normal() + 
                (-10 if 1.25 + 0.5*np.pi < x_p % (4 + np.pi) < 1.75 + 0.5*np.pi else 0) +
                (2 if 2.25 + 0.5 * np.pi < x_p % (4 + np.pi) < 2.75 + 0.5 * np.pi else 0) +
                0.1 * np.sin(2 * np.pi * 0.3 * t_p))

    def f_reponse(rep, K, cmd, pert, tau):
        return (K * cmd + pert - rep) / tau

    # --- 3. Boucle de Simulation ---
    for e in t:
        # Gestion consigne (Rampe + Cycle simplifié pour optimisation)
        if e < t_ramp:
            consigne = consigne_base * (e / t_ramp)
        else:
            # On garde une consigne simple pour l'optimisation pour bien voir la stabilité
            consigne = consigne_base 
            # On peut ajouter un échelon inverse à t_max/2 pour tester la réactivité
            if e > t_max/2: consigne = 8.0 

        consignes.append(consigne)

        # Mesure bruitée
        mesure = reponse + np.random.normal(0, np.sqrt(R))
        
        # Filtre de Kalman
        # Prédiction
        x_hat = x_hat + (K_g * (Kp*(consigne-x_hat)) + get_perturbation(x_pos, e) - x_hat) * (dt / tau_g) # Approximation commande précédente
        P = P + Q
        # Correction
        K_gain = P / (P + R)
        x_hat = x_hat + K_gain * (mesure - x_hat)
        P = (1 - K_gain) * P
        
        sortie_filtree.append(x_hat)

        # PID (Basé sur x_hat filtré)
        erreur = consigne - x_hat
        integrale += erreur * dt
        integrale = np.clip(integrale, -10, 10) # Anti-windup
        
        derivee = (erreur - erreur_precedente) / dt
        erreur_precedente = erreur
        
        commande = Kp * erreur + Ki * integrale + Kd * derivee
        commande = np.clip(commande, -20, 20)

        # Physique (RK4)
        k1 = f_reponse(reponse, K_g, commande, get_perturbation(x_pos, e), tau_g)
        k2 = f_reponse(reponse + 0.5*dt*k1, K_g, commande, get_perturbation(x_pos, e), tau_g)
        k3 = f_reponse(reponse + 0.5*dt*k2, K_g, commande, get_perturbation(x_pos, e), tau_g)
        k4 = f_reponse(reponse + dt*k3, K_g, commande, get_perturbation(x_pos, e), tau_g)
        reponse += (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
        
        # Cinématique
        vr = 0.03 * reponse
        x_pos += vr * dt
        
        sortie_reelle.append(reponse)

    # --- 4. Calcul du Score (ITAE + Pénalités) ---
    consignes = np.array(consignes)
    sortie_reelle = np.array(sortie_reelle) # On juge sur la réalité, pas sur le filtre !
    erreur_finale = consignes - sortie_reelle
    
    # Critère ITAE (Integral Time Absolute Error)
    itae = np.sum(t * np.abs(erreur_finale)) * dt
    
    # Pénalités
    depassement = np.max(sortie_reelle) - np.max(consignes)
    oscillation = np.std(erreur_finale[-int(len(t)*0.2):]) # Stabilité sur la fin
    
    # Score composite
    score = itae 
    if depassement > 1.0: score += 1000 * depassement # Forte pénalité si dépassement excessif
    if oscillation > 0.5: score += 2000 * oscillation # Forte pénalité si instable
    
    return score, Kp, Ki, Kd