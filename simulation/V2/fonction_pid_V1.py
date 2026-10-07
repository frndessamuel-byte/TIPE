import numpy as np

# Configuration moteur
K_g, tau_g = 2.3, 0.25 # indentifié pour le robot
K_d, tau_d = 2.3, 0.25
dt = 0.01     # pas de temps
t_max = 50   # durée de la simulation 
t = np.arange(0, t_max, dt)

def calcul_réponse(reponse, K, consigne, integrale, erreur_precedente, Kp, Ki, Kd, perturbations, tau):
        erreur= consigne - (reponse + np.random.normal(0, 0.1)) # ajout de bruit de mesure
        integrale += erreur * dt
        integrale = np.clip(integrale, -10, 10)
        derivee = (erreur - erreur_precedente) / dt
        erreur_precedente= erreur
        commande = Kp * erreur + Ki * integrale + Kd * derivee
        commande = np.clip(commande, -20, 20)
        return reponse + (K * commande + perturbations - reponse) * (dt / tau), integrale, erreur_precedente

def simuler_pid(Kp_g, Ki_g, Kd_g, Kp_d, Ki_d, Kd_d):
    consigne_base=12.56
    reponse_g, reponse_d, x = 0, 0, 0
    integrale_g, integrale_d = 0, 0
    erreur_precedente_g, erreur_precedente_d = 0, 0
    consigne_g = consigne_d = consigne_base
    sortie_r, Cr = [], []
    
    for e in t:
        reponse_g, integrale_g, erreur_precedente_g = calcul_réponse(reponse_g, K_g, consigne_g, integrale_g,
                                                                     erreur_precedente_g, Kp_g, Ki_g, Kd_g, 
                                                                     perturbations, tau_g)
        reponse_d, integrale_d, erreur_precedente_d = calcul_réponse(reponse_d, K_d, consigne_d, integrale_d, 
                                                                     erreur_precedente_d, Kp_d, Ki_d, Kd_d, 
                                                                     perturbations, tau_d)

        if x % (4 + np.pi) > 1:
            consigne_g = 8.11
        if x % (4 + np.pi) > (1 + 0.5 * np.pi):
            consigne_g = 12.56
        if x % (4 + np.pi) > (3 + 0.5 * np.pi):
            consigne_g = 8.11
        if x % (4 + np.pi) > (3 + np.pi):
            consigne_g = 12.56

        perturbations = (
            0.2 * np.random.normal() +
            (-2 if 1.25 + 0.5 * np.pi < x % (4 + np.pi) < 1.75 + 0.5 * np.pi else 0) +
            (2 if 2.25 + 0.5 * np.pi < x % (4 + np.pi) < 2.75 + 0.5 * np.pi else 0) +
            0.1 * np.sin(2 * np.pi * 0.3 * e)
        )

        Cr.append(0.5 * (consigne_g + consigne_d))
        vr = 0.03 * (reponse_g + reponse_d) / 2 # 0.03 rayon de la roue
        x += vr * dt
        sortie_r.append((reponse_g + reponse_d) / 2)

    sortie_r = np.array(sortie_r)
    Cr = np.array(Cr)
    erreur_r = Cr - sortie_r

    itae = np.trapezoid(t * np.abs(erreur_r), t)
    return itae, sortie_r, Cr

# Kp_g = 8.024 
# Ki_g = 2.202
# Kd_g = 0.081