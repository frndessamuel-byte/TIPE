import torch 
import time
import numpy as np

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

K_g = torch.tensor(2.3, device=device)
tau_g = torch.tensor(0.25, device=device)
K_d = torch.tensor(2.3, device=device)
tau_d = torch.tensor(0.25, device=device)
dt = torch.tensor(0.01, device=device)
t_max = 50.0
t = torch.arange(0, t_max, dt.item(), device=device)

c=time.time()

def calcul_réponse(reponse, K, consigne, integrale, erreur_precedente, Kp, Ki, Kd, perturbations, tau):
        erreur= consigne - (reponse + torch.randn(1, device=device) * 0.1 )# ajout de bruit de mesure
        integrale += erreur * dt
        integrale = torch.clamp(integrale, -10, 10)
        derivee = (erreur - erreur_precedente) / dt
        erreur_precedente= erreur
        commande = Kp * erreur + Ki * integrale + Kd * derivee
        commande = torch.clamp(commande, -20, 20)
        return reponse + (K * commande + perturbations - reponse) * (dt / tau), integrale, erreur_precedente

def simuler_pid(Kp_g, Ki_g, Kd_g, Kp_d, Ki_d, Kd_d, device="cuda"):
    try:
        # Constantes du système
        consigne_base = 18.85
        reponse_g, reponse_d, x = torch.zeros(1, device=device), torch.zeros(1, device=device), torch.zeros(1, device=device)
        integrale_g, integrale_d = torch.zeros(1, device=device), torch.zeros(1, device=device)
        erreur_precedente_g, erreur_precedente_d = torch.zeros(1, device=device), torch.zeros(1, device=device)
        consigne_g = consigne_d = torch.full((1,), consigne_base, device=device)
        sortie_r, Cr = [], []

        for e in t:
            if x % (4 + np.pi) > 1:
                consigne_g = 8.11
            if x % (4 + np.pi) > (1 + 0.5 * np.pi):
                consigne_g = 12.56
            if x % (4 + np.pi) > (3 + 0.5 * np.pi):
                consigne_g = 8.11
            if x % (4 + np.pi) > (3 + np.pi):
                consigne_g = 12.56
        
            Cr.append(0.5 * (consigne_g + consigne_d))


            perturbations = (
                0.2 * torch.randn(1, device=device) +
                (-2.0 if 1.25 + 0.5 * torch.pi < x % (4 + torch.pi) < 1.75 + 0.5 * torch.pi else 0.0) +
                (2.0 if 2.25 + 0.5 * torch.pi < x % (4 + torch.pi) < 2.75 + 0.5 * torch.pi else 0.0) +
                0.1 * torch.sin(2 * torch.pi * 0.3 * e)
            )

            reponse_g, integrale_g, erreur_precedente_g = calcul_réponse(reponse_g, K_g, consigne_g, integrale_g, erreur_precedente_g, Kp_g, Ki_g, Kd_g, perturbations, tau_g)
            reponse_d, integrale_d, erreur_precedente_d = calcul_réponse(reponse_d, K_d, consigne_d, integrale_d, erreur_precedente_d, Kp_d, Ki_d, Kd_d, perturbations, tau_d)

            vr = 0.03 * (reponse_g + reponse_d) / 2
            x += vr * dt
            sortie_r.append((reponse_g + reponse_d) / 2)
        
        
        sortie_r = torch.cat(sortie_r, dim=0)
        Cr = torch.cat(Cr, dim=0)

        erreur_r = Cr - sortie_r
        itae = torch.trapz(t * torch.abs(erreur_r), t)

        return itae.item(), sortie_r.cpu().numpy(), Cr.cpu().numpy()

    except Exception as e:
        print("❌ Erreur dans la simulation PID :", e)
        return None, None, None


if __name__ == "__main__":
    print("▶️ Lancement de la simulation PID...")

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"🖥️ Utilisation du device : {device}")

    # Crée les coefficients comme tensors compatibles GPU
    Kp = torch.tensor(9.0, device=device)
    Ki = torch.tensor(2.0, device=device)
    Kd = torch.tensor(0.04, device=device)

    itae, sortie, Cr = simuler_pid(Kp, Ki, Kd, Kp, Ki, Kd, device=device)

    if itae is not None:
        print(f"✅ ITAE : {itae:.4f}")
    else:
        print("⚠️ Simulation échouée.")

# print(simuler_pid(
#     torch.tensor(9.0, device="cuda"),
#     torch.tensor(2.0, device="cuda"),
#     torch.tensor(0.04, device="cuda"),
#     torch.tensor(9.0, device="cuda"),
#     torch.tensor(2.0, device="cuda"),
#     torch.tensor(0.04, device="cuda"))) 
