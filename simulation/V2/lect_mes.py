from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np

def lire_donnees(nom_fichier):
    temps = []
    sortie_g = [] # Réponse du moteur gauche
    sortie_d = [] # Réponse du moteur droit
    # Ouvre le fichier en mode lecture
    with open(nom_fichier, 'r') as fichier:
        # Parcours chaque ligne du fichier
        for ligne in fichier:
            # Supprime espaces en début/fin et remplace les virgules par rien
            ligne_nettoyee = ligne.strip().replace(',', '')
            # Passe à la ligne suivante si la ligne est vide
            if not ligne_nettoyee:
                continue
            # Sépare la ligne en éléments individuels
            parties = ligne_nettoyee.split()
            # Continue seulement si au moins trois éléments (temps, moteur gauche, moteur droit) sont présents
            if len(parties) < 3:
                continue
            # Extrait la chaîne de temps et les valeurs des moteurs gauche et droit
            temps_str, g, d = parties[:3]
            try:
                # Convertit la chaîne de temps en un objet datetime
                t = datetime.strptime(temps_str, "%H:%M:%S.%f")
                # Ajoute le temps converti à la liste des temps
                temps.append(t)
                # Convertit et ajoute la valeur du moteur gauche
                sortie_g.append(float(g))
                # Convertit et ajoute la valeur du moteur droit
                sortie_d.append(float(d))
            except Exception:
                # En cas d'erreur lors de la conversion, ignore la ligne
                continue
    # Retourne les listes de temps et des sorties des moteurs gauche et droit
    return temps, sortie_g, sortie_d

def main():
    nom_fichier = "C:/Users/ferna/OneDrive/Bureau/TIPE/simulation/V4_test_programme/mes_ouv.txt"
    temps, sortie_g, sortie_d = lire_donnees(nom_fichier)
    if not temps:
        print("Aucune donnée valide trouvée.")
        return

    # Conversion en secondes depuis le premier relevé
    temps_debut = temps[0]
    temps_sec = [(t - temps_debut).total_seconds() for t in temps]

    # Calcul de la vitesse angulaire du robot (moyenne des deux moteurs)
    sortie_r = [(g + d) / 2 for g, d in zip(sortie_g, sortie_d)]
    sortie_r = np.array(sortie_r)

    plt.figure(figsize=(20, 3))
    plt.suptitle("Figure 21 : Boucle fermée - Seconde réponse réelle du robot avec régulation PID (4.902, 19.872, 0.019)", fontsize=16)

    # plt.subplot(2, 2, 1)
    # plt.plot(temps_sec, sortie_g)
    # plt.title("Moteur gauche", loc='center')
    # plt.axhline(y=12.56, color='r', linestyle='--')
    # plt.xlabel("Temps (s)")
    # plt.ylabel("Vitesse angulaire (rad/s)")
    # plt.grid(True)
    
    # plt.subplot(2, 2, 2)
    # plt.plot(temps_sec, sortie_d)
    # plt.title("Moteur droit", loc='center')
    # plt.axhline(y=12.56, color='r', linestyle='--')
    # plt.xlabel("Temps (s)")
    # plt.ylabel("Vitesse angulaire (rad/s)")
    # plt.grid(True)
    
    plt.subplot(2, 1, 2)
    plt.plot(temps_sec, sortie_r*0.03)
    plt.axhline(y=12.56*0.03, color='r', linestyle='--')
    plt.title("Robot", loc='center')
    plt.xlabel("Temps (s)")
    plt.ylabel("Vitesse du robot (m/s)")
    plt.grid(True)
    plt.show()
    
if __name__ == '__main__':
    main()
