import matplotlib.pyplot as plt

def lire_et_trier_scores(fichier_entree, fichier_sortie):
    # Initialisation d'une liste vide pour stocker les tuples (score, ligne)
    lignes = []

    # Lecture du fichier d'entrée
    with open(fichier_entree, "r") as f:
        # Lecture de toutes les lignes du fichier
        lignes_brutes = f.readlines()
    
    # Extraction des deux premières lignes comme en-tête
    en_tete = lignes_brutes[:1] #2 hors kalman
    # Les lignes suivantes contiennent les données
    lignes_donnees = lignes_brutes[1:]

    # Traiter chaque ligne de données
    for ligne in lignes_donnees:
        # Suppression des espaces en début et fin et séparation par espaces
        elements = ligne.strip().split()
        # Vérification que la ligne contient exactement 4 éléments
        if len(elements) == 4:
            score, kp, ki, kd = elements  
            try:
                # Conversion de l'élément score en float pour pouvoir trier
                score = float(score)
                # Ajout d'un tuple (score, ligne) à la liste
                lignes.append((score, ligne.strip()))
            except ValueError:
                # En cas d'erreur de conversion, on ignore simplement la ligne mal formatée
                continue

    # Tri de la liste de tuples en fonction du score (valeur float) en ordre croissant
    lignes_tries = sorted(lignes, key=lambda x: x[0])

    # Ecriture des données triées dans le fichier de sortie
    with open(fichier_sortie, "w") as f:
        # Ecriture de l'en-tête dans le fichier de sortie
        f.writelines(en_tete)
        # Ecriture de chaque ligne triée avec un retour à la fin de ligne
        for _, ligne in lignes_tries:
            f.write(ligne + "\n")

    # Affichage d'un message indiquant la réussite de l'opération
    print(f"Scores triés par ordre croissant écrits dans {fichier_sortie}")

    

# Utilisation de la fonction avec les chemins d'accès aux fichiers
lire_et_trier_scores("C:/Users/ferna/OneDrive/Bureau/TIPE/simulation/V4_test_programme/scores_kalman.txt",
                      "C:/Users/ferna/OneDrive/Bureau/TIPE/simulation/V4_test_programme/scores_kalman.txt")

# lire_et_trier_scores("C:/Users/ferna/OneDrive/Bureau/TIPE/simulation/V2/scores.txt",
#                       "C:/Users/ferna/OneDrive/Bureau/TIPE/simulation/V2/scores.txt")