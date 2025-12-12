import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os

def create_optimized_gif():
    """
    Crée un GIF animé optimisé des trajectoires du mothership et du scout B
    """
    # Charger les données
    coordsA = np.load('testcoordsA.npy')  # Mothership
    coordsB = np.load('testcoordsB.npy')  # Scout B
    
    print(f"Données chargées:")
    print(f"- Mothership: {coordsA.shape[0]} points")
    print(f"- Scout B: {coordsB.shape[0]} points")
    
    # Réduire le nombre de points pour l'animation (1 point sur 5)
    step = 5
    coords_A_anim = coordsA[::step]
    coords_B_anim = coordsB[::step]
    
    print(f"Points pour l'animation: {len(coords_A_anim)} (réduit d'un facteur {step})")
    
    # Configuration de la figure avec une résolution plus faible
    plt.style.use('default')
    fig, ax = plt.subplots(figsize=(10, 8), dpi=80)
    
    # Définir les limites du plot
    all_coords = np.vstack([coordsA, coordsB])
    margin = 0.002
    ax.set_xlim(all_coords[:, 0].min() - margin, all_coords[:, 0].max() + margin)
    ax.set_ylim(all_coords[:, 1].min() - margin, all_coords[:, 1].max() + margin)
    
    # Labels et titre
    ax.set_xlabel('Longitude', fontsize=11)
    ax.set_ylabel('Latitude', fontsize=11)
    ax.set_title('Trajectoires Mothership et Scout B', fontsize=13, fontweight='bold')
    ax.grid(True, alpha=0.3)
    
    # Trajectoires complètes en arrière-plan
    ax.plot(coordsA[:, 0], coordsA[:, 1], 'lightblue', alpha=0.4, linewidth=1)
    ax.plot(coordsB[:, 0], coordsB[:, 1], 'lightcoral', alpha=0.4, linewidth=1)
    
    # Initialiser les lignes pour l'animation
    line_A, = ax.plot([], [], 'b-', linewidth=2.5, label='Mothership')
    line_B, = ax.plot([], [], 'r-', linewidth=2.5, label='Scout B')
    
    # Points pour les positions actuelles
    point_A, = ax.plot([], [], 'bo', markersize=6)
    point_B, = ax.plot([], [], 'ro', markersize=6)
    
    # Légende
    ax.legend(loc='upper right')
    
    def animate(frame):
        """Fonction d'animation optimisée"""
        if frame < len(coords_A_anim):
            # Mettre à jour les trajectoires
            line_A.set_data(coords_A_anim[:frame+1, 0], coords_A_anim[:frame+1, 1])
            line_B.set_data(coords_B_anim[:frame+1, 0], coords_B_anim[:frame+1, 1])
            
            # Mettre à jour les positions actuelles
            point_A.set_data([coords_A_anim[frame, 0]], [coords_A_anim[frame, 1]])
            point_B.set_data([coords_B_anim[frame, 0]], [coords_B_anim[frame, 1]])
            
        return line_A, line_B, point_A, point_B
    
    # Créer l'animation
    print("Création de l'animation...")
    anim = animation.FuncAnimation(fig, animate, frames=len(coords_A_anim), 
                                 interval=100, blit=True, repeat=True)
    
    # Sauvegarder en GIF avec des paramètres optimisés
    output_file = 'trajectoires_optimized.gif'
    print(f"Sauvegarde en cours vers {output_file}...")
    
    try:
        # Utiliser des paramètres optimisés pour un GIF plus léger
        writer = animation.PillowWriter(fps=10, metadata=dict(artist='Scout'))
        anim.save(output_file, writer=writer, savefig_kwargs={'bbox_inches': 'tight'})
        print(f"GIF sauvegardé avec succès: {output_file}")
    except Exception as e:
        print(f"Erreur lors de la sauvegarde: {e}")
        # Essayer avec imagemagick si disponible
        try:
            writer = animation.ImageMagickWriter(fps=10)
            anim.save(output_file, writer=writer)
            print(f"GIF sauvegardé avec ImageMagick: {output_file}")
        except:
            print("Impossible de sauvegarder le GIF. Affichage du plot seulement.")
    
    plt.tight_layout()
    return anim

def create_detailed_static_plot():
    """
    Crée un plot statique détaillé avec plus d'informations
    """
    # Charger les données
    coordsA = np.load('testcoordsA.npy')  # Mothership
    coordsB = np.load('testcoordsB.npy')  # Scout B
    
    # Calculer les distances pour l'analyse
    distances_A = np.sqrt(np.diff(coordsA[:, 0])**2 + np.diff(coordsA[:, 1])**2)
    distances_B = np.sqrt(np.diff(coordsB[:, 0])**2 + np.diff(coordsB[:, 1])**2)
    
    # Créer une figure avec subplots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
    
    # Plot principal des trajectoires
    ax1.plot(coordsA[:, 0], coordsA[:, 1], 'b-', linewidth=2, label='Mothership', alpha=0.8)
    ax1.plot(coordsB[:, 0], coordsB[:, 1], 'r-', linewidth=2, label='Scout B', alpha=0.8)
    
    # Marquer les points importants
    ax1.plot(coordsA[0, 0], coordsA[0, 1], 'go', markersize=10, label='Départ Mothership', markeredgecolor='black')
    ax1.plot(coordsA[-1, 0], coordsA[-1, 1], 'g^', markersize=10, label='Arrivée Mothership', markeredgecolor='black')
    ax1.plot(coordsB[0, 0], coordsB[0, 1], 'mo', markersize=10, label='Départ Scout B', markeredgecolor='black')
    ax1.plot(coordsB[-1, 0], coordsB[-1, 1], 'm^', markersize=10, label='Arrivée Scout B', markeredgecolor='black')
    
    # Configuration du premier plot
    ax1.set_xlabel('Longitude', fontsize=12)
    ax1.set_ylabel('Latitude', fontsize=12)
    ax1.set_title('Trajectoires complètes', fontsize=14, fontweight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot des distances entre points consécutifs
    ax2.plot(distances_A, 'b-', label='Mothership', alpha=0.7)
    ax2.plot(distances_B, 'r-', label='Scout B', alpha=0.7)
    ax2.set_xlabel('Point #', fontsize=12)
    ax2.set_ylabel('Distance entre points consécutifs', fontsize=12)
    ax2.set_title('Vitesse relative (distance entre points)', fontsize=14, fontweight='bold')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('trajectoires_analyse_complete.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    # Afficher quelques statistiques
    print(f"\n=== Statistiques des trajectoires ===")
    print(f"Mothership:")
    print(f"  - Distance totale: {np.sum(distances_A):.6f}")
    print(f"  - Distance moyenne entre points: {np.mean(distances_A):.8f}")
    print(f"  - Distance max entre points: {np.max(distances_A):.8f}")
    
    print(f"Scout B:")
    print(f"  - Distance totale: {np.sum(distances_B):.6f}")
    print(f"  - Distance moyenne entre points: {np.mean(distances_B):.8f}")
    print(f"  - Distance max entre points: {np.max(distances_B):.8f}")
    
    print(f"\nAnalyse sauvegardée: trajectoires_analyse_complete.png")

if __name__ == "__main__":
    print("=== Visualisation optimisée des trajectoires ===")
    
    # Créer le plot d'analyse détaillé
    print("\n1. Création de l'analyse détaillée...")
    create_detailed_static_plot()
    
    # Créer le GIF optimisé
    print("\n2. Création du GIF optimisé...")
    anim = create_optimized_gif()
    
    print("\n=== Terminé ===")