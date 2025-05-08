import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from easydubins import dubin_path
import os

dubin_path.MAX_LINE_DISTANCE = 0.1  # Ex. 10 cm entre chaque point sur ligne droite
dubin_path.MAX_CURVE_DISTANCE = 0.1 # Ex. 10 cm sur les arcs
dubin_path.MAX_CURVE_ANGLE = 0.1    # Ex. 0.1 radian (~5.7°) entre deux points d'un arc

# Style sombre
#plt.style.use('dark_background')
class Save():
    def __init__(self):
        self.points = np.array([])

# Mise à jour du graphique des sliders
def update(val):
    # Récupère les valeurs des sliders pour la position initiale, finale, et le rayon
    x0, y0, theta0 = slider_x0.val, slider_y0.val, slider_theta0.val
    x1, y1, theta1 = slider_x1.val, slider_y1.val, slider_theta1.val
    radius = slider_radius.val
    
    # Calculer le chemin Dubins en fonction des sliders
    solutions = dubin_path.all_dubins_paths([x0, y0, theta0], [x1, y1, theta1], radius=radius)
    ax.clear()
    colors = ["red", "orange", "yellow", "green", "blue", "violet"]
    for k in range(len(solutions)):
        solution = solutions[k]
        mode = solution[0]
        #print(mode)
        save.points = np.array(dubin_path.get_projection([x0, y0, theta0], [x1, y1, theta1], solution))
    
        # Mettre à jour l'affichage du chemin Dubins
        if k == 0 : ax.plot(save.points[:,0], save.points[:,1], label=mode, color="red")
        else : ax.plot(save.points[:,0], save.points[:,1], label=mode, color=[0.0,0.0,1.0,0.2])
        
        ax.quiver(save.points[0,0],save.points[0,1],np.sin(np.deg2rad(save.points[0,2])),np.cos(np.deg2rad(save.points[0,2])), color="green")
        ax.quiver(save.points[-1,0],save.points[-1,1],np.sin(np.deg2rad(save.points[-1,2])),np.cos(np.deg2rad(save.points[-1,2])), color="red")
        ax.add_patch(plt.Circle((x0+radius*np.cos(np.pi/2+theta0), y0+radius*np.sin(np.pi/2+theta0)), radius, fill=False, linestyle = "--", edgecolor=[0.5,0.5,0.5,0.]))
        ax.add_patch(plt.Circle((x0+radius*np.cos(-np.pi/2+theta0), y0+radius*np.sin(-np.pi/2+theta0)), radius, fill=False, linestyle = "--", edgecolor=[0.5,0.5,0.5,0.]))
        ax.add_patch(plt.Circle((x1+radius*np.cos(np.pi/2+theta1), y1+radius*np.sin(np.pi/2+theta1)), radius, fill=False, linestyle = "--", edgecolor=[0.5,0.5,0.5,0.]))
        ax.add_patch(plt.Circle((x1+radius*np.cos(-np.pi/2+theta1), y1+radius*np.sin(-np.pi/2+theta1)), radius, fill=False, linestyle = "--", edgecolor=[0.5,0.5,0.5,0.]))

# Ajouter les sliders et labels
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.axis("equal")
    ax.legend()
    
    fig.canvas.draw_idle()

# Fonction pour enregistrer le chemin généré par le Dubins path
def save_path(event):
    file_name = file_dir +'/Missions/dubins_path.txt'
    np.savetxt(file_name, save.points, delimiter=',', header="x, y, theta", comments='')
    print(f"Path saved to {file_name}")

if __name__ == "__main__":
    
    # --------------------------------------------------------------------
    #                    Création de la figure et des axes
    # -------------------------------------------------------------------- 
    fig, ax = plt.subplots()
    
    
    plt.subplots_adjust(bottom=0.45)  # Ajusté pour éviter la superposition des sliders et bouton
    file_path = os.path.abspath(__file__)
    file_dir = os.path.dirname(file_path)
    # Initialisation des valeurs des sliders pour la position initiale, finale, et le rayon
    x0_init, y0_init, theta0_init = 0.0, 0.0, 0.0
    x1_init, y1_init, theta1_init = 10.0, 10.0, 1.57
    radius_init = 5.0
    save = Save()
    # Création des sliders pour position initiale (x0, y0, theta0), position finale (x1, y1, theta1) et rayon
    ax_x0 = plt.axes([0.1, 0.35, 0.35, 0.03])
    ax_y0 = plt.axes([0.1, 0.3, 0.35, 0.03])
    ax_theta0 = plt.axes([0.1, 0.25, 0.35, 0.03])
    ax_x1 = plt.axes([0.55, 0.35, 0.35, 0.03])
    ax_y1 = plt.axes([0.55, 0.3, 0.35, 0.03])
    ax_theta1 = plt.axes([0.55, 0.25, 0.35, 0.03])
    ax_radius = plt.axes([0.1, 0.2, 0.8, 0.03])

    slider_x0 = Slider(ax_x0, "x0", -10.0, 10.0, valinit=x0_init)
    slider_y0 = Slider(ax_y0, "y0", -10.0, 10.0, valinit=y0_init)
    slider_theta0 = Slider(ax_theta0, "theta0", 0, 2*np.pi, valinit=theta0_init)
    slider_x1 = Slider(ax_x1, "x1", -10.0, 20.0, valinit=x1_init)
    slider_y1 = Slider(ax_y1, "y1", -10.0, 20.0, valinit=y1_init)
    slider_theta1 = Slider(ax_theta1, "theta1", 0, 2*np.pi, valinit=theta1_init)
    slider_radius = Slider(ax_radius, "Rayon de giration", 1.0, 10.0, valinit=radius_init)

    # Ajout des callbacks pour les sliders
    slider_x0.on_changed(update)
    slider_y0.on_changed(update)
    slider_theta0.on_changed(update)
    slider_x1.on_changed(update)
    slider_y1.on_changed(update)
    slider_theta1.on_changed(update)
    slider_radius.on_changed(update)

    # --------------------------------------------------------------------
    #                    Création du bouton "Save Path"
    # --------------------------------------------------------------------
    ax_save = plt.axes([0.8, 0.05, 0.1, 0.04])
    button_save = Button(ax_save, 'Save Path', color='gray', hovercolor='lightgray')
    button_save.on_clicked(save_path)

    # Initialiser et afficher le chemin Dubins pour les valeurs initiales des sliders
    update(None)

    # Affichage
    plt.show()
