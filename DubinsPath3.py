import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import interp1d
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
        self.pointsH = np.array([])
        self.pointsV = np.array([])
        self.points3D = np.array([])


def matchXYZ(Lxy, Lxz):
    x1 = Lxy[:, 0]
    y = Lxy[:, 1]
    psi = Lxy[:, 2]
    
    x2 = Lxz[:, 0]
    z = Lxz[:, 1]
    theta = Lxz[:, 2]

    # Abscisses curvilignes cumulées
    ds1 = np.sqrt(np.diff(x1)**2 + np.diff(y)**2)
    s1 = np.hstack(([0], np.cumsum(ds1)))
    s1 = s1 / s1[-1] * 100  # normalisation à [0,100]

    ds2 = np.sqrt(np.diff(x2)**2 + np.diff(z)**2)
    s2 = np.hstack(([0], np.cumsum(ds2)))
    s2 = s2 / s2[-1] * 100  # normalisation à [0,100]

    # Intervalle commun (normalisé)
    n_points = 5000
    s_common = np.linspace(0,100, n_points)
    print(s_common)

    # Interpolateurs
    interp_x = interp1d(s1, x1, kind='linear', fill_value='extrapolate')
    interp_y = interp1d(s1, y, kind='linear', fill_value='extrapolate')
    interp_psi = interp1d(s1, psi, kind='linear', fill_value='extrapolate')

    interp_x2 = interp1d(s2, x2, kind='linear', fill_value='extrapolate')
    interp_z = interp1d(s2, z, kind='linear', fill_value='extrapolate')
    interp_theta = interp1d(s2, theta, kind='linear', fill_value='extrapolate')

    # Interpolation
    x_common1 = interp_x(s_common)
    x_common2 = interp_x2(s_common)
    x_common = (x_common1 + x_common2) / 2  

    y_common = interp_y(s_common)
    z_common = interp_z(s_common)
    psi_common = interp_psi(s_common)
    theta_common = interp_theta(s_common)

    # Reconstruction
    Lxyz = np.column_stack((x_common, y_common, z_common, theta_common, psi_common))
    return Lxyz



# Mise à jour du graphique des sliders
def update(val):
    # Récupère les valeurs des sliders pour la position initiale, finale, et le rayon
    x0, y0, z0, theta0, psi0 = display.slider_x0.val, display.slider_y0.val, display.slider_z0.val, display.slider_theta0.val, display.slider_psi0.val
    x1, y1, z1, theta1, psi1 = display.slider_x1.val, display.slider_y1.val, display.slider_z1.val, display.slider_theta1.val, display.slider_psi1.val
    radiusH = display.slider_radiusH.val
    radiusV = display.slider_radiusV.val
    
    # Calculer le chemin Dubins en fonction des sliders
    solutionsH = dubin_path.all_dubins_paths([x0, y0, psi0], [x1, y1, psi1], radius=radiusH)
    solutionsV = dubin_path.all_dubins_paths([x0, z0, theta0], [x1, z1, theta1], radius=radiusV)

    
    ax.clear()
    colors = ["red", "orange", "yellow", "green", "blue", "violet"]
    for k in range(len(solutionsH)):
        
        solutionH = solutionsH[k]
        modeH = solutionH[0]
        save.pointsH = np.array(dubin_path.get_projection([x0, y0, psi0], [x1, y1, psi1], solutionH))
    
        # Mettre à jour l'affichage du chemin Dubins
        L0H = [0.0 for k in range(len(save.pointsH[:,1]))]
        if k == 0 : 
            bestH = save.pointsH
            ax.plot(save.pointsH[:,0], save.pointsH[:,1], L0H, label=modeH, color="blue")
        else : 
            ax.plot(save.pointsH[:,0], save.pointsH[:,1], L0H, color=[0.0,0.0,1.0,0.2])

    
    for k in range(len(solutionsV)):
        solutionV = solutionsV[k]
        modeV = solutionV[0]
        save.pointsV = np.array(dubin_path.get_projection([x0, z0, theta0], [x1, z1, theta1], solutionV))
    
        # Mettre à jour l'affichage du chemin Dubins
        L0V = [0.0 for k in range(len(save.pointsV[:,1]))]
        if k == 0 : 
            bestV = save.pointsV
            ax.plot(save.pointsV[:,0], L0V, save.pointsV[:,1], label=modeV, color="green")
        else : 
            ax.plot(save.pointsV[:,0], L0V, save.pointsV[:,1], color=[0.0,1.0,0.0,0.2])
    
    save.points3D = matchXYZ(bestH,bestV)
    ax.plot(save.points3D[:,0], save.points3D[:,1], save.points3D[:,2], label="3D", color="red")

    ax.scatter(save.points3D[0,0], save.points3D[0,1], save.points3D[0,2], label="point init", color="green")
    ax.scatter(save.points3D[-1,0], save.points3D[-1,1], save.points3D[-1,2], label="point final", color="red")
    ax.set_box_aspect([1,1,1])

# Ajouter les sliders et labels
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    #ax.axis("equal")
    ax.legend()
    
    fig.canvas.draw_idle()

# Fonction pour enregistrer le chemin généré par le Dubins path
def save_path(event):
    file_name3D = file_dir + '/Missions/dubins_path3D.txt'
    points3Drounded = np.round(save.points3D, 2)
    np.savetxt( file_name3D, points3Drounded, delimiter=',', header="x, y, z, psi, theta", fmt="%.2f")
    print(f"Path saved to {file_name3D}")


class Display:
    def __init__(self,ax):
        # Création des sliders pour position initiale (x0, y0, psi0), position finale (x1, y1, psi1) et rayon
        ymax = 0.25
        delta_y = 0.03
        
        ax_x0 = plt.axes([0.1, ymax-0*delta_y, 0.35, 0.02])
        ax_y0 = plt.axes([0.1, ymax-1*delta_y, 0.35, 0.02])
        ax_z0 = plt.axes([0.1, ymax-2*delta_y, 0.35, 0.02])
        ax_theta0 = plt.axes([0.1, ymax-3*delta_y, 0.35, 0.02])
        ax_psi0 = plt.axes([0.1, ymax-4*delta_y, 0.35, 0.02])
        
        ax_x1 = plt.axes([0.55, ymax-0*delta_y, 0.35, 0.02])
        ax_y1 = plt.axes([0.55,ymax-1*delta_y, 0.35, 0.02])
        ax_z1 = plt.axes([0.55, ymax-2*delta_y, 0.35, 0.02])
        ax_theta1 = plt.axes([0.55, ymax-3*delta_y, 0.35, 0.02])
        ax_psi1 = plt.axes([0.55, ymax-4*delta_y, 0.35, 0.02])

        ax_radiusH = plt.axes([0.1, ymax-5*delta_y, 0.35, 0.02])
        ax_radiusV = plt.axes([0.55, ymax-5*delta_y, 0.35, 0.02])

        self.slider_x0 = Slider(ax_x0, "x0", -10.0, 10.0, valinit=x0_init)
        self.slider_y0 = Slider(ax_y0, "y0", -10.0, 10.0, valinit=y0_init)
        self.slider_z0 = Slider(ax_z0, "z0", -10.0, 10.0, valinit=z0_init)
        self.slider_theta0 = Slider(ax_theta0, "theta0", 0.0,2*np.pi, valinit=theta0_init)
        self.slider_psi0   = Slider(ax_psi0, "psi0", 0, 2*np.pi, valinit=psi0_init)

        self.slider_x1 = Slider(ax_x1, "x1", -10.0, 20.0, valinit=x1_init)
        self.slider_y1 = Slider(ax_y1, "y1", -10.0, 20.0, valinit=y1_init)
        self.slider_z1 = Slider(ax_z1, "z1", -10.0, 10.0, valinit=z1_init)
        self.slider_theta1 = Slider(ax_theta1, "theta1", 0.0,2*np.pi, valinit=theta1_init)
        self.slider_psi1 = Slider(ax_psi1, "psi1", 0, 2*np.pi, valinit=psi1_init)
        self.slider_radiusH = Slider(ax_radiusH, "rMinHori", 1.0, 10.0, valinit=radius_initH)
        self.slider_radiusV = Slider(ax_radiusV, "rMinVerti", 1.0, 10.0, valinit=radius_initV)

if __name__ == "__main__":
    
    # --------------------------------------------------------------------
    #                    Création de la figure et des axes
    # -------------------------------------------------------------------- 
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.view_init(elev=30., azim=45)
    #fig, ax = plt.subplots()
    
    
    plt.subplots_adjust(bottom=0.35)  # Ajusté pour éviter la superposition des sliders et bouton
    file_path = os.path.abspath(__file__)
    file_dir = os.path.dirname(file_path)
    # Initialisation des valeurs des sliders pour la position initiale, finale, et le rayon
    x0_init, y0_init, z0_init, theta0_init, psi0_init = 0.0,   0.0, 0.0, 0.0, 0.0
    x1_init, y1_init, z1_init, theta1_init, psi1_init = 10.0, 10.0, 10.0, 0.0, 0.0
    radius_initH = 5.0
    radius_initV = 5.0
    save = Save()

    display = Display(ax)

    # Ajout des callbacks pour les sliders
    display.slider_x0.on_changed(update)
    display.slider_y0.on_changed(update)
    display.slider_z0.on_changed(update)
    display.slider_theta0.on_changed(update)
    display.slider_psi0.on_changed(update)
    display.slider_x1.on_changed(update)
    display.slider_y1.on_changed(update)
    display.slider_z1.on_changed(update)
    display.slider_theta1.on_changed(update)
    display.slider_psi1.on_changed(update)
    display.slider_radiusH.on_changed(update)
    display.slider_radiusV.on_changed(update)

    # --------------------------------------------------------------------
    #                    Création du bouton "Save Path"
    # --------------------------------------------------------------------
    ax_save = plt.axes([0.8, 0.05, 0.1, 0.04])
    button_save = Button(ax_save, 'Save', color=[0.33,0.37,0.9], hovercolor=[0.5,0.5,0.9])
    button_save.on_clicked(save_path)

    # Initialiser et afficher le chemin Dubins pour les valeurs initiales des sliders
    update(None)

    # Affichage
    plt.show()
