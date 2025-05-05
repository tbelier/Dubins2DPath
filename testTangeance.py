import numpy as np
import matplotlib.pyplot as plt

class Cercle:
    def __init__(self,cx,cy,r):
        self.x,self.y,self.r = cx,cy,r

def tangent_points_same_radius(cercle1,cercle2):
    """ Trouve les points de tangence des tangentes extérieures entre deux cercles de même rayon """
    d = np.sqrt((cercle2.x - cercle1.x)**2 + (cercle2.y - cercle1.y)**2)
    if d <= 2 * np.min([cercle1.r, cercle2.r]):
        return None  # Les cercles se chevauchent ou sont trop proches

    theta = np.arctan2(cercle2.y - cercle1.y, cercle2.x - cercle1.x)

    # Angles des points de tangence
    alpha = theta + np.pi / 2
    beta = theta - np.pi / 2

    # Points de tangence sur le premier cercle
    T1 = (cercle1.x +cercle1.r* np.cos(alpha), cercle1.y +cercle1.r* np.sin(alpha))
    T2 = (cercle1.x +cercle1.r* np.cos(beta), cercle1.y +cercle1.r* np.sin(beta))

    # Points de tangence sur le second cercle
    T3 = (cercle2.x + cercle2.r * np.cos(alpha), cercle2.y + cercle2.r * np.sin(alpha))
    T4 = (cercle2.x + cercle2.r * np.cos(beta), cercle2.y + cercle2.r * np.sin(beta))

    return T1, T2, T3, T4

def plot_circles_and_tangents(cercle1,cercle2):
    """ Affiche les cercles et les tangentes extérieures entre eux """
    fig, ax = plt.subplots()
    
    # Tracer les cercles
    circle1 = plt.Circle((cercle1.x, cercle1.y), cercle1.r, color='b', fill=False, linestyle="dashed")
    circle2 = plt.Circle((cercle2.x, cercle2.y), cercle2.r, color='r', fill=False, linestyle="dashed")
    
    ax.add_patch(circle1)
    ax.add_patch(circle2)

    # Calcul des tangentes
    tangents = tangent_points_same_radius(cercle1,cercle2)
    if tangents:
        T1, T2, T3, T4 = tangents

        # Tracer les points de tangence
        ax.scatter(*T1, color="blue", marker="o", label="Tangence 1")
        ax.scatter(*T2, color="blue", marker="o")
        ax.scatter(*T3, color="red", marker="o", label="Tangence 2")
        ax.scatter(*T4, color="red", marker="o")

        # Tracer les tangentes
        ax.plot([T1[0], T3[0]], [T1[1], T3[1]], 'g-', label="Tangente 1")
        ax.plot([T2[0], T4[0]], [T2[1], T4[1]], 'g-', label="Tangente 2")

    # Paramètres d'affichage
    ax.set_xlim(min(cercle1.x, cercle2.x) -cercle2.r- 1, max(cercle1.x, cercle2.x) +cercle2.r+ 1)
    ax.set_ylim(min(cercle1.y, cercle2.y) -cercle2.r- 1, max(cercle1.y, cercle2.y) +cercle2.r+ 1)
    ax.set_aspect('equal')
    ax.legend()
    ax.set_title("Tangentes entre deux cercles de même rayon")
    plt.grid()
    plt.show()

# Exemple d'utilisation
cercle1 = Cercle(0,0,2)
cercle2 = Cercle(5,2,4)
plot_circles_and_tangents(cercle1,cercle2)
