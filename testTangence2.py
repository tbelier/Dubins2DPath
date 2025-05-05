import matplotlib.pyplot as plt
import numpy as np

class Cercle:
    def __init__(self,cx,cy,r):
        self.x,self.y,self.r = cx,cy,r
        self.c = (self.x,self.y)

    def XY(self,cercle_other):
        x,y,r = self.x,self.y,self.r
        theta12 = theta(self,cercle_other)

        xt1,yt1 = x + r*np.cos(theta12+np.pi/2), y + r*np.sin(theta12+np.pi/2)
        xt2,yt2 = x + r*np.cos(theta12-np.pi/2), y + r*np.sin(theta12-np.pi/2)

        X = np.array([xt1,xt2])
        Y = np.array([yt1,yt2])
        return X,Y
    
    def display(self,ax):
        ax.add_patch(plt.Circle(self.c, self.r, color='blue', fill=False, linestyle='--'))
    
    def display_tangent_points(self,other_circle,ax):
        X,Y = self.XY(other_circle)
        for k in range(len(X)):
            if k == 0: ax.plot(X[k],Y[k], 'rx',)
            elif k==1: ax.plot(X[k],Y[k], 'gx',)

def draw_lines(cercle1,cercle2,ax):
    X1,Y1 = cercle1.XY(cercle2)
    X2,Y2 = cercle2.XY(cercle1)

    ax.plot([X1[0],X2[0]],[Y1[0],Y2[0]],"r")
    ax.plot([X1[1],X2[1]],[Y1[1],Y2[1]],"g")

def theta(cercle1,cercle2):
    if cercle2.x < cercle1.x : cercle1,cercle2 = cercle2,cercle1
    x1,y1 = cercle1.x,cercle1.y
    x2,y2 = cercle2.x,cercle2.y
    return np.arctan2(y2-y1,x2-x1)

# Définir les centres et rayons des cercles
cercle1 = Cercle(2,10,1.5)
cercle2 = Cercle(6,5,2)

# Création de la figure
fig, ax = plt.subplots()

# Création des cercles
cercle1.display(ax)
cercle2.display(ax)

# Tracer les centres
cercle1.display_tangent_points(cercle2,ax)
cercle2.display_tangent_points(cercle1,ax)
draw_lines(cercle1,cercle2,ax)
# Mise à l’échelle et légende
ax.set_aspect('equal')
ax.grid(True)
ax.legend()
plt.title("Affichage de 2 cercles, leurs centres et 2 points")
plt.xlabel("X")
plt.ylabel("Y")
plt.show()
