import numpy as np
import matplotlib.pyplot as plt

class Cercle:
    def __init__(self,cx,cy,r):
        self.x,self.y,self.r = cx,cy,r
        self.c = (self.x,self.y)

    def find_p1p2(self,cercle_other):
        x,y,r = self.x,self.y,self.r
        theta12 = theta(self,cercle_other)

        xt1,yt1 = x + r*np.cos(theta12+np.pi/2), y + r*np.sin(theta12+np.pi/2)
        xt2,yt2 = x + r*np.cos(theta12-np.pi/2), y + r*np.sin(theta12-np.pi/2)

        p1 = [xt1,yt1]
        p2 = [xt2,yt2]
        return p1,p2
    
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

def generate_points_on_line(t_max, x0, y0, theta0, v, N):
    points = []
    for t in np.linspace(0, t_max, N):
        xt = x0 + v*np.cos(theta0) * t
        yt = y0 + v*np.sin(theta0) * t
        points.append([t, xt, yt])
    
    return np.array(points)

def distLin(p1,p2):
    x1,y1 = p1
    x2,y2 = p2

    return np.sqrt((x2-x1)**2+(y2-y1)**2)

def distRot(p1,p2,cercle):
    x1,y1 = p1
    x2,y2 = p2
    cx,cy,r = cercle.x,cercle.y,cercle.r

    dtheta1 = np.arctan2(y2-cy,x2-cx)
    dtheta2 = np.arctan2(y1-cy,x1-cx)

    return abs(dtheta2-dtheta1)*r

def choose_circles(X1,X2,rmin): #au lieu de mettre rmin ici on pourrait mettre r1 et r2, si le robot a un angle de braquement différent de chaque côté
    x1,y1,theta1 = X1.flatten()
    x2,y2,theta2 = X2.flatten()
    xm,ym = (x1+x2)/2,(y1+y2)/2
    cL1x, cL1y = x1+rmin*np.cos(theta1+np.pi/2), y1+rmin*np.sin(theta1+np.pi/2)
    cR1x, cR1y = x1+rmin*np.cos(theta1-np.pi/2), y1+rmin*np.sin(theta1-np.pi/2)
    cL2x, cL2y = x2+rmin*np.cos(theta2+np.pi/2), y2+rmin*np.sin(theta2+np.pi/2)
    cR2x, cR2y = x2+rmin*np.cos(theta2-np.pi/2), y2+rmin*np.sin(theta2-np.pi/2)

    circleLeft1  = Cercle(cL1x, cL1y,rmin)
    circleRight1 = Cercle(cR1x, cR1y,rmin)

    circleLeft2  = Cercle(cL2x, cL2y,rmin)
    circleRight2 = Cercle(cR2x, cR2y,rmin)

    if np.sqrt((cL1y-ym)**2+(cL1x-xm)**2) < np.sqrt((cR1y-ym)**2+(cR1x-xm)**2): circle1 = circleLeft1
    else : circle1 = circleRight1

    if np.sqrt((cL2y-ym)**2+(cL2x-xm)**2) < np.sqrt((cR2y-ym)**2+(cR2x-xm)**2): circle2 = circleLeft2
    else : circle2 = circleRight2

    return circle1, circle2
        
    



if __name__ == "__main__":
    #Conditions initiales
    N, t_max = 500, 50
    x0, y0, theta0 = 0,0,0 # Position initiale du prédateur
    xi, yi, thetai = 10, 10, np.pi/2  # Position initiale de la proie
    v,rmin = 0.5,2  # m/s
    v1,v2,v3 = v,v,v
    pts = generate_points_on_line(t_max, xi, yi, thetai, v, N)

    # Find p1,p2,pend
    pts_reachable = []
    for k,pt in enumerate(pts):
        tend,xend,yend = pt
        X0 = np.array([[x0],
                        [y0],
                        [theta0]])
        Xend = np.array([[xend],
                        [yend],
                        [thetai]])
        cercle1, cercle2 = choose_circles(X0,Xend,rmin)
        p1,p2 = cercle1.find_p1p2(cercle2)
        d1 = distRot([x0,y0],p1,cercle1)
        d2 = distLin(p1,p2)
        d3 = distRot(p1,[xend,yend],cercle2)
        t_trajet =  d1/v1+d2/v2+d3/v3
        if t_trajet < tend: pts_reachable.append([t_trajet,xend,yend])

    pts_reachable = np.array(pts_reachable)

    plt.figure()
    plt.quiver(x0,y0,np.cos(theta0), np.sin(theta0), color ="g")
    plt.quiver(xi,yi,np.cos(thetai), np.sin(thetai), color ="r")
    plt.plot(pts[:,1],pts[:,2], marker=".", color = (0.1, 0.1, 0.1, 0.03))
    scatter = plt.scatter(pts_reachable[:,1],pts_reachable[:,2], c=pts_reachable[:,0], cmap='viridis', marker='.')
    plt.colorbar(scatter, label="Distance parcourue (m)")
    plt.show()

