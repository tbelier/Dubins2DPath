import numpy as np
import matplotlib.pyplot as plt

def generate_points_on_line(t_max, x0, y0, v, a, b, dt=0.1):
    # Direction vector of the line y = ax + b
    # Normalize the direction vector (1, a) to unit length
    norm = np.sqrt(1**2 + a**2)
    
    points = []
    for t in np.arange(0, t_max + dt, dt):
        x = x0 + 1/norm * t
        y = y0 + a/norm * t
        points.append((x, y))
    
    return points

# Exemple d'utilisation
if __name__ == "__main__":
    t_max = 50
    x0, y0 = 0, 0
    v = 1  # m/s
    a = 0.5
    b = 0
    dt = 0.1  # pas de temps
    X,Y = [],[]

    pts = generate_points_on_line(t_max, x0, y0, v, a, b, dt)
    for i, (x, y) in enumerate(pts):
        X.append(x)
        Y.append(y)
        print(f" à t={i*dt:.1f}s -> x={x:.2f}, y={y:.2f}")

    plt.figure()
    plt.plot(X,Y, ".")
    plt.show()
