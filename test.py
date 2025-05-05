from easydubins import dubin_path

x0, y0, theta0 = 0,0,0
x1, y1, theta1 = 10,0,0
radius = 1
    
# Calculer le chemin Dubins en fonction des sliders
all_solutions = dubin_path.all_dubins_paths([x0, y0, theta0], [x1, y1, theta1], radius=radius)

print(f"{len(all_solutions)} solutions possibles : ")
for k in range(len(all_solutions)):
    print(all_solutions[k])