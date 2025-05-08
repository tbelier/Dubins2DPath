import numpy as np

cx1,cy1,r1 = -32.32,11.85, 2.2
cx2,cy2,r2 = -27.42,15.25, 1.5

eta = np.arctan((cy2-cy1)/(cx2-cx1))
#tmp1 = (r1-r2)/np.sqrt((cy2-cy1)**2+(cx2-cx1)**2)
tmp2 = r1/np.sqrt((cy2-cy1)**2+(cx2-cx1)**2)
alpha = np.arccos(tmp2)

xI1 = cx1+r1*np.cos(alpha+eta)
yI1 = cy1+r1*np.sin(alpha+eta)

print(f"xI1 : {xI1}")
print(f"yI1 : {yI1}")

xI2 = cx1+r1*np.cos(alpha+eta-np.pi)
yI2 = cy1+r1*np.sin(alpha+eta-np.pi)

print(f"xI2 : {xI2}")
print(f"yI2 : {yI2}")