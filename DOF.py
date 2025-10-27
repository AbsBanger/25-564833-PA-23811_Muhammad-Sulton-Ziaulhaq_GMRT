import math

a = 56
b = 65
sudut_theta1 = 40
sudut_theta2 = 30

sudut_theta_radian1 = math.radians(sudut_theta1)
sudut_theta_radian2 = math.radians(sudut_theta2)

x = a*math.cos(sudut_theta_radian1) + a*math.cos(sudut_theta_radian1 + sudut_theta_radian2)
y = b*math.sin(sudut_theta_radian1) + b*math.sin(sudut_theta_radian1 + sudut_theta_radian2)

print("Koordinat titik akhir pada ujung kaki adalah")
print(f"x = {x:.0f}")
print(f"y = {y:.0f}")