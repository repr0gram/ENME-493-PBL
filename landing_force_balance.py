import math

Max_landing_mass = 75000
landing_velocity = 142 #knots
V_l = landing_velocity * 0.5144
Cl_max = 1.4 #for 50 degree flap activation
Sw = 122.4 #m^2 (more like wing area, but thats the best thing I can find)
Rho = 1.1 #kg/m^3 (at yyc via ai)


L = 0.5*Rho*(V_l)**2*Cl_max*Sw
W = Max_landing_mass * 9.81 
aoa = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

for alpha in aoa:
    Fm= W-(L*math.cos(math.radians(alpha)))
    print(Fm)
