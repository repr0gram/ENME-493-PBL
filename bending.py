#bending
import math
import landing_force_balance
import os 
os.system('cls')


l = 0.2045 #m 
y = 0.08 #m 
for stress in landing_force_balance.normal_force:
    M = (stress / 2) * l
    I = (math.pi * (0.08**4)) / 4
    Sigma = (M * y) / I
    print(Sigma)


