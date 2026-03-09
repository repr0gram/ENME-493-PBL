%% Landing Gear Structural Safety Analysis
clc
clear

%% ------------- Aircraft/Landing Inputs ------------

Max_landing_mass = 75000;         % kg
landing_velocity = 142;           % knots
V = landing_velocity*0.5144;      % m/s

Cl_max = 1.4;
Sw = 122.4;
rho = 1.1;

mu = 0.8;                         % tire friction

SigmaY = 1.51e9;                  % Pa
Su = 1.58e9;
E = 2.049e11;

bank_angle = 5;                   % deg
n = 1.5;                          % landing load factor

%% ------------- Axle Geometry ------------

r = 0.08;                         % axle radius
l = 0.2045;                       % moment arm
I_axle = (pi*r^4)/4;

Q = (pi*r^3)/2;
t = 0.162;

%% ------------ Buckling Loads -------------


Pcrit_Strut = ((pi)^3*(0.1155)^4*E)/(4*(2.788)^2);
Pcrit_Stay  = ((pi)^2*E*(1.4e-6))/(1.093^2);

%% ------------ Aerodynamic Lift -------------

Lift = 0.5*rho*V^2*Cl_max*Sw;
Weight = Max_landing_mass*9.81;

aoa = 0:10;
N = length(aoa);

%% ------------ Preallocate Results -------------

vonMisesSF1W = zeros(N,1);
vonMisesSF4W = zeros(N,1);

trescaSF1W = zeros(N,1);
trescaSF4W = zeros(N,1);

buckleStrut1WSF = zeros(N,1);
buckleStrut4WSF = zeros(N,1);

buckleStay1WSF = zeros(N,1);
buckleStay4WSF = zeros(N,1);

%% -------------------------
% Main Loop
%% -------------------------

for i = 1:N

    alpha = aoa(i);

    % Total landing force
    Fm = (n*Weight) - Lift*cosd(alpha);

    %% =====================
    % FOUR WHEEL LANDING
    %% =====================

    Vload = Fm/4;

    % Bending
    M = Vload*l;
    sigma_b = (M*r)/I_axle;

    % Transverse shear
    tau_v = (Vload*Q)/(I_axle*t);

    % Torsion from braking
    Fric = mu*Vload;
    T = Fric*r;
    tau_t = (16*T)/(pi*r^3);

    % Von Mises
    sigma_vm = sqrt(sigma_b^2 + 3*(tau_v+tau_t)^2);
    vonMisesSF4W(i) = SigmaY/sigma_vm;

    % Tresca
    tau_max = sqrt((sigma_b/2)^2 + (tau_v+tau_t)^2);
    trescaSF4W(i) = (SigmaY/2)/tau_max;

    % Strut Buckling
    buckleStrut4WSF(i) = Pcrit_Strut/Fm;

    % Side Stay Force Estimate (gear load distribution)
    Fstay = 0.35*Fm;
    buckleStay4WSF(i) = Pcrit_Stay/Fstay;

    %% =====================
    % ONE WHEEL LANDING
    %% =====================

    Vload = Fm*cosd(bank_angle);

    % Bending
    M = Vload*l*cosd(bank_angle);
    sigma_b = (M*r)/I_axle;

    % Shear
    tau_v = (Vload*Q)/(I_axle*t);

    % Torsion
    Fric = mu*Vload;
    T = Fric*r;
    tau_t = (16*T)/(pi*r^3);

    % Von Mises
    sigma_vm = sqrt(sigma_b^2 + 3*(tau_v+tau_t)^2);
    vonMisesSF1W(i) = SigmaY/sigma_vm;

    % Tresca
    tau_max = sqrt((sigma_b/2)^2 + (tau_v+tau_t)^2);
    trescaSF1W(i) = (SigmaY/2)/tau_max;

    % Strut Buckling
    buckleStrut1WSF(i) = Pcrit_Strut/Fm;

    % Stay Buckling
    Fstay = 0.5*Fm;
    buckleStay1WSF(i) = Pcrit_Stay/Fstay;

end

%% -------------------------
% Output Tables
%% -------------------------

T1 = table(aoa',vonMisesSF1W,trescaSF1W,buckleStrut1WSF,buckleStay1WSF,...
    'VariableNames',{'AoA','VonMisesSF','TrescaSF','StrutBucklingSF','StayBucklingSF'});

T4 = table(aoa',vonMisesSF4W,trescaSF4W,buckleStrut4WSF,buckleStay4WSF,...
    'VariableNames',{'AoA','VonMisesSF','TrescaSF','StrutBucklingSF','StayBucklingSF'});

disp('One Wheel Landing Safety Factors')
disp(T1)

disp('Four Wheel Landing Safety Factors')
disp(T4)

%% -------------------------
% Safety Factor Plots
%% -------------------------

figure
tiledlayout(4,2)

nexttile
plot(aoa,vonMisesSF1W,'LineWidth',2)
title('Von Mises SF - 1 Wheel')
xlabel('AoA (deg)')
ylabel('Safety Factor')
grid on

nexttile
plot(aoa,vonMisesSF4W,'LineWidth',2)
title('Von Mises SF - 4 Wheel')
xlabel('AoA (deg)')
ylabel('Safety Factor')
grid on

nexttile
plot(aoa,trescaSF1W,'LineWidth',2)
title('Tresca SF - 1 Wheel')
xlabel('AoA (deg)')
ylabel('Safety Factor')
grid on

nexttile
plot(aoa,trescaSF4W,'LineWidth',2)
title('Tresca SF - 4 Wheel')
xlabel('AoA (deg)')
ylabel('Safety Factor')
grid on

nexttile
plot(aoa,buckleStrut1WSF,'LineWidth',2)
title('Strut Buckling SF - 1 Wheel')
xlabel('AoA (deg)')
ylabel('Safety Factor')
grid on

nexttile
plot(aoa,buckleStrut4WSF,'LineWidth',2)
title('Strut Buckling SF - 4 Wheel')
xlabel('AoA (deg)')
ylabel('Safety Factor')
grid on

nexttile
plot(aoa,buckleStay1WSF,'LineWidth',2)
title('Side Stay Buckling SF - 1 Wheel')
xlabel('AoA (deg)')
ylabel('Safety Factor')
grid on

nexttile
plot(aoa,buckleStay4WSF,'LineWidth',2)
title('Side Stay Buckling SF - 4 Wheel')
xlabel('AoA (deg)')
ylabel('Safety Factor')
grid on
