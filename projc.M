clc
clear

% Given values
Max_landing_mass = 75000;
landing_velocity = 142;   % knots
V_1 = landing_velocity * 0.5144;   % m/s

Cl_max = 1.4;   % assumed lift coefficient
Sw = 122.4;     % m^2
Rho = 1.1;      % kg/m^3

l = 0.2045;     % m
y = 0.08;       % m

% Force calculations
L = 0.5 * Rho * (V_1)^2 * Cl_max * Sw;
W = Max_landing_mass * 9.81;

aoa = 0:10;

normal_force = zeros(size(aoa));
moment = zeros(size(aoa));
sigma = zeros(size(aoa));

I = (pi * (0.08^4)) / 4;

for i = 1:length(aoa)
    alpha = aoa(i);

    % Normal force
    Fm = W - (L * cosd(alpha));
    normal_force(i) = Fm;

    % Moment
    M = (Fm / 2) * l;
    moment(i) = M;

    % Bending stress
    Sigma = (M * y) / I;
    sigma(i) = Sigma;
end

% Display results
T = table(aoa', normal_force', moment', sigma', 'VariableNames', {'AOA', 'Normal_Force_N', 'Moment_Nm', 'Stress_Pa'});
disp(T)
