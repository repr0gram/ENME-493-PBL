clc
clear

% Given values
Max_landing_mass = 75000;
landing_velocity = 142;   % knots
V_1 = landing_velocity * 0.5144;   % m/s

Cl_max = 1.4;   % assumed lift coefficient
Sw = 122.4;     % m^2
Rho = 1.1;      % kg/m^3

Mew = 0.8;      % between tarmac / wheel
SigmaY = 1930000000;

% Force calculations
L = 0.5 * Rho * (V_1)^2 * Cl_max * Sw;
W = Max_landing_mass * 9.81;

aoa = 0:10;

% 2 Wheel landing results
normal_force2W = zeros(size(aoa));
moment2W = zeros(size(aoa));
sigmaBend2W = zeros(size(aoa));
taoShear2W = zeros(size(aoa));
taoTorsion2W = zeros(size(aoa));
vonMises2W = zeros(size(aoa));
vonMisesSF2W = zeros(size(aoa));
tresca2W = zeros(size(aoa));
trescaSF2W = zeros(size(aoa));

I_axle = (pi * (0.08^4)) / 4;

for i = 1:length(aoa)
    alpha = aoa(i);

    % 2 WHEEL LANDING
        l = 0.2045; %m
        y = 0.08;   %m

        % Normal force
        Fm2W = W - (L * cosd(alpha));
        normal_force2W(i) = Fm2W;

        % Bending Moment
        M2W = (Fm2W / 4) * l;
        moment2W(i) = M2W;
    
        % Bending stress
        SigmaBend2W = (M2W * y) / I_axle;
        sigmaBend2W(i) = SigmaBend2W;

        % Transverse Shear
        Q = (pi * (0.08)^3) / 2;
        V = (Fm2W/4);
        t = 0.162;
        TaoShear2W = (V * Q) / I_axle * t;
        taoShear2W(i) = TaoShear2W;

        % Torsional Shear
        Fric = Mew * (Fm2W/4); 
        T = Fric * (0.08);
        TaoTorsion2W = (16 * T) / (pi * (0.08)^3);
        taoTorsion2W(i) = TaoTorsion2W;

        %Von Mises
        VonMises2W = sqrt((SigmaBend2W)^2 + 3*(TaoShear2W + TaoTorsion2W)^2);
        vonMises2W(i) = VonMises2W;
        VonMisesSF2W = SigmaY / VonMises2W;
        vonMisesSF2W(i) = VonMisesSF2W;

        %Tresca Shear
        Tresca2W = sqrt((SigmaBend2W/2)^2 + (TaoShear2W + TaoTorsion2W)^2);
        tresca2W(i) = Tresca2W;
        TrescaSF2W = (SigmaY/2)/Tresca2W;
        trescaSF2W(i) = TrescaSF2W;

        %Torsional Fatigue (infinte)


end

% Display results
disp('One Wheel Landing Stresses')
disp(' aoa BendingStress(Pa) TransverseShear(Pa) TorsionShear' )
disp([aoa' sigmaBend2W' taoShear2W' taoTorsion2W'])
disp('One Wheel Landing Failures')
disp('VonMises VonMisesSF Tresca TrescaSF')
disp([vonMises2W' vonMisesSF2W' tresca2W' trescaSF2W'])
