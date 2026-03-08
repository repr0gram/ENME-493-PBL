clc
clear

% Given values
Max_landing_mass = 75000;
landing_velocity = 142;   % knots
V_1 = landing_velocity * 0.5144;   % m/s

Cl_max = 1.4;   % assumed lift coefficient
Sw = 122.4;     % m^2
Rho = 1.1;      % kg/m^3

Mew = 0.8;           % between tarmac / wheel
SigmaY = 1590000000; % yield strength of axle (Pa)
Su = 1930000000;     % ultimate tensile strength of axle (Pa)

% Axle fatigue Coefficients
LC = 0.58;      % Torsion
Cg = 1.51 * (162)^(-0.157); % diameter driven
Ct = 1;         % Temp - assume 1
Cr = 0.81;      % Reliability of 99%
Cs = 0.28;      % Forged Steel 
SnPrime = 0.5*Su; % Steel

% Force calculations
L = 0.5 * Rho * (V_1)^2 * Cl_max * Sw;
W = Max_landing_mass * 9.81;

aoa = 0:10;
bank_angle = 0:10;

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
fatigueSF = zeros(size(aoa));

% 2 Wheel landing results
normal_force1W = zeros(size(aoa));
moment1W = zeros(size(aoa));
sigmaBend1W = zeros(size(aoa));
taoShear1W = zeros(size(aoa));
taoTorsion1W = zeros(size(aoa));
vonMises1W = zeros(size(aoa));
vonMisesSF1W = zeros(size(aoa));
tresca1W = zeros(size(aoa));
trescaSF1W = zeros(size(aoa));

I_axle = (pi * (0.08^4)) / 4;

for i = 1:length(aoa)
    alpha = aoa(i);
    beta = bank_angle(i);

        l = 0.2045; %m
        y = 0.08;   %m

    % Normal force
        Fm = W - (L * cosd(alpha));
        normal_force(i) = Fm;

    % 2 WHEEL LANDING
        l = 0.2045; %m
        y = 0.08;   %m

        % Bending Moment
        M2W = (Fm / 4) * l;
        moment2W(i) = M2W;
    
        % Bending stress
        SigmaBend2W = (M2W * y) / I_axle;
        sigmaBend2W(i) = SigmaBend2W;

        % Transverse Shear
        Q = (pi * (0.08)^3) / 2;
        V = (Fm/4);
        t = 0.162;
        TaoShear2W = (V * Q) / I_axle * t;
        taoShear2W(i) = TaoShear2W;

        % Torsional Shear
        Fric = Mew * (Fm/4); 
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

        %Torsional Fatigue (infinte lifetime)
        FatigueSF = (LC * Cg * Ct * Cr * Cs * SnPrime)/TaoTorsion2W;
        fatigueSF(i) = FatigueSF;

    % 2 WHEEL LANDING
        
        % Bending Moment
        M1W = (Fm * cosd(beta)) * (l * cosd(beta));
        moment1W(i) = M1W;

        % Bending stress
        SigmaBend1W = (M1W * y) / I_axle;
        sigmaBend1W(i) = SigmaBend1W;

        % Transverse Shear
        Q = (pi * (0.08)^3) / 2;
        V = (Fm * cosd(beta));
        t = 0.162;
        TaoShear1W = (V * Q) / I_axle * t;
        taoShear1W(i) = TaoShear1W;

        % Torsional Shear
        Fric = Mew * (Fm * cosd(beta)); 
        T = Fric * (0.08);
        TaoTorsion1W = (16 * T) / (pi * (0.08)^3);
        taoTorsion1W(i) = TaoTorsion1W;

        %Von Mises
        VonMises1W = sqrt((SigmaBend1W)^2 + 3*(TaoShear1W + TaoTorsion1W)^2);
        vonMises1W(i) = VonMises1W;
        VonMisesSF1W = SigmaY / VonMises1W;
        vonMisesSF1W(i) = VonMisesSF1W;

        %Tresca Shear
        Tresca1W = sqrt((SigmaBend1W/2)^2 + (TaoShear1W + TaoTorsion1W)^2);
        tresca1W(i) = Tresca1W;
        TrescaSF1W = (SigmaY/2)/Tresca1W;
        trescaSF1W(i) = TrescaSF1W;
end

% Display results
disp('Two Wheel Landing Stresses')
disp(' aoa BendingStress(Pa) TransverseShear(Pa) TorsionShear' )
disp([aoa' sigmaBend2W' taoShear2W' taoTorsion2W'])
disp('Two Wheel Landing Failures')
disp('VonMises VonMisesSF Tresca TrescaSF FatigueSF')
disp([vonMises2W' vonMisesSF2W' tresca2W' trescaSF2W' fatigueSF'])

disp('One Wheel Landing Stresses')
disp(' aoa BendingStress(Pa) TransverseShear(Pa) TorsionShear' )
disp([aoa' sigmaBend1W' taoShear1W' taoTorsion1W'])
disp('One Wheel Landing Failures')
disp('VonMises VonMisesSF Tresca TrescaSF FatigueSF')
disp([vonMises1W' vonMisesSF1W' tresca1W' trescaSF1W'])