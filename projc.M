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
bank_angle = 5:15;

% 2 Wheel landing results
normal_force4W = zeros(size(aoa));
moment4W = zeros(size(aoa));
sigmaBend4W = zeros(size(aoa));
taoShear4W = zeros(size(aoa));
taoTorsion4W = zeros(size(aoa));
vonMises4W = zeros(size(aoa));
vonMisesSF4W = zeros(size(aoa));
tresca4W = zeros(size(aoa));
trescaSF4W = zeros(size(aoa));
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
        n = 2;    %vertical landing load factor

    % Normal force
        Fm = (n*W) - (L * cosd(alpha));
        normal_force(i) = Fm;

    % 4 WHEEL LANDING
        l = 0.2045; %m
        y = 0.08;   %m

        % Bending Moment
        M4W = (Fm / 4) * l;
        moment2W(i) = M4W;
    
        % Bending stress
        SigmaBend4W = (M4W * y) / I_axle;
        sigmaBend4W(i) = SigmaBend4W;

        % Transverse Shear
        Q = (pi * (0.08)^3) / 2;
        V = (Fm/4);
        t = 0.162;
        TaoShear4W = (V * Q) / I_axle * t;
        taoShear4W(i) = TaoShear4W;

        % Torsional Shear
        Fric = Mew * (Fm/4); 
        T = Fric * (0.08);
        TaoTorsion4W = (16 * T) / (pi * (0.08)^3);
        taoTorsion4W(i) = TaoTorsion4W;

        %Von Mises
        VonMises4W = sqrt((SigmaBend4W)^2 + 3*(TaoShear4W + TaoTorsion4W)^2);
        vonMises4W(i) = VonMises4W;
        VonMisesSF4W = SigmaY / VonMises4W;
        vonMisesSF4W(i) = VonMisesSF4W;

        %Tresca Shear
        Tresca4W = sqrt((SigmaBend4W/2)^2 + (TaoShear4W + TaoTorsion4W)^2);
        tresca4W(i) = Tresca4W;
        TrescaSF4W = (SigmaY/2)/Tresca4W;
        trescaSF4W(i) = TrescaSF4W;

        %Torsional Fatigue (infinte lifetime)
        FatigueSF = (LC * Cg * Ct * Cr * Cs * SnPrime)/TaoTorsion4W;
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
T1 = table(aoa', sigmaBend4W', taoShear4W', taoTorsion4W', 'VariableNames', {'aoa_deg', 'BendingStress_Pa', 'TransverseShear_Pa', 'TorsionShear_Pa'});
disp(T1)

disp('Two Wheel Landing Failures')
T2 = table(aoa', vonMises4W', vonMisesSF4W', tresca4W', trescaSF4W', fatigueSF', 'VariableNames', {'aoa_deg', 'VonMises_Pa', 'VonMisesSF', 'Tresca_Pa', 'TrescaSF', 'FatigueSF'});
disp(T2)

disp('One Wheel Landing Stresses')
T3 = table(aoa', sigmaBend1W', taoShear1W', taoTorsion1W', 'VariableNames', {'aoa_deg', 'BendingStress_Pa', 'TransverseShear_Pa', 'TorsionShear_Pa'});
disp(T3)

disp('One Wheel Landing Failures')
T4 = table(aoa', vonMises1W', vonMisesSF1W', tresca1W', trescaSF1W', 'VariableNames', {'aoa_deg', 'VonMises_Pa', 'VonMisesSF', 'Tresca_Pa', 'TrescaSF'});
disp(T4)