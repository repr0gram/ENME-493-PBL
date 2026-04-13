%% Deformation Cases
clc
clear

% Given values
Max_landing_mass = 75000;
landing_velocity = 142;            % knots
V_1 = landing_velocity * 0.5144;   % m/s
x_cg = 17.44;
x_n = 5.07;
x_m = 21.976;

Cl_max = 1.4;   % assumed lift coefficient
Sw = 122.4;     % m^2
Rho = 1.1;      % kg/m^3

Mew = 0.8;           % between tarmac / wheel
SigmaY = 1510000000; % yield strength of axle (Pa)
Su = 2030000000;     % ultimate tensile strength of axle (Pa)
E = 204900000000;    % Elastic Modulus

% Axle fatigue Coefficients
LC = 0.58;        % Torsion
Cg = 1.51 * (162)^(-0.157); % diameter driven
Ct = 1;           % Temp - assume 1
Cr = 0.81;         % Reliability of 99%
Cs = 0.77;        % Forged Steel 
SnPrime = 0.5*Su; % Steel

%Buckling Pcrit
Pcrit_Strut = ((pi)^3 * (0.1155)^4 * E) / (4 * (2.788)^2);
Pcrit_Stay = ((pi)^2 * E * (0.0000014)) / (1.093^2);

% Force calculations
L = 0.5 * Rho * (V_1)^2 * Cl_max * Sw;
W = Max_landing_mass * 9.81;
Fm_static = (W*(x_cg-x_n))/(x_m-x_n);

aoa = 0:10;
bank_angle = 0;

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
buckleStrut4WSF = zeros(size(aoa));
buckleStay4WSF = zeros(size(aoa));

% 4 Wheel landing results
normal_force1W = zeros(size(aoa));
moment1W = zeros(size(aoa));
sigmaBend1W = zeros(size(aoa));
taoShear1W = zeros(size(aoa));
taoTorsion1W = zeros(size(aoa));
vonMises1W = zeros(size(aoa));
vonMisesSF1W = zeros(size(aoa));
tresca1W = zeros(size(aoa));
trescaSF1W = zeros(size(aoa));
buckleStrut1WSF = zeros(size(aoa));
buckleStay1WSF = zeros(size(aoa));

I_axle = (pi * (0.08^4)) / 4;

sFactors = 1.25:0.1:2;
d_min_all = zeros(length(aoa), length(sFactors));
fatigue_safety_factor_all = zeros(size(aoa));

for i = 1:length(aoa)
    alpha = aoa(i);
    beta = bank_angle;

        l = 0.2045; %m
        y = 0.08;   %m
        n = 1.5;    %vertical landing load factor

    % Normal force
        Fm = (n*W) - (L * cosd(alpha));
        normal_force(i) = Fm;

    % 4 WHEEL LANDING
        %Inputs: 
        %Weight (Normal Force on tires): Newtons, kilograms
        weight = 75000*9.81;
        N1x = 0;
        N2x = 0;
        N1y = Fm/4;
        N2y = Fm/4;
        N1z = 0;
        N2z = 0;
        %Lengths and angles of members: Meters and degrees
        L_strut = 2.8687;
        L3 = 1.0644;
        L7 = 2.48532;
        
        L_lock_stay = 0.35379;
        theta_xy_lock_stay = 59.16*(pi()/180); %Note WRT x-axis
        
        z7 = 0.7374;
        zb1 = 0.7374; %approximately the same
        
        L_side_stay_lower = 1.093493;
        L_side_stay_upper = 0.70094;
        
        theta3_xy = 38*(pi()/180); %Note: WRT y-axis
        theta3_yz = 38*(pi()/180); %Note: WRT y-axis
        theta3_xz = 45*(pi()/180); 
        
        theta5_xy = 67.9*(pi()/180);
        theta5_yz = 67.9*(pi()/180);
        theta5_xz = 45*(pi()/180);
        u5x = tan(theta5_yz)/sqrt(tan(theta5_yz)^2+tan(theta5_xz)^2+tan(theta5_xy)^2);
        u5y = tan(theta5_xz)/sqrt(tan(theta5_yz)^2+tan(theta5_xz)^2+tan(theta5_xy)^2);
        u5z = tan(theta5_xy)/sqrt(tan(theta5_yz)^2+tan(theta5_xz)^2+tan(theta5_xy)^2);
        
        theta7_xy = 59.16*(pi()/180);
        theta7_yz = 59.16*(pi()/180);
        theta7_xz = 45*(pi()/180);
        u7x = tan(theta7_yz)/sqrt(tan(theta7_yz)^2+tan(theta7_xz)^2+tan(theta7_xy)^2);
        u7y = tan(theta7_xz)/sqrt(tan(theta7_yz)^2+tan(theta7_xz)^2+tan(theta7_xy)^2);
        u7z = tan(theta7_xy)/sqrt(tan(theta7_yz)^2+tan(theta7_xz)^2+tan(theta7_xy)^2);
        
        %Outputs: 1 = wheels and axle, 2 = main strut, 3 = side stay lower, 4 = side stay upper, 
        % 5 = Down lock actuator, 6 = lock stay lower, 7 = lock stay upper
        
        %1 Wheels and axle:
        F1x = -N1x - N2x;
        F1y = -N1y - N2y;
        F1z = -N1z - N2z;
        M1z = N1y*0.46355 - N2y*0.46355 + N1x*0.635 - N2x*0.635;
        M1y = -N1z*0.46355 + N2z*0.46355;
        
        %2 Main Strut:
        syms F3x F3y F3z F7 Fb1x Fb1y Fb1z Fb2x Fb2y Fb2z
        F7x = F7*u7x;
        F7y = F7*u7y;
        F7z = F7*u7z;
        
        eqn1 = -F1x + F3x + F7x + Fb1x + Fb2x == 0;
        eqn2 = -F1y + F3y + F7y + Fb1y + Fb2y == 0;
        eqn3 = Fb1z + Fb2z + F7z + F3z - F1z == 0;
        eqn4 = -M1z - L_strut*F1x + (L_strut - L3)*F3x + (L_strut - L7)*F7x == 0;
        eqn5 = -L3*F3z - L7*F7z - L_strut*Fb1z - L_strut*Fb2z + z7*F7y + zb1*Fb1y == 0;
        eqn6 = -M1y + z7*F7x + zb1*Fb1x == 0;
        
        %4 upper:
        syms Fb3x Fb3y Fb3z F6x F6y F6z F5
        F5x = F5*u5x;
        F5y = F5*u5y;
        F5z = F5*u5z;
        
        eqn7 = Fb3x + F6x + F5x - F3x == 0;
        eqn8 = Fb3y + F6y + F5y - F3y == 0;
        eqn9 = -F3z + F6z + Fb3z + F5z == 0;
        eqn10 = -L_side_stay_upper*sin(theta3_xy)*Fb3y - L_side_stay_upper*sin(theta3_xy)*F5y - L_side_stay_upper*cos(theta3_xy)*Fb3x - L_side_stay_upper*cos(theta3_xy)*F6x == 0;
        eqn11 = -L_side_stay_upper*sin(theta3_yz)*Fb3y - L_side_stay_upper*sin(theta3_yz)*F5y - L_side_stay_upper*cos(theta3_yz)*F5z - L_side_stay_upper*cos(theta3_yz)*Fb3z == 0;
        eqn12 = -L_side_stay_upper*sin(theta3_xz)*Fb3z - L_side_stay_upper*sin(theta3_xz)*F5z - L_side_stay_upper*cos(theta3_xz)*F5x - L_side_stay_upper*cos(theta3_xz)*Fb3x == 0;
        
        %6 lock stay lower: (Can be treated as a two-force member)
        eqn13 = -F6x - F5x - F7x == 0;
        eqn14 = -F6y - F5y - F7y == 0;
        eqn15 = -F5z - F7z - F6z == 0;
        
        equations = [eqn1, eqn2, eqn3, eqn4, eqn5, eqn6, eqn7, eqn8, eqn9, eqn10, eqn11, eqn12, eqn13, eqn14, eqn15];
        variables = [F3x, F3y, F3z, F7, Fb1x, Fb1y, Fb1z, Fb2x, Fb2y, Fb2z, Fb3x, Fb3y, Fb3z, F6x, F6y, F6z, F5];
        [A,B] = equationsToMatrix(equations,variables);
        sol = linsolve(A,B);
        
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

        % Buckling 
        BuckleStrut4WSF = Pcrit_Strut / sqrt(F1x^2 + F1y^2 + F1z^2);
        buckleStrut4WSF(i) = BuckleStrut4WSF;
        BuckleStay4WSF = Pcrit_Stay / sqrt(sol(1)^2 + sol(2)^2 + sol(3)^2);
        buckleStay4WSF(i) = BuckleStay4WSF;

    % 1 WHEEL LANDING
        %Inputs: 
        %Weight (Normal Force on tires): Newtons, kilograms
        weight = 75000*9.81;
        N1x = 0;
        N2x = 0;
        N1y = Fm;
        N2y = 0;
        N1z = 0;
        N2z = 0;
        %Lengths and angles of members: Meters and degrees
        L_strut = 2.8687;
        L3 = 1.0644;
        L7 = 2.48532;
        
        L_lock_stay = 0.35379;
        theta_xy_lock_stay = 59.16*(pi()/180); %Note WRT x-axis
        
        z7 = 0.7374;
        zb1 = 0.7374; %approximately the same
        
        L_side_stay_lower = 1.093493;
        L_side_stay_upper = 0.70094;
        
        theta3_xy = 38*(pi()/180); %Note: WRT y-axis
        theta3_yz = 38*(pi()/180); %Note: WRT y-axis
        theta3_xz = 45*(pi()/180); 
        
        theta5_xy = 67.9*(pi()/180);
        theta5_yz = 67.9*(pi()/180);
        theta5_xz = 45*(pi()/180);
        u5x = tan(theta5_yz)/sqrt(tan(theta5_yz)^2+tan(theta5_xz)^2+tan(theta5_xy)^2);
        u5y = tan(theta5_xz)/sqrt(tan(theta5_yz)^2+tan(theta5_xz)^2+tan(theta5_xy)^2);
        u5z = tan(theta5_xy)/sqrt(tan(theta5_yz)^2+tan(theta5_xz)^2+tan(theta5_xy)^2);
        
        theta7_xy = 59.16*(pi()/180);
        theta7_yz = 59.16*(pi()/180);
        theta7_xz = 45*(pi()/180);
        u7x = tan(theta7_yz)/sqrt(tan(theta7_yz)^2+tan(theta7_xz)^2+tan(theta7_xy)^2);
        u7y = tan(theta7_xz)/sqrt(tan(theta7_yz)^2+tan(theta7_xz)^2+tan(theta7_xy)^2);
        u7z = tan(theta7_xy)/sqrt(tan(theta7_yz)^2+tan(theta7_xz)^2+tan(theta7_xy)^2);
        
        %Outputs: 1 = wheels and axle, 2 = main strut, 3 = side stay lower, 4 = side stay upper, 
        % 5 = Down lock actuator, 6 = lock stay lower, 7 = lock stay upper
        
        %1 Wheels and axle:
        F1x = -N1x - N2x;
        F1y = -N1y - N2y;
        F1z = -N1z - N2z;
        M1z = N1y*0.46355 - N2y*0.46355 + N1x*0.635 - N2x*0.635;
        M1y = -N1z*0.46355 + N2z*0.46355;
        
        %2 Main Strut:
        syms F3x F3y F3z F7 Fb1x Fb1y Fb1z Fb2x Fb2y Fb2z
        F7x = F7*u7x;
        F7y = F7*u7y;
        F7z = F7*u7z;
        
        eqn1 = -F1x + F3x + F7x + Fb1x + Fb2x == 0;
        eqn2 = -F1y + F3y + F7y + Fb1y + Fb2y == 0;
        eqn3 = Fb1z + Fb2z + F7z + F3z - F1z == 0;
        eqn4 = -M1z - L_strut*F1x + (L_strut - L3)*F3x + (L_strut - L7)*F7x == 0;
        eqn5 = -L3*F3z - L7*F7z - L_strut*Fb1z - L_strut*Fb2z + z7*F7y + zb1*Fb1y == 0;
        eqn6 = -M1y + z7*F7x + zb1*Fb1x == 0;
        
        %4 upper:
        syms Fb3x Fb3y Fb3z F6x F6y F6z F5
        F5x = F5*u5x;
        F5y = F5*u5y;
        F5z = F5*u5z;
        
        eqn7 = Fb3x + F6x + F5x - F3x == 0;
        eqn8 = Fb3y + F6y + F5y - F3y == 0;
        eqn9 = -F3z + F6z + Fb3z + F5z == 0;
        eqn10 = -L_side_stay_upper*sin(theta3_xy)*Fb3y - L_side_stay_upper*sin(theta3_xy)*F5y - L_side_stay_upper*cos(theta3_xy)*Fb3x - L_side_stay_upper*cos(theta3_xy)*F6x == 0;
        eqn11 = -L_side_stay_upper*sin(theta3_yz)*Fb3y - L_side_stay_upper*sin(theta3_yz)*F5y - L_side_stay_upper*cos(theta3_yz)*F5z - L_side_stay_upper*cos(theta3_yz)*Fb3z == 0;
        eqn12 = -L_side_stay_upper*sin(theta3_xz)*Fb3z - L_side_stay_upper*sin(theta3_xz)*F5z - L_side_stay_upper*cos(theta3_xz)*F5x - L_side_stay_upper*cos(theta3_xz)*Fb3x == 0;
        
        %6 lock stay lower: (Can be treated as a two-force member)
        eqn13 = -F6x - F5x - F7x == 0;
        eqn14 = -F6y - F5y - F7y == 0;
        eqn15 = -F5z - F7z - F6z == 0;
        
        equations = [eqn1, eqn2, eqn3, eqn4, eqn5, eqn6, eqn7, eqn8, eqn9, eqn10, eqn11, eqn12, eqn13, eqn14, eqn15];
        variables = [F3x, F3y, F3z, F7, Fb1x, Fb1y, Fb1z, Fb2x, Fb2y, Fb2z, Fb3x, Fb3y, Fb3z, F6x, F6y, F6z, F5];
        [A,B] = equationsToMatrix(equations,variables);
        sol = linsolve(A,B);

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

    % Buckling 
        BuckleStrut1WSF = Pcrit_Strut / sqrt(F1x^2 + F1y^2 + F1z^2);
        buckleStrut1WSF(i) = BuckleStrut1WSF;
        BuckleStay1WSF = Pcrit_Stay / sqrt(sol(1)^2 + sol(2)^2 + sol(3)^2);
        buckleStay1WSF(i) = BuckleStay1WSF;

    %Bolt diameter
        F7_val = double(sol(4));
        F7y_val = F7_val * u7y;

        for k = 1:length(sFactors)
            sFactor = sFactors(k);
            d_min_all(i,k) = sqrt(abs((F7y_val * sFactor)) / (3*pi*370.2*10^6));
        end

        Pmean = (abs((sqrt(F1x^2 + F1y^2 + F1z^2))) + 0)/2;
        Palternating = (abs(sqrt(F1x^2 + F1y^2 + F1z^2)) - 0)/2;

        Sigma_m = Pmean / (0.0419);
        sigma_a = Palternating / (0.0419);

        fatigue_safety_factor_all(i) = 1/(((sigma_a/(79.23*10^6)) + (Sigma_m/(2030*10^6))));
end

% Display results
% Four-wheel landing output tables
T4W_stress = table(aoa', sigmaBend4W', taoShear4W', taoTorsion4W', ...
    'VariableNames', {'aoa_deg','BendingStress_Pa','TransverseShear_Pa','TorsionShear_Pa'});
disp('Four Wheel Landing Stresses');
disp(T4W_stress);

T4W_failure = table(aoa', vonMises4W', vonMisesSF4W', tresca4W', trescaSF4W', fatigueSF', ...
    'VariableNames', {'aoa_deg','VonMises_Pa','VonMisesSF','Tresca_Pa','TrescaSF','FatigueSF'});
disp('Four Wheel Landing Failures');
disp(T4W_failure);

T4W_buckle = table(aoa', buckleStrut4WSF', buckleStay4WSF', ...
    'VariableNames', {'aoa_deg','StrutBuckleSF','StayBuckleSF'});
disp('Four Wheel Landing Buckling Safety Factors');
disp(T4W_buckle);

% One-wheel landing output tables
T1W_stress = table(aoa', sigmaBend1W', taoShear1W', taoTorsion1W', ...
    'VariableNames', {'aoa_deg','BendingStress_Pa','TransverseShear_Pa','TorsionShear_Pa'});
disp('One Wheel Landing Stresses');
disp(T1W_stress);

T1W_failure = table(aoa', vonMises1W', vonMisesSF1W', tresca1W', trescaSF1W', ...
    'VariableNames', {'aoa_deg','VonMises_Pa','VonMisesSF','Tresca_Pa','TrescaSF'});
disp('One Wheel Landing Failures');
disp(T1W_failure);

T1W_buckle = table(aoa', buckleStrut1WSF', buckleStay1WSF', ...
    'VariableNames', {'aoa_deg','StrutBuckleSF','StayBuckleSF'});
disp('One Wheel Landing Buckling Safety Factors');
disp(T1W_buckle);

T_fatigue = table(aoa', fatigue_safety_factor_all', ...
    'VariableNames', {'aoa_deg','FatigueSafetyFactor'});
disp('Strut Fatigue Safety Factor - 1 Wheel Landing');
disp(T_fatigue);


%% Safety Factor Plots

disp("Plotting now")
drawnow

exportDir = fullfile(fileparts(mfilename('fullpath')), 'exports');
if ~isfolder(exportDir)
    mkdir(exportDir);
end

fig1 = figure;
tiledlayout(4,2)

% Von Mises SF - 1 Wheel
nexttile
plot(aoa,vonMisesSF1W,'LineWidth',2)
xlabel('Angle of Attack (deg)')
ylabel('Safety Factor')
title('Von Mises Safety Factor - 1 Wheel Landing')
grid on

% Von Mises SF - 4 Wheel
nexttile
plot(aoa,vonMisesSF4W,'LineWidth',2)
xlabel('Angle of Attack (deg)')
ylabel('Safety Factor')
title('Von Mises Safety Factor - 4 Wheel Landing')
grid on

% Tresca SF - 1 Wheel
nexttile
plot(aoa,trescaSF1W,'LineWidth',2)
xlabel('Angle of Attack (deg)')
ylabel('Safety Factor')
title('Tresca Safety Factor - 1 Wheel Landing')
grid on

% Tresca SF - 4 Wheel
nexttile
plot(aoa,trescaSF4W,'LineWidth',2)
xlabel('Angle of Attack (deg)')
ylabel('Safety Factor')
title('Tresca Safety Factor - 4 Wheel Landing')
grid on

% Strut Buckling SF - 1 Wheel
nexttile
plot(aoa,buckleStrut1WSF,'LineWidth',2)
xlabel('Angle of Attack (deg)')
ylabel('Safety Factor')
title('Strut Buckling Safety Factor - 1 Wheel Landing')
grid on

% Strut Buckling SF - 4 Wheel
nexttile
plot(aoa,buckleStrut4WSF,'LineWidth',2)
xlabel('Angle of Attack (deg)')
ylabel('Safety Factor')
title('Strut Buckling Safety Factor - 4 Wheel Landing')
grid on

% Stay Buckling SF - 1 Wheel
nexttile
plot(aoa,buckleStay1WSF,'LineWidth',2)
xlabel('Angle of Attack (deg)')
ylabel('Safety Factor')
title('Side Stay Buckling Safety Factor - 1 Wheel Landing')
grid on

% Stay Buckling SF - 4 Wheel
nexttile
plot(aoa, buckleStay4WSF, 'LineWidth', 2)
xlabel('Angle of Attack (deg)')
ylabel('Safety Factor')
title('Side Stay Buckling Safety Factor - 4 Wheel Landing')
grid on

%% Minimum Bolt Diameter
fig2 = figure;
plot(aoa, d_min_all, 'LineWidth', 2)
xlabel('Angle of Attack (deg)')
ylabel('Minimum Bolt Diameter')
title('Minimum Bolt Diameter vs Angle of Attack')
grid on

legendStrings = strings(1,length(sFactors));
for k = 1:length(sFactors)
    legendStrings(k) = "SF = " + num2str(sFactors(k));
end
legend(legendStrings, 'Location', 'best')

%% Stress Comparison: 1-Wheel vs 4-Wheel Landing
fig3 = figure('Position', [100, 100, 1100, 500]);
tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact')

% bending stress comparison
nexttile
plot(aoa, sigmaBend1W/1e6, '-o', 'LineWidth', 2, 'MarkerSize', 5, 'Color', [0.85, 0.33, 0.10])
hold on
plot(aoa, sigmaBend4W/1e6, '-s', 'LineWidth', 2, 'MarkerSize', 5, 'Color', [0.00, 0.45, 0.74])
yline(SigmaY/1e6, '--r', 'LineWidth', 1.5)
hold off
xlabel('Angle of Attack (deg)')
ylabel('Stress (MPa)')
title('Bending Stress')
legend('1-Wheel', '4-Wheel', 'Yield Strength', 'Location', 'best')
grid on

% von mises comparison
nexttile
plot(aoa, vonMises1W/1e6, '-o', 'LineWidth', 2, 'MarkerSize', 5, 'Color', [0.85, 0.33, 0.10])
hold on
plot(aoa, vonMises4W/1e6, '-s', 'LineWidth', 2, 'MarkerSize', 5, 'Color', [0.00, 0.45, 0.74])
yline(SigmaY/1e6, '--r', 'LineWidth', 1.5)
hold off
xlabel('Angle of Attack (deg)')
ylabel('Stress (MPa)')
title('Von Mises Equivalent Stress')
legend('1-Wheel', '4-Wheel', 'Yield Strength', 'Location', 'best')
grid on

% tresca comparison
nexttile
plot(aoa, tresca1W/1e6, '-o', 'LineWidth', 2, 'MarkerSize', 5, 'Color', [0.85, 0.33, 0.10])
hold on
plot(aoa, tresca4W/1e6, '-s', 'LineWidth', 2, 'MarkerSize', 5, 'Color', [0.00, 0.45, 0.74])
yline(SigmaY/2/1e6, '--r', 'LineWidth', 1.5)
hold off
xlabel('Angle of Attack (deg)')
ylabel('Stress (MPa)')
title('Tresca Shear Stress')
legend('1-Wheel', '4-Wheel', 'Yield/2', 'Location', 'best')
grid on

sgtitle('Stress Comparison Across Landing Scenarios', 'FontSize', 14, 'FontWeight', 'bold')

%% Axle Stress Breakdown - Bar Chart at Worst Case (alpha = 0)
fig4 = figure('Position', [100, 100, 900, 500]);

stressLabels = categorical({'Bending', 'Transverse Shear', 'Torsional Shear'});
stressLabels = reordercats(stressLabels, {'Bending', 'Transverse Shear', 'Torsional Shear'});
stressVals = [sigmaBend1W(1)/1e6, sigmaBend4W(1)/1e6;
              taoShear1W(1)/1e6,  taoShear4W(1)/1e6;
              taoTorsion1W(1)/1e6, taoTorsion4W(1)/1e6];

b = bar(stressLabels, stressVals, 'grouped');
b(1).FaceColor = [0.85, 0.33, 0.10];
b(2).FaceColor = [0.00, 0.45, 0.74];
ylabel('Stress (MPa)')
title('Axle Stress Breakdown at \alpha = 0° (Worst Case)', 'FontSize', 13)
legend('1-Wheel Landing', '4-Wheel Landing', 'Location', 'northwest')
grid on

% add value labels on bars
for k = 1:2
    xtips = b(k).XEndPoints;
    ytips = b(k).YEndPoints;
    labels = compose("%.1f", b(k).YData);
    text(xtips, ytips, labels, 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'bottom', 'FontSize', 9, 'FontWeight', 'bold')
end

%% Safety Factor Summary - All Failure Modes at alpha = 0
fig5 = figure('Position', [100, 100, 1000, 550]);

sfLabels = categorical({'Von Mises', 'Tresca', 'Fatigue', 'Strut Buckling', 'Stay Buckling'});
sfLabels = reordercats(sfLabels, {'Von Mises', 'Tresca', 'Fatigue', 'Strut Buckling', 'Stay Buckling'});

sf1W = [vonMisesSF1W(1), trescaSF1W(1), NaN, buckleStrut1WSF(1), buckleStay1WSF(1)];
sf4W = [vonMisesSF4W(1), trescaSF4W(1), fatigueSF(1), buckleStrut4WSF(1), buckleStay4WSF(1)];

b = bar(sfLabels, [sf1W; sf4W]', 'grouped');
b(1).FaceColor = [0.85, 0.33, 0.10];
b(2).FaceColor = [0.00, 0.45, 0.74];
hold on
yline(1, '--r', 'SF = 1', 'LineWidth', 2, 'FontSize', 11, 'LabelHorizontalAlignment', 'left')
yline(1.5, ':k', 'SF = 1.5 (Typical Design Target)', 'LineWidth', 1.5, 'FontSize', 9, ...
    'LabelHorizontalAlignment', 'left')
hold off
ylabel('Safety Factor')
title('Safety Factor Summary at \alpha = 0° — All Failure Modes', 'FontSize', 13)
legend('1-Wheel Landing', '4-Wheel Landing', 'Location', 'best')
grid on

% add value labels
for k = 1:2
    xtips = b(k).XEndPoints;
    ytips = b(k).YEndPoints;
    labels = compose("%.2f", b(k).YData);
    text(xtips, ytips, labels, 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'bottom', 'FontSize', 9, 'FontWeight', 'bold')
end

%% Safety Factor Heatmap - All AoA vs Failure Modes (4-Wheel)
fig6 = figure('Position', [100, 100, 900, 450]);

sfMatrix4W = [vonMisesSF4W; trescaSF4W; fatigueSF; buckleStrut4WSF; buckleStay4WSF];
modeLabels = {'Von Mises', 'Tresca', 'Fatigue', 'Strut Buckling', 'Stay Buckling'};

imagesc(aoa, 1:5, sfMatrix4W)
colormap(flipud(hot))
cb = colorbar;
cb.Label.String = 'Safety Factor';
cb.Label.FontSize = 11;
set(gca, 'YTick', 1:5, 'YTickLabel', modeLabels)
xlabel('Angle of Attack (deg)')
title('Safety Factor Heatmap — 4-Wheel Landing', 'FontSize', 13)

% overlay text values
for row = 1:5
    for col = 1:length(aoa)
        val = sfMatrix4W(row, col);
        if val < 2
            txtColor = 'w';
        else
            txtColor = 'k';
        end
        text(aoa(col), row, sprintf('%.1f', val), ...
            'HorizontalAlignment', 'center', 'FontSize', 8, ...
            'FontWeight', 'bold', 'Color', txtColor)
    end
end

%% Normal Force vs Angle of Attack
fig7 = figure('Position', [100, 100, 700, 450]);

plot(aoa, normal_force/1e3, '-o', 'LineWidth', 2.5, 'MarkerSize', 6, ...
    'Color', [0.47, 0.67, 0.19], 'MarkerFaceColor', [0.47, 0.67, 0.19])
xlabel('Angle of Attack (deg)')
ylabel('Normal Force (kN)')
title('Landing Gear Normal Force vs Angle of Attack', 'FontSize', 13)
grid on

% shade the region where lift increases
hold on
yyaxis right
liftVals = 0.5*Rho*(V_1)^2*Cl_max*Sw*cosd(aoa);
area(aoa, liftVals/1e3, 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'FaceColor', [0.30, 0.75, 0.93])
ylabel('Lift Component (kN)')
hold off
legend('Normal Force on Gear', 'Vertical Lift Component', 'Location', 'best')
