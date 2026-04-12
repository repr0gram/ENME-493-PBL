function sol = solveLandingGear(F1x, F1y, F1z, M1z, M1y, ...
    u5x, u5y, u5z, u7x, u7y, u7z, ...
    L_strut, L3, L7, z7, zb1, L_side_stay_upper, ...
    theta3_xy, theta3_yz, theta3_xz)
% solveLandingGear  numeric replacement for the symbolic 15x17 system.
% variables: [F3x F3y F3z F7 Fb1x Fb1y Fb1z Fb2x Fb2y Fb2z
%             Fb3x Fb3y Fb3z F6x F6y F6z F5]

    s3xy = sin(theta3_xy); c3xy = cos(theta3_xy);
    s3yz = sin(theta3_yz); c3yz = cos(theta3_yz);
    s3xz = sin(theta3_xz); c3xz = cos(theta3_xz);
    Ls = L_side_stay_upper;

    A = zeros(15, 17);
    B = zeros(15, 1);

    % eqn1: F3x + F7*u7x + Fb1x + Fb2x = F1x
    A(1,[1 4 5 8]) = [1, u7x, 1, 1];
    B(1) = F1x;

    % eqn2: F3y + F7*u7y + Fb1y + Fb2y = F1y
    A(2,[2 4 6 9]) = [1, u7y, 1, 1];
    B(2) = F1y;

    % eqn3: F3z + F7*u7z + Fb1z + Fb2z = F1z
    A(3,[3 4 7 10]) = [1, u7z, 1, 1];
    B(3) = F1z;

    % eqn4: (Ls-L3)*F3x + (Ls-L7)*u7x*F7 = M1z + L_strut*F1x
    A(4,[1 4]) = [(L_strut - L3), (L_strut - L7)*u7x];
    B(4) = M1z + L_strut*F1x;

    % eqn5: -L3*F3z + (-L7*u7z+z7*u7y)*F7 + zb1*Fb1y - L_strut*Fb1z - L_strut*Fb2z = 0
    A(5,[3 4 6 7 10]) = [-L3, (-L7*u7z + z7*u7y), zb1, -L_strut, -L_strut];
    B(5) = 0;

    % eqn6: z7*u7x*F7 + zb1*Fb1x = M1y
    A(6,[4 5]) = [z7*u7x, zb1];
    B(6) = M1y;

    % eqn7: -F3x + Fb3x + F6x + u5x*F5 = 0
    A(7,[1 11 14 17]) = [-1, 1, 1, u5x];

    % eqn8: -F3y + Fb3y + F6y + u5y*F5 = 0
    A(8,[2 12 15 17]) = [-1, 1, 1, u5y];

    % eqn9: -F3z + Fb3z + F6z + u5z*F5 = 0
    A(9,[3 13 16 17]) = [-1, 1, 1, u5z];

    % eqn10: -Ls*c3xy*Fb3x - Ls*s3xy*Fb3y - Ls*c3xy*F6x - Ls*s3xy*u5y*F5 = 0
    A(10,[11 12 14 17]) = [-Ls*c3xy, -Ls*s3xy, -Ls*c3xy, -Ls*s3xy*u5y];

    % eqn11: -Ls*s3yz*Fb3y - Ls*c3yz*Fb3z + (-Ls*s3yz*u5y - Ls*c3yz*u5z)*F5 = 0
    A(11,[12 13 17]) = [-Ls*s3yz, -Ls*c3yz, (-Ls*s3yz*u5y - Ls*c3yz*u5z)];

    % eqn12: -Ls*c3xz*Fb3x - Ls*s3xz*Fb3z + (-Ls*s3xz*u5z - Ls*c3xz*u5x)*F5 = 0
    A(12,[11 13 17]) = [-Ls*c3xz, -Ls*s3xz, (-Ls*s3xz*u5z - Ls*c3xz*u5x)];

    % eqn13: -u7x*F7 - F6x - u5x*F5 = 0
    A(13,[4 14 17]) = [-u7x, -1, -u5x];

    % eqn14: -u7y*F7 - F6y - u5y*F5 = 0
    A(14,[4 15 17]) = [-u7y, -1, -u5y];

    % eqn15: -u7z*F7 - F6z - u5z*F5 = 0
    A(15,[4 16 17]) = [-u7z, -1, -u5z];

    sol = A \ B;
end
