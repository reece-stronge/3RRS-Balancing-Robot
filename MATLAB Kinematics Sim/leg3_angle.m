function angle3 = leg3_angle(n, h, L, L1, L2, L3)
        % Spherical Coordinates
        a3 = (L/(sqrt(n(1)^2+3*n(2)^2+4*n(3)^2-2*sqrt(3)*n(1)*n(2))))*(-n(3));
        b3 = (L/(sqrt(n(1)^2+3*n(2)^2+4*n(3)^2 - 2*sqrt(3)*n(1)*n(2))))*(sqrt(3)*n(3));
        c3 = h + (L/(sqrt(n(1)^2+3*n(2)^2+4*n(3) ^2-2*sqrt(3)*n(1)*n(2))))*(-sqrt(3)*n(2)+n(1));
        C_m = [a3, b3, c3];
        % Define Grouped Constants
        A3 = -(C_m(1)-sqrt(3)*C_m(2)+2*L3)/C_m(3);
        B3 = (C_m(1)^2+C_m(2)^2+C_m(3)^2+L2^2-L1^2-L3^2)/(2*C_m(3));
        C3 = A3^2+4;
        D3 = 2*A3*B3+4*L3;
        E3 = B3^2+L3^2-L2^2;
        % Revolute Coordinates
        x3 = (-D3-sqrt(D3^2-4*C3*E3))/(2*C3);
        y3 = -sqrt(3)*x3;
        z3 = sqrt(L2^2-4*x3^2-4*L3*x3-L3^2);
        C_2 = [x3, y3, z3];
        % Calculate Motor Angle
        angle3 = 90 - rad2deg(atan2(sqrt(C_2(1)^2+C_2(2)^2)-L3, C_2(3)));
end