function angle1 = leg1_angle(n, h, L, L1, L2, L3)
        % Spherical Coordinates
        a1 = (L/(sqrt(n(1)^2 + n(3)^2)))*(n(3));
        b1 = 0;
        c1 = h + (L/(sqrt(n(1)^2 + n(3)^2)))*(-n(1));
        A_m = [a1, b1, c1];
        
        % Define Grouped Constants
        A = (L3-A_m(1))/A_m(3);
        B = (A_m(1)^2+A_m(2)^2+A_m(3)^2-L1^2-L3^2+L2^2)/(2*A_m(3));
        C = A^2+1;
        D = 2*(A*B-L3);
        E = B^2+L3^2-L2^2;

        % Revolute Coordinates
        x1 = (-D+sqrt(D^2-4*C*E))/(2*C);
        y1 = 0;
        z1 = sqrt(L2^2-x1^2+2*L3*x1-L3^2);
        A_2 = [x1, y1, z1];

        % Calculate Motor Angle
        angle1 = 90 - rad2deg(atan2(A_2(1)-L3, A_2(3)));
end




