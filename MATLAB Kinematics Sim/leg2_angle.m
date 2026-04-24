function angle2 = leg2_angle(n, h, L, L1, L2, L3)
        % Spherical Coordinates
        a2 = (L/(sqrt(n(1)^2+3*n(2)^2+4*n(3)^2+2*sqrt(3)*n(1)*n(2))))*(-n(3));
        b2 = (L/(sqrt(n(1)^2+3*n(2)^2+4*n(3)^2 +2*sqrt(3)*n(1)*n(2))))*(-sqrt(3)*n(3));
        c2 = h + (L/(sqrt(n(1)^2+3*n(2)^2+4*n(3)^2+2*sqrt(3)*n(1)*n(2))))*(sqrt(3)*n(2)+n(1));
        B_m = [a2, b2, c2];
        % Define Grouped Constants
        A2 = -(B_m(1)+sqrt(3)*B_m(2)+2*L3)/B_m(3);
        B2 = (B_m(1)^2+B_m(2)^2+B_m(3)^2+L2^2-L1^2-L3^2)/(2*B_m(3));
        C2 = A2^2+4;
        D2 = 2*A2*B2+4*L3;
        E2 = B2^2+L3^2-L2^2;
        % Revolute Coordinates
        x2 = (-D2-sqrt(D2^2-4*C2*E2))/(2*C2);
        y2 = sqrt(3)*x2;
        z2 = sqrt(L2^2-4*x2^2-4*L3*x2-L3^2);
        B_2 = [x2, y2, z2];
        % Calculate Motor Angle
        angle2 = 90 - rad2deg(atan2(sqrt(B_2(1)^2+B_2(2)^2)-L3, B_2(3)));
end


