function angle3 = leg3_angle(n, h, L, L1, L2, L3)
        % Spherical Coordinates
        a3 = (L/(sqrt(n(1)^2+3*n(2)^2+4*n(3)^2-2*sqrt(3)*n(1)*n(2))))*(-n(3));
        b3 = (L/(sqrt(n(1)^2+3*n(2)^2+4*n(3)^2 - 2*sqrt(3)*n(1)*n(2))))*(sqrt(3)*n(3));
        c3 = h + (L/(sqrt(n(1)^2+3*n(2)^2+4*n(3) ^2-2*sqrt(3)*n(1)*n(2))))*(-sqrt(3)*n(2)+n(1));
        S3 = [a3, b3, c3];
        % Define Grouped Constants
        A3 = -(S3(1)-sqrt(3)*S3(2)+2*L3)/S3(3);
        B3 = (S3(1)^2+S3(2)^2+S3(3)^2+L2^2-L1^2-L3^2)/(2*S3(3));
        C3 = A3^2+4;
        D3 = 2*A3*B3+4*L3;
        E3 = B3^2+L3^2-L2^2;
        % Revolute Coordinates
        x3 = (-D3-sqrt(D3^2-4*C3*E3))/(2*C3);
        y3 = -sqrt(3)*x3;
        z3 = sqrt(L2^2-4*x3^2-4*L3*x3-L3^2);
        P3 = [x3, y3, z3];
        % Calculate Motor Angle
        angle3 = 90 - rad2deg(atan2(sqrt(P3(1)^2+P3(2)^2)-L3, P3(3)));
end
