function [ Xk, Pk ] = EKF( x, P, u, z, R, Q1, Q2, T, S1, S2, S3, k )
%   Discrete Extended Kalman Filter Algorithm
%   As seen on Probabilistic Robotics by Sebastian Thrun, Cap. 3, pp. 59
    
    % Prediction
    % Next states with euler-forward discretization (predicted mean)
    f = [x(2);
         ( u(1)*cos(x(5)) - u(2)*sin(x(5)) );
         x(4);
         ( u(1)*sin(x(5)) + u(2)*cos(x(5)) );
         u(3)];
    xb = x + T*f;           
    
    A = [1 T 0 0                0                       % States Jacobian matrix
         0 1 0 0 T*(-u(1)*sin(x(5)) - u(2)*cos(x(5)))
         0 0 1 T                0
         0 0 0 1 T*(u(1)*cos(x(5)) - u(2)*sin(x(5)))
         0 0 0 0                1];
    Pb = A*P*A' + R;        % Predicted covariance
    
    % Update
    d1 = (xb(1)-S1(1))^2 + (xb(3)-S1(2))^2;
    d2 = (xb(1)-S2(1))^2 + (xb(3)-S2(2))^2;
    d3 = (xb(1)-S3(1))^2 + (xb(3)-S3(2))^2;
    
    h1 = 20*log10(4*pi*sqrt(d1)*2.4e9/299792458);
    h2 = 20*log10(4*pi*sqrt(d2)*2.4e9/299792458);
    h3 = 20*log10(4*pi*sqrt(d3)*2.4e9/299792458);
    dh1x1 = 20*(xb(1)-S1(1))/( log(10)*(d1) );
    dh2x1 = 20*(xb(1)-S2(1))/( log(10)*(d2) );
    dh3x1 = 20*(xb(1)-S3(1))/( log(10)*(d3) );
    dh1x3 = 20*(xb(3)-S1(2))/( log(10)*(d1) );
    dh2x3 = 20*(xb(3)-S2(2))/( log(10)*(d2) );
    dh3x3 = 20*(xb(3)-S3(2))/( log(10)*(d3) );
    
    h4 = xb(2);
    h5 = xb(4);
    
    if mod(k,5) == 0 && ~isnan(z(6)) && ~isnan(z(7))
        h6 = xb(1);
        h7 = xb(3);
        
        h = [h1 h2 h3 h4 h5 h6 h7]';
        H = [dh1x1 0 dh1x3 0 0    % Measurements Jacobian matrix
             dh2x1 0 dh2x3 0 0
             dh3x1 0 dh3x3 0 0
             0     1   0   0 0
             0     0   0   1 0
             1     0   0   0 0
             0     0   1   0 0];

        Kk = Pb*H'*(H*Pb*H' + Q2)^-1;    % Kalman Gain
        Xk = xb + Kk*(z-h);             % Corrected mean
        Pk = (eye(length(Xk))-Kk*H)*Pb; % Corrected covariance
    else
        z = z(1:end-2);
        
        h = [h1 h2 h3 h4 h5]';    % Measurements function vector
        H = [dh1x1 0 dh1x3 0 0    % Measurements Jacobian matrix
             dh2x1 0 dh2x3 0 0
             dh3x1 0 dh3x3 0 0
             0     1   0   0 0
             0     0   0   1 0];

        Kk = Pb*H'*(H*Pb*H' + Q1)^-1;    % Kalman Gain
        Xk = xb + Kk*(z-h);             % Corrected mean
        Pk = (eye(length(Xk))-Kk*H)*Pb; % Corrected covariance
    end
    
end

