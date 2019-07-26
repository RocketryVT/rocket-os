function [P, rho, T] = stdAtmosphereCalc_hgp(h, RE, g0)
% Computes the Temperature, Pressure, and Density of the Atmosphere
% Uses the Geometric Alitude to compute the Temperature, Pressure, and
% Density of the atmosphere according to the Standard Atmosphere Model
% presented in John D. Anderson Jr. "Introduction to Flight", c 2016.
% 
% This atmospheric model uses a gravity model that assumes a spherical,
% non-rotating Earth. It uses the solutions to the following differential
% equation from Anderson as the Standard Atmosphere Model:
% 
%     dp/p = - g / ( R * T(h) ) * dh
% 
% to solve for Pressure at a specific altitude. In this version of the
% Standard Atmospheric calculations, the Geopotential Altitude is used to
% simplify the integration.
% 
% @arg
% h   - double
%       Altitude
% RE  - double
%       Planetary Radius
% g0  - double
%       Sea level gravity (gravity at R)
% 
% @return
% P   - double
%       Pressure at altitude
% rho - double
%       Density at altitude
% T   - double
%       Temperature at altitude
% 
% @author: Matt Marti
% @date: 2019-04-25

% Define global variables (Gas Constant)
global R
if ~numel(R)
    constants_HRFS;
end

% Compute geopotential altitude
hgp = h * RE / (RE + h);

% Initial conditions for temperature
him1 = 0;
Tim1 = 288.16;
Pim1 = 1.01325e5;

% Standard atmposhere Temperature gradient and associated altitudes
avec = [ -6.5e-3; 0; 3e-3; 0; -4.5e-3; 0; 4e-3; 0 ];
hgpavec = [ -RE; 11e3; 25e3; 47e3; 53e3; 79e3; 90e3; 105e3; inf ];

% Iterate to given altitude
i = 1;
while hgp > hgpavec(i)
    hi = min(hgp, hgpavec(i+1));
    ai = avec(i);
    deltah = hi - him1;
    Ti = Tim1 + ai * deltah;
    if ai % Gradient Temperature Layer
        TioTim1 = Ti / Tim1;
        Pi = Pim1 * TioTim1^(-g0/(ai*R));
    else % Isothermal layer
        Pi = Pim1 * exp(-g0/(R*Ti)*deltah);
    end
    i = i + 1;
    him1 = hi;
    Pim1 = Pi;
    Tim1 = Ti;
end

% Assign output
T = Ti;
P = Pi;
rho = P / (R * T);

end

