function [ F, rdot, mfdot, mdot, P1, Isp, CF, c ] ...
        = hybridRocketThrustCalc( ...
            g, rhof, regress_fun, rsplinevec, cstarsplinedata, ...
            ksplinedata, eta, At, A2, Ap, Ab, modot, P3 )
% Computes the thrust of a hybrid rocket motor
% Computes the thrust of a hybrid rocket motor given the ambient
% pressure, oxidizer flow rate, physical design of the nozzle, etc.
% 
% Areas and anonymous functions for Oxidizer Mass Flux Rates are passed as 
% arguements to this function in order to keep it flexible to account for
% any fuel grain geometry.
% 
% @arg
% g               - double
%                   Gravity magnitude
% rhof            - double
%                   Fuel mass density
% regress_fun     - Anonymous Function
%                   Regression Rate as a function of Oxidizer Mass Velocity
% rsplinevec      - Nr x 1 double vector
%                   Fuel/Oxidizer Ratio vector for spline interpolation of 
%                   cstar and specific heat ratio.
% cstarsplinedata - Nr x 1 double vector
%                   cstar data vector as a function of Fuel/Oxidizer ratio
%                   for spline interpolation.
% ksplinedata     - Nr x 1 double vector
%                   Specific heat ratio data vector as a function of 
%                   Fuel/Oxidizer ratio
%                   for spline interpolation.
% eta             - double
%                   cstar efficiency value, scale 0.0 to 1.0
% At              - double
%                   Nozzle throat area
% A2              - double
%                   Nozzle exit area
% Ap              - double
%                   Total combustion port area
% Ab              - double
%                   Exposed fuel surface area
% modot           - double
%                   Oxidizer mass flow rate
% P3              - double
%                   Ambient Pressure
% 
% @return
% F               - double
%                   Thrust
% rdot            - double
%                   Fuel Regression Rate
% mfdot           - double
%                   Fuel mass rate of change
% mdot            - double
%                   Mass Flow Rate
% P1              - double
%                   Combustion Chamber Pressure
% Isp             - double
%                   Specific Impulse
% CF              - double
%                   Thrust coefficient
% c               - double
%                   Exhaust Velocity
% 
% @author: Matt Marti
% @date: 2019-04-25

% If there is no oxidizer mass flow rate, thrust is 0
if ~modot
    F = 0;
    rdot = 0;
    mfdot = 0;
    mdot = 0;
    P1 = P3;
    Isp = NaN;
    CF = NaN;
    c = 0;
    return;
end

% Oxidizer mass velocity
Go = modot / Ap; % Oxidizer Mass Velocity

% Determine Mass Flow Rates
rdot = regress_fun(Go);

% Determine Fuel mass burn rate
mfdot = rhof * Ab * rdot; % Eq 16-11

% Mixing ratio
r = modot / mfdot;

% Determine characteristic velocity and specific heats using cubic spline
cstar_theory = cubicspline(rsplinevec, cstarsplinedata, r); % [ft/s]
cstar = eta * cstar_theory;
k = cubicspline(rsplinevec, ksplinedata, r); % [ft/s]

% Determine combustion chamber pressure
mdot = mfdot + modot;
P1 = mdot * cstar / At; % Sutton 16-8

% Determine Exit Mach number
A2oAt = A2 / At; % Sutton 3-19
A2oAt_fun = @(M) 1./M.*(2*(1+0.5*(k-1)*M.^2)/(k+1)).^(0.5*(k+1)/(k-1)); % Sutton 3-14
Mach_error_fun = @(M) log(A2oAt_fun(M)) - log(A2oAt); % Error function
M2 = secantrootsolve( Mach_error_fun, 1, 5); % Exit Mach No.

% Compute Stagnation Pressure ratio (Stagnation Pressure P0 ~= P1)
P1oP2 = (1+0.5*(k-1)*M2.^2).^(k/(k-1)); % Sutton 3-13

% Determine Thrust coefficient
P2 = P1 / P1oP2; % Exit pressure
P2oP1 = P2/P1;
CF = sqrt(((2*k^2)/(k-1))*(2/(k+1))^((k+1)/(k-1))*(1-(P2oP1)^((k-1)/k))) ...
    + (P2-P3)/P1*A2oAt; % Thrust coefficient

% Compute thrust
F = At * CF * P1; % Thrust

% Determine Specific impluse
Isp = CF*cstar / g;

% Exhaust velocity
c = F / mdot;

end