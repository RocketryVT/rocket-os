function [ xdotk, data ] = fHRFSdynamics_1D( tk, xk, p, modot, ...
    rsplinevec, cstarsplinedata, ksplinedata, atm )
% State transition function for modeling the flight of a Hybrid Rocket
% This function computes the nonlinear partial differential equation:
% 
%     xdot_k = f( t_k, x_k, p )
% 
% for the flight dynamics of a Hybrid Rocket. This function only models a
% rocket flight in one spatial dimension: Altitude. Eventually, this
% functino will be adapted to model the three-dimensional flight of a
% hybrid rocket for Rocketry @ Virginia Tech and the Intercollegeiate
% Rocket Engineering Competition (IREC).
% 
% Computes the acceleration and fuel regression rate of a hybrid rocket
% during flight. This function acts as the time dependent partial 
% differential equation which describes the flight of a Hybrid Rocket
% 
% @arg
% tk              - double
%                   Current time
% xk              - Nx x 1 double vector
%                   Rocket state at time tk. This is the vectorized form of
%                   rocket state which has the contents:
%                       xk = [ posk velk db mk ]
% p               - Struct
%                   Rocket parameters struct. Contents are as follows:
%                   - rhof        - double
%                                   Fuel density. For Thrust calculation
%                   - regress - Anonymous Function
%                                   Fuel regression rate function. For
%                                   Thrust calculation
%                   - eta         - double
%                                   Fuel burn efficiency. For Thrust
%                                   calculation
%                   - At          - double
%                                   Nozzle Throat Area. For Thrust
%                                   calculation
%                   - A2          - double
%                                   Nozzle Exit Area. For Thrust
%                                   calculation
%                   - Ap      - Anonymous Function
%                                   Fuel grain total port area. For Thrust
%                                   calculation. Must accept only "db"
%                                   (Fuel Grain width) as input:
%                                       Ap = Ap(db)
%                                   calculation
%                   - Ab      - Anonymous Function
%                                   Fuel grain exposed burn area. For
%                                   Thrust calculation. Must accept only 
%                                   "db" (Fuel Grain width) as input:
%                                       Ap = Ap(db)
%                   - Cd          - double
%                                   Drag Coefficient. For Drag calculation
% modot       - Anonymous Function
%                   Oxidizer Mass Flow rate as a function of time
% rsplinevec      - Nr x 1 double vector
%                   Fuel/Oxidizer ratio vector for spline interpolation.
%                   For Thrust calculation
% cstarsplinedata - Nr x 1 double vector
%                   cstar vector for spline interpolation. For Thrust 
%                   calculation
% ksplinedata     - Nr x 1 double vector
%                   Specific heat ratio vector for spline interpolation.
%                   For Thrust calculation
% atm         - Anonymous Function
%                   Atmospheric pressure and density as a function of 
%                   altitude. This function must accept the altitude as an
%                   argument and return at least the Pressure and Density
%                   value in the form:
%                       [ P, rho ] = atm( hk )
% rho         - Anonymous Function
%                   Atmospheric density as a function of altitude
% 
% @return
% xdotk           - Nx x 1 double vector
%                   Rocket state time rate of change
%                   State components are: 
%                       xdotk = [ vk; ak; rdotk; modotk; mfdotk; mdotk ]
% data            - Nx x 1 double vector
%                   Important data for analysis
% 
% @author: Matt Marti
% @date: 2019-04-25

% Parse state vector for rocket state values
posk = xk(1); % Rocket position
velk = xk(2); % Rocket acceleration
db = xk(3); % Fuel grain width
mok = xk(4); % Oxidizer mass
mfk = xk(5); % Fuel mass
mk = xk(6); % Rocket total mass

% Parse Parameter struct for rocket parameter values
rhof = p.rhof; % Fuel density. For Thrust calculation
regress_fun = p.regress_fun; % Fuel regression rate function. For Thrust calculation
eta = p.eta; % Fuel burn efficiency. For Thrust calculation
At = p.At; % Nozzle Throat Area. For Thrust calculation
A2 = p.A2; % Nozzle Exit Area. For Thrust calculation
Ap_fun = p.Ap_fun; % Fuel grain total port area. For Thrust calculation
Ab_fun = p.Ab_fun; % Fuel grain exposed burn area. For Thrust calculation
Af = p.Af; % Frontal area
Cd = p.Cd; % Drag Coefficient. For Drag calculation

% Compute altitude and atmospheric parameters
hk = posk; % Altitude
[P3, rho] = atm(hk); % Atmosphere pressure and density

% Compute Accelerations caused by external forces and rocket state
g = 9.807; % Scalar gavity acceleration

% Compute exposed fuel area, port area, and Oxidizer Mass Flow Rate
modotk = modot(tk);
Ap_fun = Ap_fun(db); % Total Port radius
Ab_fun = Ab_fun(db); % Total exposed fuel burn surface

% Only compute a thrust value if the fuel mass is non-zero
err = 3.5; % Minimum amount of fuel to compute thrust.
if mfk > err && mok > err
    [ F, rdotk, mfdotk, mdotk, P1, Isp ] ...
            = hybridRocketThrustCalc( ...
                g, rhof, regress_fun, rsplinevec, cstarsplinedata, ...
                ksplinedata, eta, At, A2, Ap_fun, Ab_fun, modotk, P3 );
else % Empty tank condition
    F = 0;
    rdotk = 0;
    mfdotk = 0;
    if mok <= err % If oxidizer tank is empty
        modotk = 0;
    end
    mdotk = modotk + mfdotk;
    P1 = 0;
    Isp = NaN;
end

% Compute Drag Force from Aerodynamics model
[D, q] = dragCalc(Cd, Af, rho, velk);

% Acceleration is the sum of the forces divided by mass
acck = (F + D)/mk - g; % Acceleration

% Compile state rate of change values into one variable
xdotk = [velk; acck; - rdotk; -modotk; - mfdotk; - mdotk];

% Compile important analysis data
data = [P1; F; D; q; Isp];

end

