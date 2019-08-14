%% main_HRFS.m
% 
% Driver script for the Hybrid Rocket Flight Simulation.
% 
% This function simulates the flight of a hybrid rocket to determine the
% maximum altitude that can be achieved with the given parameters. This
% suite is the project for Virginia Tech AOE 4404: Numerical Methods.
% 
% Eventually (hopefully) this MATLAB suite will be converted to C++ to
% predict the altitude of the rocket for Rocketry@VT.
% 
% @author: Matt Marti
% @date: 2019-05-07

clear, clc, clear global
include_HRFS('.');
constants_HRFS;


%% Parameters

% Integration parameters
t0 = 0;
tend = 1500; % Maximum time of simulation
dt = 1e-1; % Simulation delta time step
nRK = 6; % Runge-Kutta integration number of steps

% Differentiation parameters
nFD = 1; % Order of finite difference differentiation

% Dynamic Pressure Root Search Parameters
t0_maxq_1 = t0; % Ascent Start
tend_maxq_1 = t0 + 100; % Ascent End
ta_maxq_1 = t0_maxq_1 + 20;
tb_maxq_1 = tend_maxq_1 - 40;

% Rocket initial position and velocity conditions
pos0 = 1400; % [m] Altitude, Spaceport America, New Mexico
vel0 = 0; % [m/s] Initial velocity

% Oxidizer Mass Flow profile
modot_fun = @(tk) 3096.55536595829;

% Rocket Mass
me = 5100; % [kg] Empty mass
mo0 = 3.7159e+05; % [kg] Oxidizer mass
% mf = 0; % [kg] Fuel mass (Calculated from fuel density and geometry)
m_error_margin = mo0*1e-4;

% Propellant Characteristics for HTPB/LOX
etacombust = 0.95; % Combustion efficiency
rhof = 913.436110336788; % [kg/m^3] Fuel density (Thrust)
a = 3.045015375e-05; % Fuel grain regression coefficient (Thrust)
n = 0.680825577; % Fuel grain regression exponent (Thrust)
regress_fun = @(Go) a*(Go^n);
rsplinevec = (1:.2:3)'; % Mass Mixture Ratio spline data
cstarsplinedata = [
    4825;
    5180;
    5543;
    5767;
    5882;
    5912;
    5885;
    5831;
    5768;
    5703;
    5639] * 0.3048; % [m/s] Characteristic velocity spline data
ksplinedata = [
    1.308;
    1.282;
    1.239;
    1.201;
    1.171;
    1.152;
    1.143;
    1.138;
    1.135;
    1.133;
    1.132]; % [no units] Specific heat ratio spline data

% Fuel Grain Design
Nt = 7; % Number of fuel grain ports
L = 30.2371738789983; % [m] Fuel grain length
R0 = 0.363557686863076; % [m] Initial Port radius
db0 = 0.271442313136924; % [m] Initial Fuel Grain Width
Ap_fun = @(db) pi * Nt * (R0+db0-db)^2; % Port area function
Ab_fun = @(db) Nt * 2 * (R0+db0-db) * pi * L; % Burn area function
mf0 = rhof*Nt*L*pi*((db0+R0)^2 - db0^2);

% Nozzle Parameters
At = 1.64750631789099; % [m^2] Nozzle throat area (Thrust)
A2 = 12.7187487741184; % [m^2] Nozzle Exit area (Thrust)

% Areodynamic Parameters
Af = pi*(160/12*.3048)^2; % [m^2] Rocket front area (Drag)
Cd = 0.25; % CD for powered flight
RE = 6.356766e6; % [m] Earth Radius
g0 = 9.807; % [m/s/s] Earth sea level gravity acceleration
atm_fun = @(h) stdAtmosphereCalc_hgp(h, RE, g0);


%% Simulate flight

% Flight initial conditions
m0 = me + mo0 + mf0;
x0 = [ pos0; vel0; db0; mo0; mf0; m0 ];

% Compile parameter values
p = struct( ...
        'rhof', rhof, ... % Fuel density
        'regress_fun', regress_fun, ... % Fuel regression rate function
        'eta', etacombust, ... % Fuel/Oxidizer burn efficiency
        'At', At, ... % Throat Area
        'A2', A2, ... % Nozzle Exit Area
        'Ap_fun', Ap_fun, ... % Port area function
        'Ab_fun', Ab_fun, ... % Burn area function
        'Af', Af, ... % Frontal area
        'Cd', Cd); % Drag coefficient

% Differential equation to be solved
xdot_fun = @(tk, xk) fHRFSdynamics_1D(tk, xk, p, modot_fun, ...
    rsplinevec, cstarsplinedata, ksplinedata, atm_fun);

% Integrate using Runge-Kutta
tlims = [t0; tend];
[xhist, thist, xdothist] ...
    = rungekuttaint(xdot_fun, x0, tlims, dt, nRK);
Nt = length(thist);

% Parse integration output
hhist = xhist(1,:);
vhist = xhist(2,:);
ahist = xdothist(2,:);
dbhist = xhist(3,:);
mohist = xhist(4,:);
mfhist = xhist(5,:);
mhist = xhist(6,:);
rdothist = xdothist(3,:);
modothist = xdothist(4,:);
mfdothist = xdothist(5,:);
mdothist = xdothist(6,:);


%% Plots

% Altitude curve
figure(1); hold off
plot(thist, hhist, 'linewidth', 2);
title('Altitude Time History');
xlabel('Time [s]')
ylabel('Altitude [m]');
grid on, grid minor

% Velocity curve
figure(2); hold off
plot(thist, vhist, 'linewidth', 2);
title('Velocity Time History');
xlabel('Time [s]')
ylabel('Velocity [m/s]');
grid on, grid minor

% Acceleration curve
figure(3); hold off
plot(thist, ahist, 'linewidth', 2);
title('Acceleration Time History');
xlabel('Time [s]')
ylabel('Acceleration [m/s/s]');
grid on, grid minor

% Fuel grain width curve
figure(4); hold off
plot(thist, dbhist*1e2, 'linewidth', 2);
title('Fuel Grain Width Time History');
xlabel('Time [s]')
ylabel('Fuel Grain Width [cm]');
grid on, grid minor

% Fuel mass curve
figure(5); hold off
plot(thist, [mohist; mfhist], 'linewidth', 2);
title('HTPB/LOX Mass Time History');
xlabel('Time [s]')
ylabel('Mass [kg]');
legend({'LOX', 'HTPB'});
grid on, grid minor

% Fuel Grain Regression Rate curve
figure(6); hold off
plot(thist, rdothist*1e2, 'linewidth', 2);
title('Fuel Grain Regression Rate Time History');
xlabel('Time [s]')
ylabel('Regression Rate [cm/s]');
grid on, grid minor

% Mass flow rate curve
figure(7); hold off
plot(thist, [modothist; mfdothist; mdothist], 'linewidth', 2);
title('Mass Flow Rate Time History');
xlabel('Time [s]')
ylabel('Mass Flow Rate [kg/s]');
legend({'LOX', 'HTPB', 'Total'});
grid on, grid minor

% Mass flow ratio
figure(8); hold off
plot(thist, modothist ./ mfdothist, 'linewidth', 2);
title('Mass Flow Rate Ratio Time History');
xlabel('Time [s]')
ylabel('modot / mfdot');
grid on, grid minor

% Total Mass Time History
figure(9); hold off
plot(thist, mhist, 'linewidth', 2);
title('Total Mass Time History');
xlabel('Time [s]')
ylabel('Rocket Mass [kg]');
grid on, grid minor


%% Dynamic Pressure Analysis

% Compute dynamic pressure
Dhist = zeros(Nt, 1);
qhist = zeros(Nt, 1);
rhohist = zeros(Nt, 1);
Thist = zeros(Nt, 1);
for i = 1:Nt
    [~, rhoi, Thist(i)] = stdAtmosphereCalc_hgp(hhist(i), RE, g0);
    [Dhist(i), qhist(i)] = dragCalc(Cd, Af, rhoi, vhist(i));
    rhohist(i) = rhoi;
end
qdothist = finitedifference(qhist, dt, nFD);
qdotdothist = finitedifference(qdothist, dt, nFD);

% Develop spline interpolation based function for first dynamic pressure
thist_1 = t0_maxq_1:dt:tend_maxq_1;
Nq_1 = length(thist_1);
qhist_1 = qhist(1:Nq_1);
qdothist_1 = qdothist(1:Nq_1);
qdotdothist_1 = qdotdothist(1:Nq_1);
qdotlim = [qdothist(1); qdothist(Nq_1)];
q_1_fun = @(t) cubicspline(...
    thist_1, qhist_1, t, qdotlim);

% Dynamic pressure derivative using finite difference
qdotdotlim = [qdotdothist(1); qdotdothist(Nq_1)];
qdot_splineDataMat = cubicsplineSolve(thist_1, qdothist_1', qdotdotlim');
qdot_1_fun = @(t) cubicsplineInterp( qdot_splineDataMat, t );

% Dynamic pressure derivative using the spline derivative
qdot_2_fun = @(t) anonymousFuncSecondArg(q_1_fun, t);
D_1_fun = @(t) cubicspline(...
    thist_1, Dhist(1:Nq_1), t, qdotlim);
thist_2 = t0_maxq_1:dt/100:tend_maxq_1;

% Ascent Dynamic Pressure curve
figure(10); hold off
plot(thist_1, qhist_1*1e-3, 'linewidth', 2);
title('Dynamic Pressure of Ascent Time History');
xlabel('Time [s]')
ylabel('Dynamic Pressure [kPa]');
grid on, grid minor

% Search for first dynamic pressure maximum
[t_qmax_1, niter, erra] = secantrootsolve(...
    qdot_1_fun, ta_maxq_1, tb_maxq_1);
qmax_1 = q_1_fun(t_qmax_1);
hold on
plot(t_qmax_1, qmax_1*1e-3, 'r.', 'markersize', 25);

% Ascent Dynamic Pressure Derivative curve
figure(11); hold off
plot(thist_1, qdothist_1, 'linewidth', 2);
hold on
plot(thist_2, qdot_2_fun(thist_2), 'linewidth', 2);
title('Dynamic Pressure of Ascent Time Derivative Time History');
xlabel('Time [s]')
ylabel('Dynamic Pressure Rate of Change [kPa/s]');
legend({'Finite Difference', 'Spline Interpolation'})
grid on, grid minor

% Output maximum dynamic pressure
fprintf('Max Q for Ascent Time: %.2f [s]\n', t_qmax_1);
fprintf('Max Q for Ascent Value: %.2f [kPa]\n', 1e-3*qmax_1);
fprintf('Drag Force at Max Q: %.2f [kN]\n', 1e-3*D_1_fun(t_qmax_1));

% Altitude of Max Q
h_fun = @(t) cubicspline(...
    thist_1, hhist(1:Nq_1), t);
fprintf('Altitude of Max Q: %.3f [km]\n', 1e-3*h_fun(t_qmax_1));

% Ascent Dynamic Pressure curve 2 
figure(12); hold off
plot(thist_1, qhist_1*1e-3, 'linewidth', 2);
hold on
plot(thist_2, q_1_fun(thist_2)*1e-3, 'linewidth', 2)
plot(t_qmax_1, qmax_1*1e-3, 'r.', 'markersize', 25);
title('Dynamic Pressure of Ascent Time History');
xlabel('Time [s]')
ylabel('Dynamic Pressure [kPa]');
legend({'Finite Difference', 'Spline Interpolation'})
grid on, grid minor
