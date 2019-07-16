%% Test_All.m
% 
% Runs all unit tests in the Hybrid Rocket Flight Sim suite.
% 
% @author: Matt Marti
% @date: 2019-05-07

clear, clc, clear global

% Numerical Methods
Test_secantrootsolve
Test_gausselimination
Test_cubicspline
Test_rungekuttaint
Test_finitedifference
Test_cubicsplineSolve
Test_cubicsplineInterp

% Ambient
Test_stdAtmosphereCalc_hgp

% Dynamics
Test_dragCalc

% RocketEngine
Test_hybridRocketThrustCalc

% Time-dependent PDE for computing position
Test_fHRFSdynamics_1D







