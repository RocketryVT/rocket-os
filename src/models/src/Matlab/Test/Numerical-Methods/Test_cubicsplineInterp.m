%% Test_cubicsplineInterp.m
% 
% Test case for the cubic spline function. This function just interpolates
% at the given values, given a set of spline data.
% 
% @author: Matt Marti
% @date: 2019-05-18

clear


%% Part 1
% Test the spline works for one dimensional case

% Given
f = @(x) sin(x);
df = @(x) cos(x);
ddf = @(x) -sin(x);
xkvec = linspace(0, 10, 20);
fkvec = f(xkvec);
xinter = linspace(0, 10, 1000);
fslope = [ cos(xkvec(1)), cos(xkvec(end)) ]; % Clambed B.C.s

% Solve spline
dflag = 1;
ddflag = 1;
[ splineDataMat ] = cubicsplineSolve( xkvec, fkvec, fslope );
[finter, dfinter, ddfinter] ...
    = cubicsplineInterp( splineDataMat, xinter, dflag, ddflag );

% Test Function values
fitrue = f(xinter);
error = fitrue - finter;
maxerr = max(abs(error));
assert(maxerr < 2.5e-3, 'Spline error too high');

% Test Derivative values
dfitrue = df(xinter);
errord = dfitrue - dfinter;
maxerrd = max(abs(errord));
assert(maxerrd < 1.5e-3, 'Spline derivative error too high');

% Test Derivative values
ddfitrue = ddf(xinter);
errordd = ddfitrue - ddfinter;
maxerrdd = max(abs(errordd));
assert(maxerrdd < 2.5e-2, 'Spline second derivative error too high');


%% Part 2
% Test the spline works for a two dimensional case

% Given
f = @(x) [sin(x); -10*x.^2 + 50*x + 1000];
df = @(x) [cos(x); -20*x + 50];
ddf = @(x) [-sin(x); -20 + 0*x];
xkvec = linspace(0, 10, 20);
fkvec = f(xkvec);
xinter = linspace(0, 10, 1000);
fslope = [ df(xkvec(1)), df(xkvec(end)) ]; % Clambed B.C.s

% Run spline
dflag = 1;
ddflag = 1;
[ splineDataMat ] = cubicsplineSolve( xkvec, fkvec, fslope );
[finter, dfinter, ddfinter] ...
    = cubicsplineInterp( splineDataMat, xinter, dflag, ddflag );

% Test Function values
fitrue = f(xinter);
error = fitrue - finter;
maxerr = max(max(abs(error)));
assert(maxerr < 2.5e-4, 'Spline error too high');

% Test Derivative values
dfitrue = df(xinter);
errord = dfitrue - dfinter;
maxerrd = max(max(abs(errord)));
assert(maxerrd < 1.5e-3, 'Spline derivative error too high');

% Test Second Derivative values
ddfitrue = ddf(xinter);
errordd = ddfitrue - ddfinter;
maxerrdd = max(max(abs(errordd)));
assert(maxerrdd < 2.5e-2, 'Spline second derivative error too high');


%% Pass

fprintf('PASSED: Test_cubicsplineInterp\n');