%% Test_cubicsplineSolve.m
% 
% Test case for the cubic spline solver function. This function just solves
% for the spline data, so that the spline can be precomputed before code is
% run. This improves code performance by removing the need to invert a
% matrix every time the spline function is called.
% 
% @author: Matt Marti
% @date: 2019-05-18

clear


%% Part 1
% Test the spline

% Given
f = @(x) sin(x);
df = @(x) cos(x);
xkvec = linspace(0, 10, 20);
fkvec = f(xkvec);
xinter = linspace(0, 10, 1000);
fslope = [ df(xkvec(1)), df(xkvec(end)) ]; % Clambed B.C.s

% Run true spline
[~, ~, akvec, bkvec, ckvec, dkvec] ...
    = cubicspline(xkvec, fkvec, xinter, fslope);
splineDataTrue = zeros(1, length(xkvec), 4);
splineDataTrue(1,:,1) = akvec;
splineDataTrue(1,:,2) = bkvec;
splineDataTrue(1,:,3) = ckvec;
splineDataTrue(1,:,4) = dkvec;
splineDataTrue(1,:,5) = xkvec;
splineDataTrue(1,end,2) = fslope(1,2);

% Run new spline
[ splineDataMat ] = cubicsplineSolve( xkvec, fkvec, fslope );

% Test Function truth values
error = splineDataMat - splineDataTrue;
maxerr = max(max(max(abs(error))));
assert(maxerr < 1e-12, 'Spline error too high');


%% Part 2
% Test the spline works for a two dimensional case

% Given
f = @(x) [sin(x); -10*x.^2 + 50*x + 1000];
df = @(x) [cos(x); -20*x + 50];
xkvec = 0:.1:10;
fkvec = f(xkvec);
xinter = linspace(0, 10, 1000);
fslope = [ df(xkvec(1)), df(xkvec(end)) ]; % Clambed B.C.s

% Preallocate truth spline data
m = 2;
n = length(xkvec);
splineDataTrue = zeros(m, n, 4);
splineDataTrue(1,:,5) = xkvec;

% Run true spline for first dataset
[~, ~, akvec, bkvec, ckvec, dkvec] ...
    = cubicspline(xkvec, fkvec(1,:), xinter, fslope(1,:));
splineDataTrue(1,:,1) = akvec;
splineDataTrue(1,:,2) = bkvec;
splineDataTrue(1,:,3) = ckvec;
splineDataTrue(1,:,4) = dkvec;
splineDataTrue(1,n,2) = fslope(1,2);

% Run true spline for second dataset
[~, ~, akvec, bkvec, ckvec, dkvec] ...
    = cubicspline(xkvec, fkvec(2,:), xinter, fslope(2,:));
splineDataTrue(2,:,1) = akvec;
splineDataTrue(2,:,2) = bkvec;
splineDataTrue(2,:,3) = ckvec;
splineDataTrue(2,:,4) = dkvec;
splineDataTrue(2,n,2) = fslope(2,2);

% Run new spline
[ splineDataMat ] = cubicsplineSolve( xkvec, fkvec, fslope );

% Test Function truth values
error = splineDataMat - splineDataTrue;
maxerr = max(max(max(abs(error))));
assert(maxerr < 1e-12, 'Spline error too high');


%% Pass

fprintf('PASSED: Test_cubicsplineSolve\n');