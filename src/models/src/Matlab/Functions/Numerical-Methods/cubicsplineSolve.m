function [ splineDataMat ] = cubicsplineSolve( xkvec, fkvec, fslope )
% Cubic spline interpolation function
% Solves for the parameters that describe a cubic spline. To be used in
% conjunction with "cubicsplineInterp.m". This function can solve the
% parameters of a cubic spline given data from a vectorized system. For
% example, the three space position of a planet in orbit can be
% interpolated.
% 
% @arg
% xkvec         - 1 x n double matrix
%                 Independent variable data points
% fkvec         - m x n double matrix
%                 Dependent variable data points
% fslope        - m x 2 double matrix (optional)
%                 Function slope at boundary points
% 
% @return
% splineDataMat - m x n x 5 double matrix
%                 Spline coefficient data matrix. Organized by input data
%                 dimension, known value points, and coefficient.
% 
% @author: Matt Marti
% @date: 2019-05-18

% Size
nx = length(xkvec);
m = size(fkvec, 1);

% Check input array sizes
assert(size(xkvec,1) == 1 || size(xkvec,2) == 1, ...
    'Argument ''xkvec'' is not a vector');
if size(xkvec,1) ~= 1
    xkvec = xkvec';
end
assert(size(fkvec,2) == nx, ...
    'Argument ''fkvec'' does not match length of ''xkvec''');
if nargin > 2
    assert(size(fslope,1) == m && size(fslope,2) == 2, ...
        'Argument ''fslope'' is of incorrect size: is %d by %d', ...
        size(fslope,1), size(fslope,2));
    cbcflag = 1;
else
    cbcflag = 0;
end

% Preallocate spline data matrix for output
splineDataMat = zeros(m, nx, 4);

% Build tri-diagonal system of equations
hkvec = xkvec(2:nx) - xkvec(1:nx-1);
n = nx - 1;
H = eye(nx);
for k = 2:n
    H(k, k-1) = hkvec(k-1);
    H(k, k) = 2 * ( hkvec(k-1) + hkvec(k) );
    H(k, k+1) = hkvec(k);
end
if cbcflag
    H(1, 1) = 2*hkvec(1);
    H(1, 2) = hkvec(1);
    H(n+1, n) = hkvec(n);
    H(n+1, n+1) = 2*hkvec(n);
end

% Generate right hand side
akvec = fkvec';
xstar = zeros(nx,m);
for k = 2:n
    xstar(k,:) = 3*( ( akvec(k+1,:) - akvec(k,:) ) / hkvec(k) ...
                  - ( akvec(k,:) - akvec(k-1,:) ) / hkvec(k-1) )';
end
if cbcflag
    xstar(1,:) = 3*( (akvec(2,:) - akvec(1,:) )/hkvec(1) - fslope(:,1)' );
    xstar(nx,:) = 3*( fslope(:,2)' - (akvec(nx,:) - akvec(nx-1,:) )/hkvec(nx-1) );
end

% Solve tri-diagonal system of equations
ckvec = gausselimination(H, xstar);

% Compute bkvec and dkvec
bkvec = zeros(nx,m);
dkvec = zeros(nx,m);
for k = 1:n
    bkvec(k,:) = ( akvec(k+1,:) - akvec(k,:) ) / hkvec(k) ...
        - hkvec(k) * ( 2*ckvec(k,:) + ckvec(k+1,:) ) / 3;
    dkvec(k,:) = ( ckvec(k+1,:) - ckvec(k,:) ) / ( 3*hkvec(k) );
end
if cbcflag
    bkvec(nx,:) = fslope(:,2)';
end

% Assign data vectors to output
splineDataMat(:,:,1) = akvec';
splineDataMat(:,:,2) = bkvec';
splineDataMat(:,:,3) = ckvec';
splineDataMat(:,:,4) = dkvec';
splineDataMat(1,:,5) = xkvec;

end