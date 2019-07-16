function [finter, dfinter, akvec, bkvec, ckvec, dkvec, xstar] ...
    = cubicspline(xkvec, fkvec, xinter, fslope)
% Cubic spline interpolation function
% Interpolates function values at specified points using a cubic spline
% 
% @arg
% xkvec   - n x 1 double matrix
%           Independent variable data points
% fkvec   - n x 1 double matrix
%           Dependent variable data points
% xinter  - n x 1 double matrix
%           Interpolation points
% fslope  - 2 x 1 double matrix (optional)
%           Function slope at boundary points
% 
% @return
% finter  - n x 1 double matrix
%           Interpolated function value
% dfinter - n x 1 double matrix
%           Interpolated function derivative value
% akvec   - n x 1 double matrix
%           Spline coefficient vector: a
% bkvec   - n x 1 double matrix
%           Spline coefficient vector: b
% ckvec   - n x 1 double matrix
%           Spline coefficient vector: c
% dkvec   - n x 1 double matrix
%           Spline coefficient vector: d
% xstar   - n x 1 double matrix
%           Intermediate solution in tri-diagonal equations
% 
% @author: Matt Marti
% @date: 2019-05-06

% Size
nx = length(xkvec);

% Check boundary condition
cbcflag = 1;
if nargin < 4
    cbcflag = 0;
    fslope = [0;0];
else
    assert(numel(fslope) == 2, 'argument ''fslope'' must have 2 elements');
end

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
akvec = fkvec;
xstar = zeros(nx,1);
for k = 2:n
    xstar(k) = 3*( ( akvec(k+1) - akvec(k) ) / hkvec(k) ...
                  - ( akvec(k) - akvec(k-1) ) / hkvec(k-1) );
end
if cbcflag
    xstar(1) = 3*( (akvec(2) - akvec(1) )/hkvec(1) - fslope(1) );
    xstar(nx) = 3*( fslope(2) - (akvec(nx) - akvec(nx-1) )/hkvec(nx-1) );
end

% Solve tri-diagonal system of equations
ckvec = gausselimination(H, xstar);

% Compute bkvec and dkvec
bkvec = zeros(nx,1);
dkvec = zeros(nx,1);
for k = 1:n
    bkvec(k) = ( akvec(k+1) - akvec(k) ) / hkvec(k) ...
        - hkvec(k) * ( 2*ckvec(k) + ckvec(k+1) ) / 3;
    dkvec(k) = ( ckvec(k+1) - ckvec(k) ) / ( 3*hkvec(k) );
end
bkvec(nx) = fslope(2);

% Interpolate function
finter = zeros(size(xinter));
dfinter = zeros(size(xinter));
for i = 1:length(xinter)
    
    % Check that interpolated value is within function range
    assert(xinter(i) >= xkvec(1) && xinter(i) <= xkvec(nx),...
        'Interpolation value not within bounds');
    
    % Find x value just below xinter(i)
    k = 2;
    while k <= nx
        if xinter(i) < xkvec(k), k = k - 1; break; end
        k = k + 1;
    end
    if k > nx % Point is on upper boundary
        finter(i) = akvec(nx);
        dfinter(i) = bkvec(nx);
        continue;
    end
    
    % Spline interpolation
    hi = xinter(i) - xkvec(k);
    finter(i) = akvec(k) + bkvec(k)*hi + ckvec(k)*hi^2 + dkvec(k)*hi^3;
    dfinter(i) = bkvec(k) + 2*ckvec(k)*hi + 3*dkvec(k)*hi^2;
end
    
end

