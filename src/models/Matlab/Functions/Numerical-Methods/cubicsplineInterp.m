function [finter, dfinter, ddfinter] ...
    = cubicsplineInterp( splineDataMat, xinter, dflag, ddflag )
% Cubic spline interpolation function
% Interpolates function values at specified points using data from a solved
% cubic spline. To be used in conjunction with "cubicsplineSolve.m".
% 
% @arg
% splineDataMat - m x n x 5 double matrix
%                 Spline coefficient data matrix. Organized by input data
%                 dimension, known value points, and coefficient.
% xinter        - 1 x n double matrix
%                 Interpolation points
% dflag         - bool (optional)
%                 Optional flag to make the function return the derivative.
%                 False by default.
% ddflag        - bool (optional)
%                 Optional flag to make the function return the second 
%                 derivative of the interpolated function. False by 
%                 default.
% 
% @return
% finter        - n x 1 double matrix
%                 Interpolated function value
% dfinter       - n x 1 double matrix
%                 Interpolated function derivative value
% ddfinter      - n x 1 double matrix
%                 Interpolated function second derivative value
% 
% @author: Matt Marti
% @date: 2019-05-18

% Check input array sizes
assert(size(xinter,1) == 1 || size(xinter,2) == 1, ...
    'Argument ''xinter'' is not a vector');

% Size
nx = size(splineDataMat, 2);
m = size(splineDataMat, 1);

% Check derivative flags
if nargin < 4
    ddflag = 0;
    if nargin < 3
        dflag = 0;
    end
end

% Preallocate output
finter = zeros(m,length(xinter));
if dflag
    dfinter = zeros(m,length(xinter));
else
    dfinter = [];
end
if ddflag
    ddfinter = zeros(m,length(xinter));
else
    ddfinter = [];
end

% Partition a, b, c, d coefficients
akvec = squeeze(splineDataMat(:,:,1));
bkvec = squeeze(splineDataMat(:,:,2));
ckvec = squeeze(splineDataMat(:,:,3));
dkvec = squeeze(splineDataMat(:,:,4));
xkvec = squeeze(splineDataMat(1,:,5));

% Interpolate function
for j = 1:length(xinter)
    
    % Check that interpolated value is within function range
    assert(xinter(j) >= xkvec(1) && xinter(j) <= xkvec(nx),...
        'Interpolation value not within bounds');
    
    % Find x value just below xinter(i)
    k = 2;
    while k <= nx
        if xinter(j) < xkvec(k), k = k - 1; break; end
        k = k + 1;
    end
    if k > nx % Point is on upper boundary
        finter(:,j) = akvec(:,nx);
        if dflag
            dfinter(:,j) = bkvec(:,nx);
        end
        if ddflag
            ddfinter(:,j) = 2*ckvec(:,nx);
        end
        continue;
    end
    
    % Spline interpolation
    hi = xinter(j) - xkvec(k);
    finter(:,j) = akvec(:,k) + bkvec(:,k)*hi + ckvec(:,k)*hi^2 + dkvec(:,k)*hi^3;
    if dflag
        dfinter(:,j) = bkvec(:,k) + 2*ckvec(:,k)*hi + 3*dkvec(:,k)*hi^2;
    end
    if ddflag
        ddfinter(:,j) = 2*ckvec(:,k) + 6*dkvec(:,k)*hi;
    end
end
    
end

