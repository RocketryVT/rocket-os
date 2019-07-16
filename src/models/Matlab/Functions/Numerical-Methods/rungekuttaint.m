function [ykhist, tkhist, ydotkhist] = rungekuttaint(...
    ydot, y0, tlims, h, N, cvec, amat, bvec, errcheckflag)
% Runge-Kutta integration
% This function uses an N-th order Runge Kutta integration technique to
% solve a time-dependent differential equation.
% 
% @arg
% ydot     - Anonymous Function
%                Function which returns the time derivative of the state, 
%                which is a Ny x 1 double vector. Function takes the form
%                    ydot = f(t, y)
% y0           - Ny x 1 double vector
%                Initial condition
% tlims        - 2 x 1 double
%                Integration time limits: [ Initial time; Final time ]
% h            - double
%                Time step value for integration
% N            - int (optional)
%                Number of stages of Runge-Kutta integration. Default
%                Butcher Tableaus are written into the function, so "cvec",
%                "amat", and "bvec" need not be specified.
% cvec         - N x 1 double matrix (optional)
%                Butcher Tableau values for delta times in Runge-Kutta 
%                integration. Default is for 4-th order Runge-Kutta 
%                integration.
% amat         - N x N double matrix (optional)
%                Butcher Tableau values for intermediate steps in 
%                Runge-Kutta integration. Default is for 4-th order 
%                Runge-Kutta integration.
% bvec         - 1 x N double matrix (optional)
%                Butcher Tableau values for summation in Runge-Kutta 
%                integration. Default is for 4-th order Runge-Kutta 
%                integration.
% errcheckflag - bool (optional)
%                Flag to throw error if Butcher Tableau values are not
%                consistent with established constraints. Default is true,
%                but may be set to false to speed the function run time.
% 
% @return
% ykhist       - Ny x Nt double matrix
%                Time history of state values
% tkhist       - 1 x Nt double vector
%                Time history of time values of integration steps
% ydotkhist    - Ny x Nt double matrix
%                Time derivative values at each integration step
% 
% @author: Matt Marti
% @date: 2019-04-29

% Runge-Kutta order
if nargin < 5
    N = 4;
end

% Runge-Kutta Butcher Tableaus
if nargin < 6
    oo2 = 0.5;
    oo3 = 1/3;
    oo6 = 1/6;
    switch N
        case 4
            cvec = [ 0; oo2; oo2; 1 ];
            amat = [ 
                0,   0,   0,   0;
                oo2, 0,   0,   0;
                0,   oo2, 0,   0;
                0,   0,   1,   0];
            bvec = [ oo6, oo3, oo3, oo6 ];
        case 3
            cvec = [ 0; oo2; 1 ];
            amat = [ 
                0,   0,   0;
                oo2, 0,   0;
                0,   1,   0];
            bvec = [ oo6, 4*oo6, oo6 ];
        case 2
            cvec = [ 0; oo2 ];
            amat = [ 
                0,   0;
                oo2, 0];
            bvec = [ 0, 1 ];
        case 1
            cvec = 0;
            amat = 0;
            bvec = 1;
        case 6 % Default Dormand-Prince tableau from Orbit Determination
            cvec = [0; 0.2; 0.3; 0.8; 8/9; 1; 1];
            amat = [0,          0,           0,          0,        0,           0,     0;...
                    1/5,        0,           0,          0,        0,           0,     0;...
                    3/40,       9/40,        0,          0,        0,           0,     0;...
                    44/45,      -56/15,      32/9,       0,        0,           0,     0;...
                    19372/6561, -25360/2187, 64448/6561, -212/729, 0,           0,     0;...
                    9017/3168,  -355/33,     46732/5247, 49/176,   -5103/18656, 0,     0;...
                    35/384,     0,           500/1113,   125/192,  -2187/6784,  11/84, 0];
           bvec = [35/384, 0, 500/1113, 125/192, -2187/6784, 11/84];
        otherwise
            error('Unsupported Runge-Kutta integration order');
    end
end

% Check input Butcher Tableau tables
if nargin < 9
    errcheckflag = 1;
end
if nargin > 5 && errcheckflag
    
    % Input sizes
    assert(size(cvec, 1) == N, 'cvec must be a vector of length %d', N)
    assert(size(cvec, 2) == 1, 'cvec must be a column vector')
    assert(size(bvec, 1) == 1, 'bvec must be a row vector')
    assert(size(bvec, 2) == N, 'bvec must be a vector of length %d', N)
    assert(size(amat, 1) == N, 'amat must be an %d x %d matrix');
    assert(size(amat, 2) == N, 'amat must be an %d x %d matrix');
    
    % Input values
    assert(cvec(1) == 0, 'First element of cvec is not 0');
    assert(amat(1,1) == 0, 'First element (1,1) of amat is not 0');
    for i = 1:N
        assert(abs(cvec(i) - sum(amat(i,:))) < 1e-12, ['cvec(%d) does ' ...
            'not equalt the sum of the row %d in amat'], i, i);
    end
    assert(abs(sum(bvec) - 1) < 1e-12, 'bvec elements do not sum to 1');
end

% Transpose bvec to speed computations slightly
bvec = bvec'; % Trust me, it makes the computations slightly faster.

% Compute time history
tkhist = tlims(1):h:tlims(2);

% Input sizes
ny = size(y0, 1);
nt = size(tkhist, 2);

% Initialize integration loop
ydotkhist = zeros(ny, nt);
ykhist = zeros(ny, nt);
ykhist(:,1) = y0;

% Runge-Kutta integration loop
for k = 2:nt
    
    % Initialize Runge Kutta step
    Kjhist = zeros(ny, N);
    ykm1 = ykhist(:,k-1);
    tkm1 = tkhist(k-1);
    
    % Runge-Kutta loop formulation
    Kjhist(:,1) = ydot(tkm1, ykm1);
    for j = 2:N
        targ = tkm1 + h*cvec(j);
        Karg = ykm1;
        for i = 1:(j-1)
            if amat(j,i)
                Karg = Karg + h*amat(j,i)*Kjhist(:,i);
            end
        end
        Kjhist(:,j) = ydot(targ, Karg);
    end
    
    % Final compute state at time tk
    ykhist(:,k) = ykm1 + h*Kjhist*bvec;
    
    % Save ydot for ouptut
    ydotkhist(:,k-1) = Kjhist(:,1);
end

% Save ydot for ouptut
ydotkhist(:,nt) = ydot(tlims(2), ykhist(:,k));

end

