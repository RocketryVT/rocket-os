function [x, niter, erra ] = secantrootsolve(f, a, b, maxiter, errstop)
% Solves for the roots of a function using the secant method
% 
% @arg
% f       - Anonymous Function
%           Function handle to solve root for
% a       - double
%           Upper bound
% b       - double
%           Lower bound
% maxiter - int (optional)
%           Maximum number of iterations
% errstop - double (optional)
%           Minimum error stopping criteria (in difference)
% 
% @return
% x       - double
%           Function root
% niter   - int
%           Nnumber of iterations
% erra    - double
%           Root error
% 
% @author: Matt Marti
% @date: 2019-04-21

% Parameters
if nargin < 4
    maxiter = 1000;
end
if nargin < 5
    errstop = 1e-12; % Stopping criteria
end

% Check that root exists (Intermediate value theorem)
fa = f(a);
fb = f(b);
assert(f(a) * f(b) <= 0, 'Root does not exist');

% Initialize loop
if abs(fa) < abs(fb), pim1 = a; else, pim1 = b; end
erra = abs(f(pim1));

% Loop
i = 1; niter = 0;
while erra > errstop && i < maxiter
    
    % Secant method
    p = a - fa * (b - a) / (fb - fa);
    
    % Assign this guess to next bounds
    fp = f(p);
    if (fp*fa < 0)
        b = p;
    elseif (fp*fb < 0)
        a = p;
    else % fp = 0
        break;
    end
    
    % Measure error
    erra = abs( (p - pim1) / pim1 );
    
    % Stopping criteria for prompt
    if ~niter && erra < errstop
        break;
    end
    
    % Iterate
    pim1 = p;
    i = i + 1;
end
x = p;
niter = i;

end

