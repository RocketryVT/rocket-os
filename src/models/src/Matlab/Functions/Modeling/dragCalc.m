function [D, q] = dragCalc(Cd, A, rho, vvec)
% Computes the force of drag acting on an object
% 
% @arg
% Cd   - double
%        Object drag coefficient
% A    - double
%        Object reference area
% rho  - double
%        Ambient density
% vvec - double vector
%        Object velocity vector
% 
% @return
% D    - double vector
%        Drag force vector
% q    - double
%        Dynamic Pressure
% 
% @author: Matt Marti
% @date: 2019-05-01

vk = norm(vvec); % Velocity magnitude
if vk
    unit_direction = vvec ./ vk;
    q = 0.5 * rho * vk*vk; % Vector Dynamic pressure
    D = - Cd * A * q * unit_direction; % Vector drag force
else
    q = 0;
    D = 0;
end

end

