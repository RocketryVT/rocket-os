function include_HRFS(folder_path)
% Adds to the MATLAB path all functions for the Hybrid-Rocket-Flight-Sim
% 
% INPUTS
% folder_path - string (optional)
%               Directory path to the GPSv2 folder
% 
% @author: Matt Marti
% @date: 2019-04-18

% If no argument, add folders from here
if nargin == 0
    folder_path = '.';
end

% Change '\' to '/'
for i = 1:length(folder_path)
    if folder_path(i) == '\'
        folder_path(i) = '/';
    end
end

% Add '/' to the end of the path name
if folder_path(end) ~= '/'
    folder_path = [ folder_path, '/' ];
end

% Recursively add directories to the path
recursiveadd( [folder_path, '.'] );

end


function recursiveadd(str)
% Recursively add all directories in str to the path
addpath(str);
dirstruct = dir(str);
for i = 3:length(dirstruct)
    d = dirstruct(i);
    dirname = [str '/' d.name];
    if exist(dirname, 'dir')
        recursiveadd( dirname );
    end
end

end




