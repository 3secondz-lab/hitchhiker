clear;


%% Set ROS environment
addpath('/home/rnd/catkin_ws/src/matlab_gen/msggen')

% Parameter server name
node_name = 'ac_param_server';

try
    rosinit;
catch
    rosshutdown;
    rosinit;
end

while(1)
    % Get parameter as a struct
    params = rosparam('get', node_name);
    
    disp(params);
    
    % Put your function here
    % e.g.)
    % output = FUNC_MATLAB_CONTROLLER(input_data, params)
    
    pause(0.5);
end

rosshutdown;

