clear;

%% Set ROS environment
addpath('/home/rnd/catkin_ws/src/matlab_gen/msggen')


topic_name = '/ac_pub/ACRaw';


setenv('ROS_MASTER_URI', 'http://rnd-3secondz:11311');
try
    rosinit;
catch
    rosshutdown;
    rosinit;
end

ac_sub = rossubscriber(topic_name, 'BufferSize', 1);


%% Data subscription example
try
    while(1)
        data = receive(ac_sub);
        
        if isempty(data)
            continue;
        end
        
        % Get necessary data
        SpeedMs = data.SpeedMs;
        Steer = data.Steer;
        CarCoordinates = data.CarCoordinates;
        CarPositionNormalized = data.CarPositionNormalized;
        AccGFrontal = data.AccGFrontal;
        AccGHorizontal = data.AccGHorizontal;
        Brake = data.Brake;
        SlipAngle = data.SlipAngle;
        WheelAngularSpeed = data.WheelAngularSpeed;
        Load = data.Load;
        
        plane_center_x = data.PlaneCenterX;
        plane_center_y = data.PlaneCenterY;
        plane_left_x = data.PlaneLeftX;
        plane_left_y = data.PlaneLeftY;
        plane_right_x = data.PlaneRightX;
        plane_right_y = data.PlaneRightY;
        plane_radius = data.PlaneRadius;
        plane_psie = data.PlanePsie;
        plane_cte = data.PlaneCte;
        
        cam_center_x = data.CamCenterX;
        cam_center_y = data.CamCenterY;
        cam_left_x = data.CamLeftX;
        cam_left_y = data.CamLeftY;
        cam_right_x = data.CamRightX;
        cam_right_y = data.CamRightY;
        
        pause(0.01);
    end
catch
    rosshutdown;
end
