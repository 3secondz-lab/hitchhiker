clear;

%% Set ROS environment
addpath('/home/rnd/catkin_ws/src/matlab_gen/msggen')


topic_name = '/matlab_controller/';


setenv('ROS_MASTER_URI', 'http://rnd-3secondz:11311');
try
    rosinit;
catch
    rosshutdown;
    rosinit;
end

matlab_pub = rospublisher(topic_name, 'std_msgs/Float64MultiArray');



%% Data subscription example
try 
    Initialization_JH;
    while(1)
        msg = rosmessage('std_msgs/Float64MultiArray');
        Head_code_JH;
        msg.Data = [u_steer, u_acc, u_brk];
        msg.Layout.Dim = rosmessage('std_msgs/MultiArrayDimension');
        msg.Layout.Dim.Label = 'val';
        msg.Layout.Dim.Size = 3;
        msg.Layout.Dim.Stride = 3;
        
        send(matlab_pub, msg);
        
        pause(0.1);
    end
catch
    pause();
end

rosshutdown
