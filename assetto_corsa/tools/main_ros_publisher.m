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
% try 
    Initialization_JH;
    timeint=0;
    t_prev = now;
    while(1)
%         tic
        try
        msg = rosmessage('std_msgs/Float64MultiArray');
%         Head_code_JH;
        head_code_0526;
        msg.Data = [u_steer, u_acc, u_brk];
        msg.Layout.Dim = rosmessage('std_msgs/MultiArrayDimension');
        msg.Layout.Dim.Label = 'val';
        msg.Layout.Dim.Size = 3;
        msg.Layout.Dim.Stride = 3;
        
        send(matlab_pub, msg);
%         figure(10);plot(Y_cam,X_cam);xlim([-10,10]);ylim([0,50])
%         figure(20);plot(curv_preview);ylim([-0.02,0.02])
%         pause(0.05);
        catch
           disp('catched'); 
        end
%         tt=toc;
%         timeint=timeint+tt;
        t_now = now;
        timeint = timeint + (t_now - t_prev);
        t_prev = t_now;
    end
% catch
%     pause();
% end

rosshutdown
