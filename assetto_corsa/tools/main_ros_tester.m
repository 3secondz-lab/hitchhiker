clear;

%% Set ROS environment
addpath('/home/rnd/catkin_ws/src/matlab_gen/msggen')


topic_name1 = '/test_msg/float_vals';
topic_name2 = '/test_msg/ACRaw';
topic_name3 = '/test_msg/image';


setenv('ROS_MASTER_URI', 'http://rnd-3secondz:11311');
try
    rosinit;
catch
    rosshutdown;
    rosinit;
end

while(1)
    matlab_pub1 = rospublisher(topic_name1, 'std_msgs/Float64MultiArray');
    matlab_pub2 = rospublisher(topic_name2, 'assetto_corsa/ACRaw');
    matlab_pub3 = rospublisher(topic_name3, 'sensor_msgs/Image');
    
    t = rostime('now');
    
    msg2.Header.Stamp = t;
    msg3.Header.Stamp = t;
    msg4.Header.Stamp = t;

    msg1 = rosmessage('std_msgs/Float64MultiArray');
    msg2 = rosmessage('assetto_corsa/ACRaw');
    msg3 = rosmessage('sensor_msgs/Image');

    msg1.Data = 10*randn(3, 1);
    msg1.Layout.Dim = rosmessage('std_msgs/MultiArrayDimension');
    msg1.Layout.Dim.Label = 'val';
    msg1.Layout.Dim.Size = 3;
    msg1.Layout.Dim.Stride = 3;

    msg2.Brake.Data = 10*randn(1, 1);
    msg2.Steer.Data = 10*randn(1, 1);
    
    img = imread('/home/rnd/Downloads/melodic_with_bg.png');
    img = imresize(img, 0.25);
    msg3.Encoding = 'rgb8';
    writeImage(msg3, img);

    send(matlab_pub1, msg1);
    send(matlab_pub2, msg2);
    send(matlab_pub3, msg3);
    
    pause(0.1);
end

rosshutdown;

