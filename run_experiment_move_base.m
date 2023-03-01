% 
clear all
rosshutdown
rosinit
node_matlab_sig = ros.Node('/move_base_sig');

% Send signal to record
signalstart = ros.Publisher(node_matlab_sig, "/start_record_wheelchair", ...
    "std_msgs/Float32", "IsLatching", true);

msg_record_start = rosmessage(signalstart);
t = rostime('now','DataFormat','struct');
msg_record_start.Data = single(t.Sec + t.Nsec*10^-9);
send(signalstart, msg_record_start);

zref = 4;
yref = 1;
pause(1.0);

% Send goal to move_base
[actClient,goalMsg] = rosactionclient('/move_base');
goalMsg.TargetPose.Header.FrameId = "wcias_odom";
goalMsg.TargetPose.Header.Stamp = rostime("now");
goalMsg.TargetPose.Pose.Position.X = zref;
goalMsg.TargetPose.Pose.Position.Y = yref;
goalMsg.TargetPose.Pose.Orientation.W = 1.0;
pause(5.0);
[resultMsg,resultState] = sendGoalAndWait(actClient, goalMsg, 20);
disp(resultMsg);
disp(resultState);

% Send signal to stop record
signalstop = ros.Publisher(node_matlab_sig, "/stop_record_wheelchair", ...
    "std_msgs/Float32", "IsLatching", true);
msg_record_stop = rosmessage(signalstop);
t = rostime('now','DataFormat','struct');
msg_record_stop.Data = single(t.Sec + t.Nsec*10^-9);
send(signalstop, msg_record_stop);

rosshutdown