% Reset matlab
rosshutdown
Initialization_Simulink
rosinit
node_matlab_sig = ros.Node('/matlab_simulink_sig');

% Send signal to record
signalstart = ros.Publisher(node_matlab_sig, "/start_record_wheelchair", ...
    "std_msgs/Float32", "IsLatching", true);

msg_record_start = rosmessage(signalstart);
t = rostime('now','DataFormat','struct');
msg_record_start.Data = single(t.Sec + t.Nsec*10^-9);
send(signalstart, msg_record_start);

zref = 4;
yref = 1;

% Start simulation
sim('WMR_ROS_MATMPC_SIMULINK', 20)

% Send signal to stop record
signalstop = ros.Publisher(node_matlab_sig, "/stop_record_wheelchair", ...
    "std_msgs/Float32", "IsLatching", true);
msg_record_stop = rosmessage(signalstop);
t = rostime('now','DataFormat','struct');
msg_record_stop.Data = single(t.Sec + t.Nsec*10^-9);
send(signalstop, msg_record_stop);

rosshutdown