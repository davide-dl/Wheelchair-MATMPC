% Reset matlab
% Initialization_Simulink
rosinit
node_matlab_sig = ros.Node('/matlab_sig1');

% Send signal to record
signalstart = ros.Publisher(node_matlab_sig, "/start_record_wheelchair", ...
    "std_msgs/Float32", "IsLatching", true);

msg_record_start = rosmessage(signalstart);
msg_record_start.Data = single(100.1);
send(signalstart, msg_record_start);

zref = 2;
yref = 1;

% Start simulation
% sim('WMR_ROS_MATMPC_SIMULINK', 20)

% Send signal to stop record
signalstop = ros.Publisher(node_matlab_sig, "/stop_record_wheelchair", ...
    "std_msgs/Float32", "IsLatching", true);
msg_record_stop = rosmessage(signalstop);
msg_record_stop.Data = single(200.2);
send(signalstop, msg_record_stop);

rosshutdown