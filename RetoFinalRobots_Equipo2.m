clear
close all
interface = "Gazebo";
%ROS Device IP
ROSDeviceAddress = '192.168.233.128';
Port= '11311';
HostAddress = '10.22.243.119';
ros_master = strcat('http://',ROSDeviceAddress,':',Port);

setenv('ROS_MASTER_URI',ros_master)
setenv('ROS_IP',HostAddress)
rosinit

%UR robot Selection
ur3e = loadrobot('universalUR3e');
ur = urROSNode(ROSDeviceAddress,'RigidBodyTree',ur3e);

%Subscribe to the topics of the Puzzlebot and the camera
odom = rossubscriber('/puzzlebot/odom','DataFormat','struct');
coordenadas_piezas_rojo = rossubscriber('/polygon_coordinates/red','geometry_msgs/PointStamped','DataFormat','struct');
coordenadas_piezas_verde = rossubscriber('/polygon_coordinates/green','geometry_msgs/PointStamped','DataFormat','struct');
coordenadas_piezas_amarillo = rossubscriber('/polygon_coordinates/yellow','geometry_msgs/PointStamped','DataFormat','struct');
coordenadas_piezas_morado = rossubscriber('/polygon_coordinates/purple','geometry_msgs/PointStamped','DataFormat','struct');

% Recibir un mensaje (espera hasta que llegue un mensaje)
msg_rojo = receive(coordenadas_piezas_rojo);
msg_verde = receive(coordenadas_piezas_verde);
msg_amarillo = receive(coordenadas_piezas_amarillo);
msg_morado = receive(coordenadas_piezas_morado);

%Services
vaccum_on = rossvcclient('/on','DataFormat','struct');
vaccum_off = rossvcclient('/off','DataFormat','struct');

%Publishers
%puzzlebot_cmd_pub = rospublisher("/puzzlebot/cmd_vel","geometry_msgs/Twist","DataFormat","struct");
target = rospublisher("/target","geometry_msgs/Point","DataFormat","struct");

%Main
sendTarget(target,1);
sendTarget(target,1);
home_pos(ur,1) %HOME
x_rojo = msg_rojo.Point.X; % Extraer las coordenadas x, y, z del mensaje
y_rojo = msg_rojo.Point.Y;
z_rojo = msg_rojo.Point.Z;
x_verde = msg_verde.Point.X;
y_verde = msg_verde.Point.Y;
z_verde = msg_verde.Point.Z;
x_amarillo = msg_amarillo.Point.X;
y_amarillo = msg_amarillo.Point.Y;
z_amarillo = msg_amarillo.Point.Z;
x_morado = msg_morado.Point.X;
y_morado = msg_morado.Point.Y;
z_morado = msg_morado.Point.Z;

pick_pos(ur,2) %Posicion de recoger
pick_red_block_arm(ur,2,x_rojo,y_rojo,z_rojo) %Recoger Rojo
call(vaccum_on);
deliver_red_block_arm(ur); %Dejar Rojo
call(vaccum_off);

pick_pos(ur,2) %Posicion de recoger
pick_green_block_arm(ur,2,x_verde,y_verde,z_verde) %Recoger Verde
call(vaccum_on);
deliver_green_block_arm(ur); %Dejar Verde
call(vaccum_off);

pick_pos(ur,2) %Posicion de recoger
pick_yellow_block_arm(ur,2,x_amarillo,y_amarillo,z_amarillo)
call(vaccum_on);
Pose = getPuzzlebotOdom(odom);
deliver_yellow_block_arm(ur,Pose); %Dejar Amarillo
call(vaccum_off);

pick_pos(ur,2) %Posicion de recoger
pick_purple_block_arm(ur,2,x_morado,y_morado,z_morado) %Recoger Morado
call(vaccum_on);
Pose = getPuzzlebotOdom(odom);
deliver_purple_block_arm(ur,Pose); %Dejar Morado
call(vaccum_off);
pause(1);
sendTarget(target,2);
sendTarget(target,2);
sendTarget(target,2);

home_pos(ur,3) %Home
rosshutdown

function home_pos(ur, end_time) %Seguir Trayectoria
    jointWaypoints = [1.57 -1.57 1.57 -1.57 -1.57 0];
    sendJointConfiguration(ur,jointWaypoints,'EndTime',end_time);
    wait_motion(ur)
end

%Seguir Trayectoria
function pick_pos(ur, end_time)
    jointWaypoints = [0 -1.57 1.57 -1.57 -1.57 0];
    sendJointConfiguration(ur,jointWaypoints,'EndTime',end_time);
    wait_motion(ur)
end


function pick_red_block_arm(ur,end_time,x_rojo,y_rojo,z_rojo) 
%Ajustar de acuerdo a la camara centro de la camara x = 0.5 ; y = 0
% Altura maxima-Zcamara= 0.259   posR=Zcamara+0.259
    desPos = [-1.57    0.000    3.1416    (0.5-x_rojo)    (0.000-y_rojo)    (1.0322542303-z_rojo)];
    sendCartesianPose(ur,desPos,'EndTime',end_time)
    wait_motion(ur)
end

function pick_green_block_arm(ur,end_time,x_verde,y_verde,z_verde)  
    desPos = [-1.57    0.000    3.1416    (0.5-x_verde)    (0.000-y_verde)    (1.0322542303-z_verde)];
    sendCartesianPose(ur,desPos,'EndTime',end_time)
    wait_motion(ur)
end

function pick_yellow_block_arm(ur,end_time,x_amarillo,y_amarillo,z_amarillo) 
    desPos = [-1.57    0.000    3.1416    (0.5-x_amarillo)    (0.000-y_amarillo)    (1.032-z_amarillo)];
    sendCartesianPose(ur,desPos,'EndTime',end_time)
    wait_motion(ur)
end

function pick_purple_block_arm(ur,end_time,x_morado,y_morado,z_morado) 
    desPos = [-1.57    0.000    3.1416    (0.5-x_morado)    (0.000-y_morado)    (1.032-z_morado)];
    sendCartesianPose(ur,desPos,'EndTime',end_time)
    wait_motion(ur)
end

%Follow trajectory
function deliver_red_block_arm(ur) 
%[roll pitch yaw x y z];
    pose = getCartesianPose(ur);
    taskWayPoints = [pose;
                     -1.57    0    3.1416    0.4       0    0.7;
                     -1.57    0    3.1416   0.4       -0.5    0.7;
                     0       0    3.1416     0       -0.5    0.6;
                     0       0    3.1416    0       -0.5    0.259];
    
    wayPointTimes = [0 1 2 3 4];
    
    followWaypoints(ur,taskWayPoints,wayPointTimes,'InterpolationMethod','trapveltraj','NumberOfSamples',100);
    [result,~] = getMotionStatus(ur);
    while ~result
        [result,~] = getMotionStatus(ur);
    end
    

end

function deliver_green_block_arm(ur) 
    pose = getCartesianPose(ur);
    taskWayPoints = [pose;
                     -1.57    0    3.1416    0.4       0.1    0.7;
                     -1.57    0    3.1416   0.4       -0.5    0.7;
                     0       0    3.1416     0.2       -0.5    0.7;
                     0       0    3.1416    0.2       -0.5    0.259];
    wayPointTimes = [0 1 2 3 4];
    
    followWaypoints(ur,taskWayPoints,wayPointTimes,'InterpolationMethod','trapveltraj','NumberOfSamples',100);
    [result,~] = getMotionStatus(ur);
    while ~result
        [result,~] = getMotionStatus(ur);
    end
end

function deliver_yellow_block_arm(ur,position) 
    pose = getCartesianPose(ur);
    position_x = position(1);
    position_y = position(2);
    taskWayPoints = [pose; 
                     -1.57    0    3.1416    0.4       -0.2    0.7;
                     -1.57    0    3.1416   0.4       -0.2    0.7;
                     0       0    3.1416     0       -0.2    0.7;
                     0       0    3.1416    (position_x-1.3)       (1.75-position_y)    0.29;
                     0       0    3.1416    (position_x-1.6)       (1.75-position_y)    0.25];
    wayPointTimes = [0 1 3 5 7 9];
    
    followWaypoints(ur,taskWayPoints,wayPointTimes,'InterpolationMethod','trapveltraj','NumberOfSamples',100);
    [result,~] = getMotionStatus(ur);
    while ~result
        [result,~] = getMotionStatus(ur);
    end
end

function deliver_purple_block_arm(ur,position) 
    pose = getCartesianPose(ur);
    position_x = position(1);
    position_y = position(2);
    taskWayPoints = [pose;
                     -1.57    0    3.1416    0.4       -0.2    0.7;
                     -1.57    0    3.1416   0.4       -0.2    0.7;
                     0       0    3.1416     0       -0.2    0.7;
                     0       0    3.1416    (position_x-1.3)       (1.75-position_y)    0.29;
                     0       0    3.1416    (position_x-1.45)       (1.84-position_y)    0.25];
    wayPointTimes = [0 1 3 5 7 9];
    
    followWaypoints(ur,taskWayPoints,wayPointTimes,'InterpolationMethod','trapveltraj','NumberOfSamples',100);
    [result,~] = getMotionStatus(ur);
    while ~result
        [result,~] = getMotionStatus(ur);
    end
end

%Wait until motion finishes
function wait_motion(ur)
    [result,~] = getMotionStatus(ur);
    while ~result
        [result,~] = getMotionStatus(ur);
    end
end

%Obtener pose del puzzlebot [x,y,theta]
function Puzzlebot_pose = getPuzzlebotOdom(subscriber)
    odom_msg = receive(subscriber,3);
    pose = odom_msg.Pose.Pose;
    orientation = quat2eul([pose.Orientation.W pose.Orientation.X pose.Orientation.Y pose.Orientation.Z]);
    Puzzlebot_pose = [pose.Position.X pose.Position.Y orientation(1)];
end

function sendTarget(target,trayectoria)
    % Crear el mensaje
    msg = rosmessage(target);
    
    if trayectoria == 1
        msg.X = 1;  % Valor en x
        msg.Y = 2;  % Valor en y
        msg.Z = 0;  % Valor en z
    end
    if trayectoria == 2
        msg.X = -4;  % Valor en x
        msg.Y = 1;  % Valor en y
        msg.Z = 0;  % Valor en z
    end
    % Publicar el mensaje
    send(target, msg);
end