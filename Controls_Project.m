%% 
clc;
clear;
J = 3.2284E-6;
b = 3.5077E-6;
K = 0.0274;
R = 4;
L = 2.75E-6;
s = tf('s');
P_motor = K/(s*((J*s+b)*(L*s+R)+K^2))
% %state space
% A = [0 1 0
%     0 -b/J K/J
%     0 -K/L -R/L];
% B = [0 ; 0 ; 1/L];
% C = [1 0 0];
% D = [0];
% 
% motor_ss = ss(A,B,C,D);
% linearSystemAnalyzer('step', P_motor, 0:0.1:5);
% rP_motor = 0.1/(0.5*s+1);

figure 
t = 0:0.001:0.2;
step(P_motor,t)
figure

isstable(P_motor)
pole(P_motor)
sys_cl = feedback(P_motor,1) 



step(sys_cl,t)
pzmap(sys_cl)
[Wn,zeta,poles] = damp(sys_cl);
Mp = exp((-zeta(1)*pi)/sqrt(1-zeta(1)^2))
Ts = 4/(zeta(1)*Wn(1))

rlocus(P_motor)
title('Root Locus - P Control')
sgrid(.5, 0)
sigrid(100)

poles = pole(P_motor)

poles(1), poles(3)

poles = pole(P_motor);
rP_motor = minreal(P_motor*(s/max(abs(poles)) + 1))

pole(rP_motor)

rlocus(rP_motor)
title('Root Locus - P Control')
axis([ -300 100 -200 200])
sgrid(.5, 0)
sigrid(100)

C = 1/s;
rlocus(C*rP_motor)
title('Root Locus - I Control')
axis([ -300 100 -200 200])
sgrid(.5, 0)
sigrid(100)

C = (s + 20) / s;
rlocus(C*rP_motor)
title('Root Locus - PI Control')
axis([ -300 100 -200 200])
sgrid(.5, 0)
sigrid(100)

C = (s + 60)*(s + 70) / s;
rlocus(C*rP_motor)
title('Root Locus - PID Control')
axis([ -300 100 -200 200])
sgrid(.5, 0)
sigrid(100)

rsys_ol = minreal(C*rP_motor, 0.1);
rlocus(rsys_ol)
title('Root Locus - PID Control')
axis([ -300 100 -200 200])
sgrid(.5, 0)
sigrid(100)

[k,poles] = rlocfind(rsys_ol)

figure

sys_cl = feedback(k*rsys_ol,1);
t = 0:0.0001:0.1;
step(sys_cl, t)
grid
ylabel('Position, \theta (radians)')
title('Response to a Step Reference with PID Control')

figure

dist_cl = feedback(P_motor,k*C);
         figure;step(dist_cl, t)
         grid
         ylabel('Position, \theta (radians)')
         title('Response to a Step Disturbance with PID Control')
%Initialize API

sim=remApi('remoteApi');

% using the prototype file (remoteApiProto.m)

sim.simxFinish(-1);

% just in case, close all opened connections

clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)       

disp('Connected to remote API server');       

sim.simxGetStringSignal(clientID,'distance',sim.simx_opmode_streaming);               

set_param('Controls_Project', 'SimulationCommand', 'start')               

    while(1)  % In this while loop, we will have the communication

        [errorCode,r_mat]=sim.simxGetStringSignal(clientID,'distance',sim.simx_opmode_buffer);           

        %%if errorCode is not vrep.simx_return_ok, this does not mean there is an error:            

       %%it could be that the first streamed values have not yet arrived, or that the signal            

       %%is empty/non-existent           

            set_param('Controls_Project/Constant','Value',num2str(r_mat));  %ballclmod is the model file and Constant is the block's name, r_mat is the variable to send.      

            pause(.01);

            theta = get_param('Controls_Project/To Workspace','RuntimeObject'); % We receive the sensor data from Simulink model ballclmod and To Workspace block via RuntimeObject

            theta.InputPort(1).Data;                                                                      % Receive the data

   end
end



