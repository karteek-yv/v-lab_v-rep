
% Make sure to have the server side running in V-REP: 
% in a child script of a V-REP scene, add following command
% to be executed just once, at simulation start:
%
% simExtRemoteApiStart(19999)
%
% then start simulation, and run this program.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!

%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% Copyright 2006-2016 Coppelia Robotics GmbH. All rights reserved. 
% marc@coppeliarobotics.com
% www.coppeliarobotics.com
% 
% -------------------------------------------------------------------
% THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
% WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
% AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
% DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
% MISUSING THIS SOFTWARE.
% 
% You are free to use/modify/distribute this file for whatever purpose!
% -------------------------------------------------------------------
%
% This file was automatically created for V-REP release V3.3.0 on February 19th 2016
%
% This file is modified by Karteek (yanumula@iitg.ernet.in , karteek.yv@gmail.com)
% and redistributed under GNU-GPL-V3.0

function DaguArmControl_FK()
  global l1=93 l2=80 l3=81 l4=172 x y z
  theta(5)=0;
  position(5)=0;
  disp('Octave remote API simulation started');
	vrep=remApiSetup();
	simxFinish(-1); % just in case, close all opened connections
	clientID=simxStart('127.0.0.1',19999,true,true,5000,5);
	if (clientID>-1)
		disp('Connected to remote API server');
    for i = 1:5
      [returnCode,shaft_handles] = simxGetObjectHandle(clientID,strcat('Servo_shaft_',mat2str(i)),vrep.simx_opmode_blocking);
      shaft_handles_list(i)=shaft_handles;
    endfor
    for i = 1:5
      [returnCode, position(i)] = simxGetJointPosition(clientID,shaft_handles_list(i),vrep.simx_opmode_streaming);
    endfor
    simxGetPingTime(clientID);
    returnCode = simxSetIntegerSignal(clientID, 'RG2_open', 1, vrep.simx_opmode_oneshot);
    val2=1;
    while(val2)    
      for i = 1:4
        [returnCode, position(i)] = simxGetJointPosition(clientID,shaft_handles_list(i),vrep.simx_opmode_buffer);
      endfor
      display('Please enter the angles in the range of [-90, +90] ');
      theta(1) = input('Enter theta1 in degrees: ');
      theta(2) = input('Enter theta2 in degrees: ');
      theta(3) = input('Enter theta3 in degrees: ');
      theta(4) = input('Enter theta4 in degrees: ');
      %theta(5) = input('Enter theta5 in degrees: ');
      theta = theta * pi/180;
      my_fun(theta);
      x
      y
      z
      simxPauseCommunication(clientID,1);
      returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(1), theta(1), vrep.simx_opmode_oneshot);
      returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(2), theta(2), vrep.simx_opmode_oneshot);
      returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(3), theta(3), vrep.simx_opmode_oneshot);
      returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(4), theta(4), vrep.simx_opmode_oneshot);
      simxPauseCommunication(clientID,0);
      %val1 = input('Enter 0 to close gripper and 1 to open gripper');
      val1 = 0;
      returnCode = simxSetIntegerSignal(clientID, 'RG2_open', val1, vrep.simx_opmode_oneshot)
      val2 = input('Enter 1 to reach new location and 0 to end program');
    endwhile
	  simxFinish(clientID);    
	else
		disp('Failed connecting to remote API server, please check if v-rep is running');
	endif
	disp('Program ended');
end
function fun1 = my_fun (theta)
  global l1 l2 l3 l4 x y z
	r = l2*sin(theta(2)) + l3*sin(theta(2)+theta(3)) + l4*sin(theta(2)+theta(3)+theta(4));
  x = r*cos(theta(1));
  y = r*sin(theta(1));
  z = l1 + l2*cos(theta(2)) + l3*cos(theta(2)+theta(3)) + l4*cos(theta(2)+theta(3)+theta(4));
endfunction
