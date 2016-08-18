function trial2()
clear
pkg load symbolic;
global l1=93 l2=80 l3=81 l4=172 x y z
  position(5)=0;
  %theta(5)=0;
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
%      x=input("Enter x-coorinate: ");
%      y=input("Enter y-coorinate: ");
%      z=input("Enter z-coorinate: ");
      my_fun_fk(position);
      x
      y
      z
      P1 = [x,y,z];
      x=P1(1);
      y=P1(2);
      z=P1(3);
      [theta, fval, info] = fsolve (@my_fun_ik, [0 0 0 0]);
      if abs(fval) < 1e-3 && z > 0
        if theta(1) > pi/2
          theta(1) = -pi + theta(1);
          theta(2) = - theta(2);
          theta(3) = - theta(3);
          theta(4) = - theta(4);
        elseif theta(1) < -pi/2
          theta(1) = pi + theta(1);
          theta(2) = - theta(2);
          theta(3) = - theta(3);
          theta(4) = - theta(4);
        endif
        theta_deg = theta*180/pi;
        simxPauseCommunication(clientID,1);
        returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(1), theta(1), vrep.simx_opmode_oneshot);
        returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(2), theta(2), vrep.simx_opmode_oneshot);
        returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(3), theta(3), vrep.simx_opmode_oneshot);
        returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(4), theta(4), vrep.simx_opmode_oneshot);
        simxPauseCommunication(clientID,0);
        %val1 = input('Enter 0 to close gripper and 1 to open gripper');
        val1 = 0;
        returnCode = simxSetIntegerSignal(clientID, 'RG2_open', val1, vrep.simx_opmode_oneshot);
        simxGetPingTime(clientID);
      else
        fprintf('Could not find soulution to given co-ordinates, please check \n')
      endif
      pause(10)
      d = 0.5;
      P2 = [125,-125,125];
      P3 = (P2-P1)/sqrt(dot(P2-P1, P2-P1));
      syms x1 y1 z1
      P = [x1,y1,z1];
      %plane_1 = dot(normal, P-P1);
      %cir_1 = dot(P-(P1+P2)/2, P-(P1+P2)/2) - dot(P1-(P1+P2)/2, P1-(P1+P2)/2);
      while (dot([x,y,z]-P1, [x,y,z]-P1)<dot(P1-P2, P1-P2))
        x=x+P3(1)*d;
        y=y+P3(2)*d;
        z=z+P3(3)*d;
        [theta, fval, info] = fsolve (@my_fun_ik, [0 0 0 0]);
        if abs(fval) < 1e-3 && z > 0
          if theta(1) > pi/2
            theta(1) = -pi + theta(1);
            theta(2) = - theta(2);
            theta(3) = - theta(3);
            theta(4) = - theta(4);
          elseif theta(1) < -pi/2
            theta(1) = pi + theta(1);
            theta(2) = - theta(2);
            theta(3) = - theta(3);
            theta(4) = - theta(4);
          endif
          theta_deg = theta*180/pi;
          simxPauseCommunication(clientID,1);
          returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(1), theta(1), vrep.simx_opmode_oneshot);
          returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(2), theta(2), vrep.simx_opmode_oneshot);
          returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(3), theta(3), vrep.simx_opmode_oneshot);
          returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(4), theta(4), vrep.simx_opmode_oneshot);
          simxPauseCommunication(clientID,0);
          %val1 = input('Enter 0 to close gripper and 1 to open gripper');
          %val1 = 0;
          %returnCode = simxSetIntegerSignal(clientID, 'RG2_open', val1, vrep.simx_opmode_oneshot);
          simxGetPingTime(clientID);
        else
          fprintf('Could not find soulution to given co-ordinates, please check \n')
        endif
        pause(0.1);
        %val2 = input('Enter 1 to reach new location and 0 to end program');
      endwhile     
      val2 = input('Enter 1 to reach new location and 0 to end program');
    endwhile
	  simxFinish(clientID);    
	else
		disp('Failed connecting to remote API server, please check if v-rep is running');
	endif
	disp('Program ended');
end
function fun1 = my_fun_ik (theta)
  global l1 l2 l3 l4 x y z
  fun1 = zeros (3, 1);
	r=sqrt(x^2 + y^2);
  fun1(1) = l1 + l2*cos(theta(2)) + l3*cos(theta(2)+theta(3)) + l4*cos(theta(2)+theta(3)+theta(4)) - z;
	fun1(2) = l2*sin(theta(2)) + l3*sin(theta(2)+theta(3)) + l4*sin(theta(2)+theta(3)+theta(4))- r;
	fun1(3) = theta(1) - atan2(y,x);
endfunction
function fun1 = my_fun_fk (theta)
  global l1 l2 l3 l4 x y z
	r = l2*sin(theta(2)) + l3*sin(theta(2)+theta(3)) + l4*sin(theta(2)+theta(3)+theta(4));
  x = r*cos(theta(1));
  y = r*sin(theta(1));
  z = l1 + l2*cos(theta(2)) + l3*cos(theta(2)+theta(3)) + l4*cos(theta(2)+theta(3)+theta(4));
endfunction