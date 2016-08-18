function trial3()
  clear
  pkg load symbolic;
  Link_len = [93,80,81,172];
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
    returnCode = simxSetIntegerSignal(clientID, 'RG2_open', 0, vrep.simx_opmode_oneshot);
    val2=1;
    while(val2)    
      for i = 1:4
        [returnCode, position(i)] = simxGetJointPosition(clientID,shaft_handles_list(i),vrep.simx_opmode_buffer);
      endfor
      P1=my_fun_fk(position,Link_len);
      disp(P1);
      d = 0.5;
      P2 = P1;
      P3 = [0,0,0];
      P4 = [0,0,0];
      P2(1)=input('Enter target x-coorinate: ');
      P2(2)=input('Enter target y-coorinate: ');
      P2(3)=input('Enter target z-coorinate: ');
      Obst_pos = [0,0,0];
%      Obst_size = 0;
%      Obst_pos(1)=input('Enter obstacle x-coorinate: ');
%      Obst_pos(2)=input('Enter obstacle y-coorinate: ');
%      Obst_pos(3)=input('Enter obstacle z-coorinate: ');
%      Obst_size = input('Enter obstacle size: ');
      Obst_pos = [200,0,60];
      Obst_size = 115;
      syms x1 y1 z1
      P = [x1,y1,z1];
      %plane_1 = dot(normal, P-P1);
      %cir_1 = dot(P-(P1+P2)/2, P-(P1+P2)/2) - dot(P1-(P1+P2)/2, P1-(P1+P2)/2);
      target = P1;
      while (dot(target-P1, target-P1)<dot(P1-P2, P1-P2))
        Unit_vec = (P2-target)/sqrt(dot(P2-target, P2-target)); %unit vector in the direction
        target=target+Unit_vec*d;
        if (dot(target-Obst_pos, target-Obst_pos) < Obst_size^2)
          Z_vec = [0,0,1];
          normal = cross(target-Obst_pos, Z_vec);
          plane_1 = dot(normal, P-target);
          cir_1 = dot(P-Obst_pos, P-Obst_pos) - dot(target-Obst_pos, target-Obst_pos);
          dth=0.5; %theta step size in degrees
          P1=target;
          P2=target+Unit_vec*d;
          while(dot(P2-Obst_pos, P2-Obst_pos) < Obst_size^2)
            P2=target+Unit_vec*d;
          endwhile
          ang_1=180/pi*abs(sign(dot(Z_vec-P1,P2-Z_vec))*atan2(norm(cross(P1,Z_vec)),dot(P1,Z_vec))+atan2(norm(cross(P2,Z_vec)),dot(P2,Z_vec)));
          while(ang_1>0.5)
            fun_1 = dot(P-P1, P-P1) - dot(P1-Obst_pos, P1-Obst_pos)*2*(1-cos(dth*pi/180));
            s=solve(plane_1==0,cir_1==0,fun_1==0);
            P3(1)=double(s{1}.x1);P3(2)=double(s{1}.y1);P3(3)=double(s{1}.z1);
            P4(1)=double(s{2}.x1);P4(2)=double(s{2}.y1);P4(3)=double(s{2}.z1);
            ang_3=180/pi*abs(sign(dot(Z_vec-P3,P2-Z_vec))*atan2(norm(cross(P3,Z_vec)),dot(P3,Z_vec))+atan2(norm(cross(P2,Z_vec)),dot(P2,Z_vec)));
            ang_4=180/pi*abs(sign(dot(Z_vec-P4,P2-Z_vec))*atan2(norm(cross(P4,Z_vec)),dot(P4,Z_vec))+atan2(norm(cross(P2,Z_vec)),dot(P2,Z_vec)));
            if(ang_3<ang_4)
              P1=P3;
            else
              P1=P4;
            endif
            [theta, fval, info] = fsolve (@(theta)my_fun_ik(theta,P1,Link_len), [0 0 0 0]);            
            ang_1=180/pi*abs(sign(dot(Z_vec-P1,P2-Z_vec))*atan2(norm(cross(P1,Z_vec)),dot(P1,Z_vec))+atan2(norm(cross(P2,Z_vec)),dot(P2,Z_vec)));
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
              %fprintf('Could not find soulution to given co-ordinates, please check \n')
              break;
            endif
            pause(0.1);
          endwhile
          target=P2;
        endif
        [theta, fval, info] = fsolve (@(theta)my_fun_ik(theta,target,Link_len), [0 0 0 0]);
        if abs(fval) < 1e-3 && target(3) > 0
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
endfunction
function fun1 = my_fun_ik (theta,target,Link_len)
  fun1 = zeros (3, 1);
	r=sqrt((target(1))^2 + (target(2))^2);
  fun1(1) = Link_len(1) + Link_len(2)*cos(theta(2)) + Link_len(3)*cos(theta(2)+theta(3)) + Link_len(4)*cos(theta(2)+theta(3)+theta(4)) - target(3);
	fun1(2) = Link_len(2)*sin(theta(2)) + Link_len(3)*sin(theta(2)+theta(3)) + Link_len(4)*sin(theta(2)+theta(3)+theta(4))- r;
	fun1(3) = theta(1) - atan2(target(2),target(1));
endfunction
function fun2 = my_fun_fk (theta,Link_len)
	r = Link_len(2)*sin(theta(2)) + Link_len(3)*sin(theta(2)+theta(3)) + Link_len(4)*sin(theta(2)+theta(3)+theta(4));
  fun2(1) = r*cos(theta(1));
  fun2(2) = r*sin(theta(1));
  fun2(3) = Link_len(1) + Link_len(2)*cos(theta(2)) + Link_len(3)*cos(theta(2)+theta(3)) + Link_len(4)*cos(theta(2)+theta(3)+theta(4));
  return;
endfunction