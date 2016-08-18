
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

function DaguArmControl_TP()
  clear 
  pkg load symbolic;
  pos_buf=zeros(3,10000);
  ang_buf=zeros(4,10000);
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
%    for i = 1:5
%      [returnCode, position(i)] = simxGetJointPosition(clientID,shaft_handles_list(i),vrep.simx_opmode_streaming);
%    endfor
    simxGetPingTime(clientID);
    returnCode = simxSetIntegerSignal(clientID, 'RG2_open', 0, vrep.simx_opmode_oneshot);
    %Intl_P=my_fun_fk(position(1:4),Link_len);
    %[theta, fval, info] = fsolve (@(theta)my_fun_ik(theta,Intl_P,Link_len), [0 0 0 0]); 
    %theta=position;
    val2=1;
    d = 5; %displacement step size in mm
    %dth=5; %theta step size in degrees
    j=1;
    while(val2)  
      for i = 1:5
        [returnCode, position(i)] = simxGetJointPosition(clientID,shaft_handles_list(i),vrep.simx_opmode_streaming);
      endfor
      j=1; 
      Intl_P=my_fun_fk(position(1:4),Link_len);
      disp(Intl_P);
      P1=Intl_P;
      P2 = P1;
      P3 = [0,0,0];
      P4 = [0,0,0];    
      Fnl_p=[0,0,0];
      Fnl_P(1)=input('Enter target x-coorinate: ');
      Fnl_P(2)=input('Enter target y-coorinate: ');
      Fnl_P(3)=input('Enter target z-coorinate: ');
      %Fnl_P=[150,-150,125];
      Obst_pos = [0,0,0];
      Obst_size = 0;
      Obst_pos = [200,0,60];
      Obst_size = 120;
      
      %Obst_pos(1)=input('Enter obstacle x-coorinate: ');
      %Obst_pos(2)=input('Enter obstacle y-coorinate: ');
      %Obst_pos(3)=input('Enter obstacle z-coorinate: ');
      %Obst_size = input('Enter obstacle size: ');
      syms x1 y1 z1 real
      P = [x1,y1,z1];
      %plane_1 = dot(normal, P-P1);
      %cir_1 = dot(P-(P1+P2)/2, P-(P1+P2)/2) - dot(P1-(P1+P2)/2, P1-(P1+P2)/2);
      target = Intl_P;
      tic();
      if sign(Intl_P(1)) == sign(Fnl_P(1))              
        while (dot(target-Intl_P, target-Intl_P)<dot(Intl_P-Fnl_P, Intl_P-Fnl_P))
          Unit_vec = (Fnl_P-target)/norm(Fnl_P-target); %unit vector in the direction
          target=target+Unit_vec*d;
          if (dot(target-Obst_pos, target-Obst_pos) < Obst_size^2)
            dth=d/Obst_size;
            Z_vec = [0,0,1];
            cir_1 = dot(P-Obst_pos, P-Obst_pos) - dot(target-Obst_pos, target-Obst_pos);
            P1=target-Unit_vec*d;
            P2=target;
            while(dot(P2-Obst_pos, P2-Obst_pos) < Obst_size^2)
              P2=P2+Unit_vec*d;
            endwhile
            normal = cross(P1-P2, Z_vec);
            plane_1 = dot(normal, P-target);
            Unit_vec2=(P2-[P1(1),P1(2),P2(3)])/norm(P2-[P1(1),P1(2),P2(3)]);
            ang_1=abs(dot([Obst_pos(1),Obst_pos(2),P1(3)]-P1,Unit_vec2)/norm([Obst_pos(1),Obst_pos(2),P1(3)]-P1)*atan2(norm(cross((P1-Obst_pos),Z_vec)),dot((P1-Obst_pos),Z_vec))+atan2(norm(cross((P2-Obst_pos),Z_vec)),dot((P2-Obst_pos),Z_vec)));
            ang_2=atan2(norm(cross(Obst_pos-P1, Fnl_P-P1)),dot(Obst_pos-P1, Fnl_P-P1));
            while(ang_1>dth && ang_2 < pi/2)
              fun_1 = dot(P-P1, P-P1) - dot(P1-Obst_pos, P1-Obst_pos)*2*(1-cos(dth));
              %tic();
              s=solve(plane_1==0,cir_1==0,fun_1==0);
              %toc();
              %P1
              %P2
              P3=[double(s{1}.x1),double(s{1}.y1),double(s{1}.z1)];
              P4=[double(s{2}.x1),double(s{2}.y1),double(s{2}.z1)];
              ang_3=abs(dot([Obst_pos(1),Obst_pos(2),P3(3)]-P3,Unit_vec2)/norm([Obst_pos(1),Obst_pos(2),P3(3)]-P3)*atan2(norm(cross((P3-Obst_pos),Z_vec)),dot((P3-Obst_pos),Z_vec))+atan2(norm(cross((P2-Obst_pos),Z_vec)),dot((P2-Obst_pos),Z_vec)));
              ang_4=abs(dot([Obst_pos(1),Obst_pos(2),P4(3)]-P4,Unit_vec2)/norm([Obst_pos(1),Obst_pos(2),P4(3)]-P4)*atan2(norm(cross((P4-Obst_pos),Z_vec)),dot((P4-Obst_pos),Z_vec))+atan2(norm(cross((P2-Obst_pos),Z_vec)),dot((P2-Obst_pos),Z_vec)));
              if(ang_3<ang_4)
                P1=P3;          
              else
                P1=P4;
              endif
              [theta, fval, info] = fsolve (@(theta)my_fun_ik(theta,P1,Link_len), [0 0 0 0]);     
              ang_1=abs(dot([Obst_pos(1),Obst_pos(2),P1(3)]-P1,Unit_vec2)/norm([Obst_pos(1),Obst_pos(2),P1(3)]-P1)*atan2(norm(cross((P1-Obst_pos),Z_vec)),dot((P1-Obst_pos),Z_vec))+atan2(norm(cross((P2-Obst_pos),Z_vec)),dot((P2-Obst_pos),Z_vec)));
              ang_2=atan2(norm(cross(Obst_pos-P1, Fnl_P-P1)),dot(Obst_pos-P1, Fnl_P-P1));
              if abs(fval) < 1e-3 && P1(3) > 0
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
                pos_buf(:,j)=P1;ang_buf(:,j)=theta;j++;
              else
                %fprintf('Could not find soulution to given co-ordinates, please check \n')
                break;
              endif
              %pause(0.1);
            endwhile
            target=P1;
            Unit_vec = (Fnl_P-target)/norm(Fnl_P-target); %unit vector in the direction
            target=P1+Unit_vec*d;
          endif
          %tic();
          [theta, fval, info] = fsolve (@(theta)my_fun_ik(theta,target,Link_len), [0 0 0 0]);
          %toc();
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
            pos_buf(:,j)=target;ang_buf(:,j)=theta;j++;
          else
            fprintf('Could not find soulution to given co-ordinates, please check \n')
          endif
          %pause(0.1);
          %val2 = input('Enter 1 to reach new location and 0 to end program');
        endwhile
      else 
        Fnl_P_temp = Fnl_P;
        Fnl_P = [0,0,426];
        while (dot(target-Intl_P, target-Intl_P)<dot(Intl_P-Fnl_P, Intl_P-Fnl_P))
          Unit_vec = (Fnl_P-target)/norm(Fnl_P-target); %unit vector in the direction
          target=target+Unit_vec*d;
          if (dot(target-Obst_pos, target-Obst_pos) < Obst_size^2)
            dth=d/Obst_size;
            Z_vec = [0,0,1];
            cir_1 = dot(P-Obst_pos, P-Obst_pos) - dot(target-Obst_pos, target-Obst_pos);
            P1=target-Unit_vec*d;
            P2=target;
            while(dot(P2-Obst_pos, P2-Obst_pos) < Obst_size^2)
              P2=P2+Unit_vec*d;
            endwhile
            normal = cross(P1-P2, Z_vec);
            plane_1 = dot(normal, P-target);
            Unit_vec2=(P2-[P1(1),P1(2),P2(3)])/norm(P2-[P1(1),P1(2),P2(3)]);
            ang_1=abs(dot([Obst_pos(1),Obst_pos(2),P1(3)]-P1,Unit_vec2)/norm([Obst_pos(1),Obst_pos(2),P1(3)]-P1)*atan2(norm(cross((P1-Obst_pos),Z_vec)),dot((P1-Obst_pos),Z_vec))+atan2(norm(cross((P2-Obst_pos),Z_vec)),dot((P2-Obst_pos),Z_vec)));
            ang_2=atan2(norm(cross(Obst_pos-P1, Fnl_P-P1)),dot(Obst_pos-P1, Fnl_P-P1));
            while(ang_1>dth && ang_2 < pi/2)
              fun_1 = dot(P-P1, P-P1) - dot(P1-Obst_pos, P1-Obst_pos)*2*(1-cos(dth));
              %tic();
              s=solve(plane_1==0,cir_1==0,fun_1==0);
              %toc();
              %P1
              %P2
              P3=[double(s{1}.x1),double(s{1}.y1),double(s{1}.z1)];
              P4=[double(s{2}.x1),double(s{2}.y1),double(s{2}.z1)];
              ang_3=abs(dot([Obst_pos(1),Obst_pos(2),P3(3)]-P3,Unit_vec2)/norm([Obst_pos(1),Obst_pos(2),P3(3)]-P3)*atan2(norm(cross((P3-Obst_pos),Z_vec)),dot((P3-Obst_pos),Z_vec))+atan2(norm(cross((P2-Obst_pos),Z_vec)),dot((P2-Obst_pos),Z_vec)));
              ang_4=abs(dot([Obst_pos(1),Obst_pos(2),P4(3)]-P4,Unit_vec2)/norm([Obst_pos(1),Obst_pos(2),P4(3)]-P4)*atan2(norm(cross((P4-Obst_pos),Z_vec)),dot((P4-Obst_pos),Z_vec))+atan2(norm(cross((P2-Obst_pos),Z_vec)),dot((P2-Obst_pos),Z_vec)));
              if(ang_3<ang_4)
                P1=P3;          
              else
                P1=P4;
              endif
              [theta, fval, info] = fsolve (@(theta)my_fun_ik(theta,P1,Link_len), [0 0 0 0]);     
              ang_1=abs(dot([Obst_pos(1),Obst_pos(2),P1(3)]-P1,Unit_vec2)/norm([Obst_pos(1),Obst_pos(2),P1(3)]-P1)*atan2(norm(cross((P1-Obst_pos),Z_vec)),dot((P1-Obst_pos),Z_vec))+atan2(norm(cross((P2-Obst_pos),Z_vec)),dot((P2-Obst_pos),Z_vec)));
              ang_2=atan2(norm(cross(Obst_pos-P1, Fnl_P-P1)),dot(Obst_pos-P1, Fnl_P-P1));
              if abs(fval) < 1e-3 && P1(3) > 0
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
                pos_buf(:,j)=P1;ang_buf(:,j)=theta;j++;
              else
                %fprintf('Could not find soulution to given co-ordinates, please check \n')
                break;
              endif
              %pause(0.1);
            endwhile
            target=P1;
            Unit_vec = (Fnl_P-target)/norm(Fnl_P-target); %unit vector in the direction
            target=P1+Unit_vec*d;
          endif
          %tic();
          [theta, fval, info] = fsolve (@(theta)my_fun_ik(theta,target,Link_len), [0 0 0 0]);
          %toc();
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
            pos_buf(:,j)=target;ang_buf(:,j)=theta;j++;
          else
            fprintf('Could not find soulution to given co-ordinates, please check \n')
          endif
          %pause(0.1);
          %val2 = input('Enter 1 to reach new location and 0 to end program');
        endwhile
        Intl_P = Fnl_P;
        Fnl_P = Fnl_P_temp;
        while (dot(target-Intl_P, target-Intl_P)<dot(Intl_P-Fnl_P, Intl_P-Fnl_P))
          Unit_vec = (Fnl_P-target)/norm(Fnl_P-target); %unit vector in the direction
          target=target+Unit_vec*d;
          if (dot(target-Obst_pos, target-Obst_pos) < Obst_size^2)
            dth=d/Obst_size;
            Z_vec = [0,0,1];
            cir_1 = dot(P-Obst_pos, P-Obst_pos) - dot(target-Obst_pos, target-Obst_pos);
            P1=target-Unit_vec*d;
            P2=target;
            while(dot(P2-Obst_pos, P2-Obst_pos) < Obst_size^2)
              P2=P2+Unit_vec*d;
            endwhile
            normal = cross(P1-P2, Z_vec);
            plane_1 = dot(normal, P-target);
            Unit_vec2=(P2-[P1(1),P1(2),P2(3)])/norm(P2-[P1(1),P1(2),P2(3)]);
            ang_1=abs(dot([Obst_pos(1),Obst_pos(2),P1(3)]-P1,Unit_vec2)/norm([Obst_pos(1),Obst_pos(2),P1(3)]-P1)*atan2(norm(cross((P1-Obst_pos),Z_vec)),dot((P1-Obst_pos),Z_vec))+atan2(norm(cross((P2-Obst_pos),Z_vec)),dot((P2-Obst_pos),Z_vec)));
            ang_2=atan2(norm(cross(Obst_pos-P1, Fnl_P-P1)),dot(Obst_pos-P1, Fnl_P-P1));
            while(ang_1>dth && ang_2 < pi/2)
              fun_1 = dot(P-P1, P-P1) - dot(P1-Obst_pos, P1-Obst_pos)*2*(1-cos(dth));
              %tic();
              s=solve(plane_1==0,cir_1==0,fun_1==0);
              %toc();
              %P1
              %P2
              P3=[double(s{1}.x1),double(s{1}.y1),double(s{1}.z1)];
              P4=[double(s{2}.x1),double(s{2}.y1),double(s{2}.z1)];
              ang_3=abs(dot([Obst_pos(1),Obst_pos(2),P3(3)]-P3,Unit_vec2)/norm([Obst_pos(1),Obst_pos(2),P3(3)]-P3)*atan2(norm(cross((P3-Obst_pos),Z_vec)),dot((P3-Obst_pos),Z_vec))+atan2(norm(cross((P2-Obst_pos),Z_vec)),dot((P2-Obst_pos),Z_vec)));
              ang_4=abs(dot([Obst_pos(1),Obst_pos(2),P4(3)]-P4,Unit_vec2)/norm([Obst_pos(1),Obst_pos(2),P4(3)]-P4)*atan2(norm(cross((P4-Obst_pos),Z_vec)),dot((P4-Obst_pos),Z_vec))+atan2(norm(cross((P2-Obst_pos),Z_vec)),dot((P2-Obst_pos),Z_vec)));
              if(ang_3<ang_4)
                P1=P3;          
              else
                P1=P4;
              endif
              [theta, fval, info] = fsolve (@(theta)my_fun_ik(theta,P1,Link_len), [0 0 0 0]);     
              ang_1=abs(dot([Obst_pos(1),Obst_pos(2),P1(3)]-P1,Unit_vec2)/norm([Obst_pos(1),Obst_pos(2),P1(3)]-P1)*atan2(norm(cross((P1-Obst_pos),Z_vec)),dot((P1-Obst_pos),Z_vec))+atan2(norm(cross((P2-Obst_pos),Z_vec)),dot((P2-Obst_pos),Z_vec)));
              ang_2=atan2(norm(cross(Obst_pos-P1, Fnl_P-P1)),dot(Obst_pos-P1, Fnl_P-P1));
              if abs(fval) < 1e-3 && P1(3) > 0
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
                pos_buf(:,j)=P1;ang_buf(:,j)=theta;j++;
              else
                %fprintf('Could not find soulution to given co-ordinates, please check \n')
                break;
              endif
              %pause(0.1);
            endwhile
            target=P1;
            Unit_vec = (Fnl_P-target)/norm(Fnl_P-target); %unit vector in the direction
            target=P1+Unit_vec*d;
          endif
          %tic();
          [theta, fval, info] = fsolve (@(theta)my_fun_ik(theta,target,Link_len), [0 0 0 0]);
          %toc();
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
            pos_buf(:,j)=target;ang_buf(:,j)=theta;j++;
          else
            fprintf('Could not find soulution to given co-ordinates, please check \n')
          endif
          %pause(0.1);
          %val2 = input('Enter 1 to reach new location and 0 to end program');
        endwhile
      endif
      toc();
      v=input('Enter any digit to continue');
      tic();
      time_buf=zeros(j-1);
      i=1;
      t1=time();
      time_buf(1)=time()-t1;
      t3=time_buf(1);
      sam_sz = 10;
      if j>2*sam_sz
        while(i<sam_sz)
          simxPauseCommunication(clientID,1);
          returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(1), ang_buf(1,i), vrep.simx_opmode_oneshot);
          returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(2), ang_buf(2,i), vrep.simx_opmode_oneshot);
          returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(3), ang_buf(3,i), vrep.simx_opmode_oneshot);
          returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(4), ang_buf(4,i), vrep.simx_opmode_oneshot);
          simxPauseCommunication(clientID,0);
          t2=0;
          while(t2^2 < sam_sz*(0.01*d)^2/i)
            t2=time()-t1-time_buf(i);
            %t2=time()-t1-t3;
          endwhile
          i++;
          time_buf(i)=t2+time_buf(i-1);
          %t3=time_buf(i);
        endwhile
        while(i<j-sam_sz)
          simxPauseCommunication(clientID,1);
          returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(1), ang_buf(1,i), vrep.simx_opmode_oneshot);
          returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(2), ang_buf(2,i), vrep.simx_opmode_oneshot);
          returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(3), ang_buf(3,i), vrep.simx_opmode_oneshot);
          returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(4), ang_buf(4,i), vrep.simx_opmode_oneshot);
          simxPauseCommunication(clientID,0);
          t2=0;
          while(t2 < 0.01*d)
            t2=time()-t1-time_buf(i);
            %t2=time()-t1-t3;
          endwhile
          i++;
          time_buf(i)=t2+time_buf(i-1);
          %t3=time_buf(i);
        endwhile
        while(i<j)
          simxPauseCommunication(clientID,1);
          returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(1), ang_buf(1,i), vrep.simx_opmode_oneshot);
          returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(2), ang_buf(2,i), vrep.simx_opmode_oneshot);
          returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(3), ang_buf(3,i), vrep.simx_opmode_oneshot);
          returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(4), ang_buf(4,i), vrep.simx_opmode_oneshot);
          simxPauseCommunication(clientID,0);
          t2=0;
          while(t2^2 < sam_sz*(0.01*d)^2/(j-i))
            t2=time()-t1-time_buf(i);
            %t2=time()-t1-t3;
          endwhile
          i++;
          time_buf(i)=t2+time_buf(i-1);
          %t3=time_buf(i);
        endwhile
      else 
        while(i<floor(j/2))
          simxPauseCommunication(clientID,1);
          returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(1), ang_buf(1,i), vrep.simx_opmode_oneshot);
          returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(2), ang_buf(2,i), vrep.simx_opmode_oneshot);
          returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(3), ang_buf(3,i), vrep.simx_opmode_oneshot);
          returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(4), ang_buf(4,i), vrep.simx_opmode_oneshot);
          simxPauseCommunication(clientID,0);
          t2=0;
          while(t2^2 < sam_sz*(0.01*d)^2/i)
            t2=time()-t1-time_buf(i);
            %t2=time()-t1-t3;
          endwhile
          i++;
          time_buf(i)=t2+time_buf(i-1);
          %t3=time_buf(i);
        endwhile
        while(i<j)
          simxPauseCommunication(clientID,1);
          returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(1), ang_buf(1,i), vrep.simx_opmode_oneshot);
          returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(2), ang_buf(2,i), vrep.simx_opmode_oneshot);
          returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(3), ang_buf(3,i), vrep.simx_opmode_oneshot);
          returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(4), ang_buf(4,i), vrep.simx_opmode_oneshot);
          simxPauseCommunication(clientID,0);
          t2=0;
          while(t2^2 < sam_sz*(0.01*d)^2/(j-i))
            t2=time()-t1-time_buf(i);
            %t2=time()-t1-t3;
          endwhile
          i++;
          time_buf(i)=t2+time_buf(i-1);
          %t3=time_buf(i);
        endwhile
      endif
%      while i<j
%        simxPauseCommunication(clientID,1);
%        returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(1), ang_buf(1,i), vrep.simx_opmode_oneshot);
%        returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(2), ang_buf(2,i), vrep.simx_opmode_oneshot);
%        returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(3), ang_buf(3,i), vrep.simx_opmode_oneshot);
%        returnCode = simxSetJointTargetPosition(clientID, shaft_handles_list(4), ang_buf(4,i), vrep.simx_opmode_oneshot);
%        simxPauseCommunication(clientID,0);
%        t2=0;
%        while(t2 < 0.01)
%          t2=time()-t1-i*0.01;
%        endwhile
%        i++;
%        time_buf(i)=t2+time_buf(i-1);
%      endwhile
      toc();
      j-1
      time()-t1
      %time_buf(1)
%      ang_buf(:,1)
%      ang_buf(:,20)
%      ang_buf(:,j-1)  
      figure();
      pos_buf(:,j)=pos_buf(:,j-1);
      plot(time_buf(1:j-1),sqrt(sum((pos_buf(:,2:j)-pos_buf(:,1:j-1)).^2))./(time_buf(2:j)-time_buf(1:j-1)), 'LineWidth',2)
      grid on
      tit = title('Time vs Velocity plot');
      set(tit,  'FontSize', 30)
      xlabel('Time (s)', 'FontSize', 30)
      ylabel('Velocit (mm/s)', 'FontSize', 30)
      xlabh = get(gca,'xlabel');
      set(xlabh,'Position',get(xlabh,'Position') - [0 0 0]);
      ylabh = get(gca,'ylabel');
      set(ylabh,'Position',get(ylabh,'Position') + [0 0 0]);
      ax = gca; 
      fnt_sz=25;
      set(gca,'FontSize',fnt_sz);
      figure();
      plot3(pos_buf(1,1:j-1),pos_buf(2,1:j-1),pos_buf(3,1:j-1),'r','LineWidth',1.5)  
      ax = gca; 
      fnt_sz=25;
      set(gca,'FontSize',fnt_sz);  
      val2 = input('Enter 1 to reach new location and 0 to end program');
    endwhile
    fid=fopen ('myfile.txt', 'w');
    for k=1:j-1
      fprintf(fid,'%f %f %f %f   %f %f %f %f\n',time_buf(k), pos_buf(1,k),pos_buf(2,k),pos_buf(3,k),ang_buf(1,k)*180/pi,ang_buf(2,k)*180/pi, ang_buf(3,k)*180/pi, ang_buf(4,k)*180/pi);
    end
    fclose(fid);
    simxFinish(clientID);
	else
		disp('Failed connecting to remote API server, please check if v-rep is running');
	endif
	disp('Program ended');
end
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