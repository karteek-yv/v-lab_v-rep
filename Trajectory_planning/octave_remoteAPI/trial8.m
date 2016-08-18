clear all;
close all;
t=time();
%pause(0.001);
i=1;
t1(i)=time()-t;
t3=t1(i);
j=50;
%if j>100
%  while(i<50);
%    t2=0;
%    while(t2 < 0.5/i)
%      %t2=time()-t-t1(i);
%      t2=time()-t-t3;
%      %t2=time()-t-i*0.01;
%    endwhile
%    i=i+1;
%    t1(i)=t2+t1(i-1);
%    t3=t1(i);
%  endwhile
%  while(i<j-50);
%    t2=0;
%    while(t2 < 0.01)
%      %t2=time()-t-t1(i);
%      t2=time()-t-t3;
%      %t2=time()-t-i*0.01;
%    endwhile
%    i=i+1;
%    t1(i)=t2+t1(i-1);
%    t3=t1(i);
%  endwhile
%  while(i<j);
%    t2=0;
%    while(t2 < 0.5/(j-i))
%      %t2=time()-t-t1(i);
%      t2=time()-t-t3;
%      %t2=time()-t-i*0.01;
%    endwhile
%    i=i+1;
%    t1(i)=t2+t1(i-1);
%    t3=t1(i);
%  endwhile
%else 
%  while(i<floor(j/2));
%    t2=0;
%    while(t2 < 0.5/i)
%      %t2=time()-t-t1(i);
%      t2=time()-t-t3;
%      %t2=time()-t-i*0.01;
%    endwhile
%    i=i+1;
%    t1(i)=t2+t1(i-1);
%    t3=t1(i);
%  endwhile
%  while(i<j);
%    t2=0;
%    while(t2 < 0.5/(j-i))
%      %t2=time()-t-t1(i);
%      t2=time()-t-t3;
%      %t2=time()-t-i*0.01;
%    endwhile
%    i=i+1;
%    t1(i)=t2+t1(i-1);
%    t3=t1(i);
%  endwhile
%endif

if j>100
  while(i<50);
    t2=0;
    while(t2^2 < 0.005/i)
      %t2=time()-t-t1(i);
      t2=time()-t-t3;
      %t2=time()-t-i*0.01;
    endwhile
    i=i+1;
    t1(i)=t2+t1(i-1);
    t3=t1(i);
  endwhile
  while(i<j-50);
    t2=0;
    while(t2 < 0.01)
      %t2=time()-t-t1(i);
      t2=time()-t-t3;
      %t2=time()-t-i*0.01;
    endwhile
    i=i+1;
    t1(i)=t2+t1(i-1);
    t3=t1(i);
  endwhile
  while(i<j);
    t2=0;
    while(t2^2 < 0.005/(j-i))
      %t2=time()-t-t1(i);
      t2=time()-t-t3;
      %t2=time()-t-i*0.01;
    endwhile
    i=i+1;
    t1(i)=t2+t1(i-1);
    t3=t1(i);
  endwhile
else 
  while(i<floor(j/2));
    t2=0;
    while(t2^2 < 0.005/i)
      %t2=time()-t-t1(i);
      t2=time()-t-t3;
      %t2=time()-t-i*0.01;
    endwhile
    i=i+1;
    t1(i)=t2+t1(i-1);
    t3=t1(i);
  endwhile
  while(i<j);
    t2=0;
    while(t2^2 < 0.005/(j-i))
      %t2=time()-t-t1(i);
      t2=time()-t-t3;
      %t2=time()-t-i*0.01;
    endwhile
    i=i+1;
    t1(i)=t2+t1(i-1);
    t3=t1(i);
  endwhile
endif

%while(i<j);
%  t2=0;
%  while(t2 < 0.02)
%    %t2=time()-t-t1(i);
%    t2=time()-t-t3;
%    %t2=time()-t-i*0.01;
%  endwhile
%  i=i+1;
%  t1(i)=t2+t1(i-1);
%  t3=t1(i);
%endwhile


time()-t
t1;
plot(t1(2:j),1./(t1(2:j)-t1(1:j-1)), 'LineWidth',2)
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
%set(ax,'XTick',linspace(0,1400,15))
%set(ax,'YTick',linspace(0,125,5))
fnt_sz=25;
set(gca,'FontSize',fnt_sz);
%j=0; 
%for i=1:25
%j=j+0.25/i;
%end
%j