close all;
clear all;
pkg load statistics;

load('./data/trajp.mat') % Track point of the storage environment
%luu tru cac diem ma AGV co the di qua.
%trajp stores reachable points for AGV
%dlmwrite('trajp.txt', trajp, 'delimiter','\t','newline','pc'); => How to export .mat to txt 
%load('./data/x.mat')    %100 initial positions of robots
x = importdata('x.txt') ;
%luu tru cac diem ma AGV co the xuat phat
%x stores points where AGVs would start
load('./data/storagepoint.mat')   %Corresponding working point on the shelf
%You could read the meaning of storagepoint at: 
%https://github.com/bkhnk48/AGV-dynamic-path-planning/issues/3#issue-1061956103
load('./data/storagerock.mat')     %Shelf position
%meaning of storagerock:
%https://github.com/bkhnk48/AGV-dynamic-path-planning/issues/3#issuecomment-977472672
%load('./data/target.mat')          %Final goal point
%meaning of target:
%https://github.com/bkhnk48/AGV-dynamic-path-planning/issues/3#issuecomment-977492322
target = importdata('target.txt') ;
load('./data/stopp.mat')           
%Corresponding to the target point, the stopping point in each robot station
%meaning of stopp:
%https://github.com/bkhnk48/AGV-dynamic-path-planning/issues/3#issuecomment-977555334

xpath=zeros(1500,200);%The remaining tasks used to store the current order of the robot
xstate=zeros(1,100); %Used to indicate the status of the robot, 0 has not accepted the order 1 is executing the order
state=zeros(1,100); %Used to judge whether the robot has reached the task point and is ready to perform the task
xpoint=zeros(3,200);
xorder=zeros(1500,200); %?Order task matrix, used to store the current path of all robots

dt=0.1;
global v
v = 0.5;
duration = 1; %Robot working hours
grid on
[t,dis,path] = Floyd1(trajp); %Generate the adjacency matrix of the graph
time=0;
worktime=zeros(1,size(x,1));%Record the time that the robot has been working at the current working point
[point,goal] = Createorderform(target);    %Generate order
num=1;
pn=zeros(1,100)
n=ones(size(x,1),1);  %Used to record which node the robot has reached on its current path
printf("size(x, 1) = %d\n", size(x, 1));
t=0;
%%
while 1
    if num<2                              %Generate an order every -s
        [point,goal] = Createorderform(target);    %Generate order
        %         point=[3 3.5]
        %          goal=[4.5 2]
        num=num+1;
        [crashstate,crash] = robotavoid(x,xstate,state,xorder,xpath,xpoint,n,pn,target,stopp,trajp,path,dis);
        [finalpath,ind,xstate] = bidfunc(x,xstate,xpath,point,goal,duration,trajp,pn,path,dis);    %Bidding, giving the path to the lowest cost car
        printf("num = %d, ind = %d\n", num, ind);
        xorder(:,2*ind-1:2*ind)=zeros(1500,2);
        
        n(ind)=1;
        xorder(1:size(finalpath,1),2*ind-1:2*ind)=finalpath;
        xpath(1:size(finalpath,1),2*ind-1:2*ind)=finalpath;
        xpoint(:,2*ind-1:2*ind)=[point;zeros(3-size(point,1),2)];
        pn(ind)=size(point,1);
        time=0;
    end
    while 1
        
        hold off      
        [crashstate,crash] = robotavoid(x,xstate,state,xorder,xpath,xpoint,n,pn,target,stopp,trajp,path,dis);     
        [x,xstate,state,xpath,n,pn,xorder] = Executetask(x,xstate,state,xorder,xpath,xpoint,n,pn,crash,target,stopp,trajp,path,dis); %Perform task
        time=time+dt;
        t=t+dt;
        worktime;
        plot(finalpath(:,1),finalpath(:,2),'--');hold on;
        plot(x(:,1),x(:,2),'ro','MarkerFaceColor','r');hold on;
        plot(storagerock(:,1),storagerock(:,2),'sk');hold on;
        simenvironment(target)
        plot(target(:,1),target(:,2),'xr');hold on;
        grid on
        drawnow;
        for j=1:size(x,1)
            if state(j)==1                     %If the robot reaches the working point, start timing until the end of the task time
                worktime(j)=worktime(j)+dt;
            end
            if worktime(j)>duration
                worktime(j)=0;
                state(j)=0;
            end
        end
        if time>2
            break;
        end
        
    end
    if num == 2 &&(~any(xstate))
    %Note: the any(xstate) returns 1 if there is at least one non-zero value in xstate
    %in case all elements of xstate are zero, the any returns 0. The ~any(xstate)
    % returns 1 then.
        break;
    end
    
end
