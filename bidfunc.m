function [finalpath,ind,xstate,dis] = bidfunc(x,xstate,xpath,point,goal,duration,trajp,pn,path0,dis)
%Bidding, choose the robot with the lowest cost, and return the path at the same time
minb=999999;
global v;

for i=1:size(x,1)
    taskp=[];
    if xstate(i)==0                         %When the robot currently has no task
        [finalpath0,d,dis] = getpath(x(i,:),point,goal,trajp,path0,dis);
        [flow] = preditflow(finalpath0,i,x,xpath);
        
        m=caculatett(d,v)+duration*size(point,1);
        tt=caculatett(d,v);
        bid=0.5*m+0.5*tt+15*flow;
    else                                       
        for j=1:size(xpath,1)
            if xpath(j,2*i-1)~=0||xpath(j,2*i)~=0
                taskp=[taskp;xpath(j,2*i-1:2*i)];
            else
                break;
            end
        end
        
        d1=norm(x(i,:)-taskp(1,:));
        for k=1:size(taskp,1)-1
            d1=d1+norm(taskp(k,:)-taskp(k+1,:));
        end
        [finalpath2,d2,dis] = getpath(taskp(size(taskp,1),:),point,goal,trajp,path0,dis);
        d=d1+d2;
        
        finalpath0=[taskp;finalpath2];
        [flow] = preditflow(finalpath0,i,x,xpath);
        m=caculatett(d,v)+duration*(pn(i)+size(point,1));
        tt=caculatett(d,v);
        bid=0.5*m+0.5*tt+15*flow;
    end
    if bid<minb
        minb=bid;
        finalpath=finalpath0;
        ind=i;
    end
    
end
xstate(ind)=1;
