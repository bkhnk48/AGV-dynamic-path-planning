function [point,goal] = Createorderform()
%Generate orders
point=[];
global storagepoint;
global target;
N=size(storagepoint,1);
n=size(target,1);
nodenumber=unidrnd(3);          %Number of mission points generated
printf("Node number = %d\n", nodenumber);

task=randperm(N,nodenumber);
for i=1:nodenumber
    point(i,:)=storagepoint(task(i),:);
end

goalind=unidrnd(n);
goal=target(goalind,1:2);

end
