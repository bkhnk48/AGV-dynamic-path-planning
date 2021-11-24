function [point,goal] = Createorderform(target)
%Generate orders
point=[];
global storagepoint;
%global target;
N=size(storagepoint,1);
n=size(target,1);
nodenumber=unidrnd(3);          %Number of mission points generated
%list of intermediate working points of AGV
printf("Node number = %d\n", nodenumber);

task=randperm(N,nodenumber);
for i=1:nodenumber
    point(i,:)=storagepoint(task(i),:);
end

goalind=unidrnd(n); %randomize the final point of the AGV in it's journey
%n is the number of all final points in the environment
goal=target(goalind,1:2);

end
