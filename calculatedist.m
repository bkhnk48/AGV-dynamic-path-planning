function [dist] = caculatedist(x,xpath,point,goal)
%Calculate the distance from the current vehicle's current position to the target point
dist = 0;
task=[xpath;point;goal]
for i=1:size(task,1)
    dist=dist+norm(x-task(i,:));
end
    
end
