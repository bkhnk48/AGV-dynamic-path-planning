function  simenvironment()
%Simulation environment
global storagerock; 
global target;
global trajp;
     axis([-1 35 -1 25]);
%      grid on
            
            %Draw the storage rock?
            rectangle('Position',[2.9,2.9,1.2,0.2]);hold on;rectangle('Position',[2.9,8.9,1.2,0.2]);hold on;
            rectangle('Position',[2.9,4.9,1.2,0.2]);hold on;rectangle('Position',[2.9,10.9,1.2,0.2]);hold on;
            rectangle('Position',[2.9,6.9,1.2,0.2]);hold on;rectangle('Position',[2.9,12.9,1.2,0.2]);hold on;
            rectangle('Position',[5.9,4.9,1.2,0.2]);hold on;rectangle('Position',[2.9,14.9,1.2,0.2]);hold on;
            rectangle('Position',[5.9,2.9,1.2,0.2]);hold on;rectangle('Position',[2.9,16.9,1.2,0.2]);hold on;
            rectangle('Position',[5.9,6.9,1.2,0.2]);hold on;rectangle('Position',[2.9,18.9,1.2,0.2]);hold on;
            rectangle('Position',[8.9,6.9,1.2,0.2]);hold on;rectangle('Position',[2.9,20.9,1.2,0.2]);hold on;
            rectangle('Position',[8.9,4.9,1.2,0.2]);hold on;rectangle('Position',[5.9,8.9,1.2,0.2]);hold on;
            rectangle('Position',[8.9,2.9,1.2,0.2]);hold on;rectangle('Position',[5.9,10.9,1.2,0.2]);hold on;
            rectangle('Position',[11.9,6.9,1.2,0.2]);hold on;rectangle('Position',[5.9,12.9,1.2,0.2]);hold on;
            rectangle('Position',[11.9,4.9,1.2,0.2]);hold on;rectangle('Position',[5.9,14.9,1.2,0.2]);hold on;
            rectangle('Position',[11.9,2.9,1.2,0.2]);hold on;rectangle('Position',[5.9,16.9,1.2,0.2]);hold on;
            rectangle('Position',[5.9,18.9,1.2,0.2]);hold on;rectangle('Position',[14.9,2.9,1.2,0.2]);hold on;
            rectangle('Position',[5.9,20.9,1.2,0.2]);hold on;rectangle('Position',[14.9,4.9,1.2,0.2]);hold on;
            rectangle('Position',[8.9,8.9,1.2,0.2]);hold on;rectangle('Position',[14.9,6.9,1.2,0.2]);hold on;
            rectangle('Position',[8.9,10.9,1.2,0.2]);hold on;rectangle('Position',[14.9,8.9,1.2,0.2]);hold on;
            rectangle('Position',[8.9,12.9,1.2,0.2]);hold on;rectangle('Position',[14.9,10.9,1.2,0.2]);hold on;
            rectangle('Position',[8.9,14.9,1.2,0.2]);hold on;rectangle('Position',[14.9,12.9,1.2,0.2]);hold on;
            rectangle('Position',[8.9,16.9,1.2,0.2]);hold on;rectangle('Position',[14.9,14.9,1.2,0.2]);hold on;
            rectangle('Position',[8.9,18.9,1.2,0.2]);hold on;rectangle('Position',[14.9,16.9,1.2,0.2]);hold on;
            rectangle('Position',[8.9,20.9,1.2,0.2]);hold on;rectangle('Position',[14.9,18.9,1.2,0.2]);hold on;
            rectangle('Position',[11.9,8.9,1.2,0.2]);hold on;rectangle('Position',[14.9,20.9,1.2,0.2]);hold on;
            rectangle('Position',[11.9,10.9,1.2,0.2]);hold on;
            rectangle('Position',[11.9,12.9,1.2,0.2]);hold on;
            rectangle('Position',[11.9,14.9,1.2,0.2]);hold on;
            rectangle('Position',[11.9,16.9,1.2,0.2]);hold on;
            rectangle('Position',[11.9,18.9,1.2,0.2]);hold on;
            rectangle('Position',[11.9,20.9,1.2,0.2]);hold on;
            rectangle('Position',[17.9,2.9,1.2,0.2]);hold on;
            rectangle('Position',[17.9,4.9,1.2,0.2]);hold on;
            rectangle('Position',[17.9,6.9,1.2,0.2]);hold on;
            rectangle('Position',[17.9,8.9,1.2,0.2]);hold on;
            rectangle('Position',[17.9,10.9,1.2,0.2]);hold on;
            rectangle('Position',[17.9,12.9,1.2,0.2]);hold on;
            rectangle('Position',[17.9,14.9,1.2,0.2]);hold on;
            rectangle('Position',[17.9,16.9,1.2,0.2]);hold on;
            rectangle('Position',[17.9,18.9,1.2,0.2]);hold on;
            rectangle('Position',[17.9,20.9,1.2,0.2]);hold on;
                        rectangle('Position',[20.9,2.9,1.2,0.2]);hold on;
            rectangle('Position',[20.9,4.9,1.2,0.2]);hold on;
            rectangle('Position',[20.9,6.9,1.2,0.2]);hold on;
            rectangle('Position',[20.9,8.9,1.2,0.2]);hold on;
            rectangle('Position',[20.9,10.9,1.2,0.2]);hold on;
            rectangle('Position',[20.9,12.9,1.2,0.2]);hold on;
            rectangle('Position',[20.9,14.9,1.2,0.2]);hold on;
            rectangle('Position',[20.9,16.9,1.2,0.2]);hold on;
            rectangle('Position',[20.9,18.9,1.2,0.2]);hold on;
            rectangle('Position',[20.9,20.9,1.2,0.2]);hold on;
                        rectangle('Position',[23.9,2.9,1.2,0.2]);hold on;
            rectangle('Position',[23.9,4.9,1.2,0.2]);hold on;
            rectangle('Position',[23.9,6.9,1.2,0.2]);hold on;
            rectangle('Position',[23.9,8.9,1.2,0.2]);hold on;
            rectangle('Position',[23.9,10.9,1.2,0.2]);hold on;
            rectangle('Position',[23.9,12.9,1.2,0.2]);hold on;
            rectangle('Position',[23.9,14.9,1.2,0.2]);hold on;
            rectangle('Position',[23.9,16.9,1.2,0.2]);hold on;
            rectangle('Position',[23.9,18.9,1.2,0.2]);hold on;
            rectangle('Position',[23.9,20.9,1.2,0.2]);hold on;
                        rectangle('Position',[26.9,2.9,1.2,0.2]);hold on;
            rectangle('Position',[26.9,4.9,1.2,0.2]);hold on;
            rectangle('Position',[26.9,6.9,1.2,0.2]);hold on;
            rectangle('Position',[26.9,8.9,1.2,0.2]);hold on;
            rectangle('Position',[26.9,10.9,1.2,0.2]);hold on;
            rectangle('Position',[26.9,12.9,1.2,0.2]);hold on;
            rectangle('Position',[26.9,14.9,1.2,0.2]);hold on;
            rectangle('Position',[26.9,16.9,1.2,0.2]);hold on;
            rectangle('Position',[26.9,18.9,1.2,0.2]);hold on;
            rectangle('Position',[26.9,20.9,1.2,0.2]);hold on;
            rectangle('Position',[29.9,2.9,1.2,0.2]);hold on;
            rectangle('Position',[29.9,4.9,1.2,0.2]);hold on;
            rectangle('Position',[29.9,6.9,1.2,0.2]);hold on;
            rectangle('Position',[29.9,8.9,1.2,0.2]);hold on;
            rectangle('Position',[29.9,10.9,1.2,0.2]);hold on;
            rectangle('Position',[29.9,12.9,1.2,0.2]);hold on;
            rectangle('Position',[29.9,14.9,1.2,0.2]);hold on;
            rectangle('Position',[29.9,16.9,1.2,0.2]);hold on;
            rectangle('Position',[29.9,18.9,1.2,0.2]);hold on;
            rectangle('Position',[29.9,20.9,1.2,0.2]);hold on;
            
            %Stopping points of robots
             %rectangle('Position',[4.3,21.8,1.4,1.4]);hold on;rectangle('Position',[7.3,21.8,1.4,1.4]);hold on;
             %rectangle('Position',[10.3,21.8,1.4,1.4]);hold on;rectangle('Position',[13.3,21.8,1.4,1.4]);hold on;
             rectangle('Position',[16.3,21.8,1.4,1.4]);hold on;
             %rectangle('Position',[19.3,21.8,1.4,1.4]);hold on;
             %rectangle('Position',[22.3,21.8,1.4,1.4]);hold on;
             %rectangle('Position',[25.3,21.8,1.4,1.4]);hold on;rectangle('Position',[28.3,21.8,1.4,1.4]);hold on;
             
             %rectangle('Position',[0.3,19.3,1.4,1.4]);hold on;rectangle('Position',[0.3,17.3,1.4,1.4]);hold on;rectangle('Position',[0.3,15.3,1.4,1.4]);hold on;
             %rectangle('Position',[0.3,13.3,1.4,1.4]);hold on;
             rectangle('Position',[0.3,11.3,1.4,1.4]);hold on;
             %rectangle('Position',[0.3,9.3,1.4,1.4]);hold on;
             %rectangle('Position',[0.3,7.3,1.4,1.4]);hold on;rectangle('Position',[0.3,5.3,1.4,1.4]);hold on;rectangle('Position',[0.3,3.3,1.4,1.4]);hold on;
             
             %rectangle('Position',[32.3,19.3,1.4,1.4]);hold on;rectangle('Position',[32.3,17.3,1.4,1.4]);hold on;rectangle('Position',[32.3,15.3,1.4,1.4]);hold on;
             %rectangle('Position',[32.3,13.3,1.4,1.4]);hold on;
             rectangle('Position',[32.3,11.3,1.4,1.4]);hold on;
             %rectangle('Position',[32.3,9.3,1.4,1.4]);hold on;
             %rectangle('Position',[32.3,7.3,1.4,1.4]);hold on;rectangle('Position',[32.3,5.3,1.4,1.4]);hold on;rectangle('Position',[32.3,3.3,1.4,1.4]);hold on;
             
             %rectangle('Position',[4.3,0.8,1.4,1.4]);hold on;rectangle('Position',[7.3,0.8,1.4,1.4]);hold on;rectangle('Position',[10.3,0.8,1.4,1.4]);hold on;
             %rectangle('Position',[13.3,0.8,1.4,1.4]);hold on;
             rectangle('Position',[16.3,0.8,1.4,1.4]);hold on;
             %rectangle('Position',[19.3,0.8,1.4,1.4]);hold on;
             %rectangle('Position',[22.3,0.8,1.4,1.4]);hold on;rectangle('Position',[25.3,0.8,1.4,1.4]);hold on;rectangle('Position',[28.3,0.8,1.4,1.4]);hold on;
             
             rectangle('Position',[0,0,34,24]);hold on;
            plot(storagerock(:,1),storagerock(:,2),'sk');hold on;
            plot(target(:,1),target(:,2),'xr');hold on;

            plot([2.5 31.5],[6 6],'--k');hold on;plot([2.5 31.5],[12 12],'--k');hold on;plot([2.5 31.5],[18 18],'--k');hold on;
            plot([2.5 31.5],[4 4],'--k');hold on;plot([2.5 31.5],[14 14],'--k');hold on;plot([2.5 31.5],[20 20],'--k');hold on;
            plot([2.5 31.5],[8 8],'--k');hold on;plot([2.5 31.5],[16 16],'--k');hold on;
            plot([2.5 31.5],[10 10],'--k');hold on;
            plot([5 5],[2.5 21.5],'--k');hold on;plot([14 14],[2.5 21.5],'--k');hold on;plot([20 20],[2.5 21.5],'--k');hold on;
            plot([8 8],[2.5 21.5],'--k');hold on;plot([17 17],[2.5 21.5],'--k');hold on;plot([23 23],[2.5 21.5],'--k');hold on;
            plot([11 11],[2.5 21.5],'--k');hold on;plot([26 26],[2.5 21.5],'--k');hold on;plot([29 29],[2.5 21.5],'--k');hold on;

end
