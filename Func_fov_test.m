
%load('kumohResults240.mat');
coordix = {[500]};
coordiy = {[500]};
[coordix, coordiy, coorditheta, robotPoses] = Func_fovPathPlanning('map_outline.jpg', coordix, coordiy, 0); % 1:출력, 2:생략


%%
x = 1:5;
y = [x; x+10; x+20; x+30]'
y1 = y;
y2 = y+100;
y3 = y+200;

pose = {y1, y2, y3};
p = pose{1}(3,2)

