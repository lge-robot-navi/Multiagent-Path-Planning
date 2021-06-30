function [robotPoses, coorditheta] = kw_imshow(coordix, coordiy)
    global coveragedMap;
   
    showimg = cat(3, coveragedMap, coveragedMap, coveragedMap);
    imshow(showimg);
    color=[1,0,0;0,1,0;0,0,1;1,1,0;0,1,1; 1,0,1];
    % imshow(coveragedMap);
    for i=1:6
        if isnan(coordix{i})
            continue;
        end
        robotPoses{i} = KWgenerateRobotPose(coordix{i},coordiy{i});
        coorditheta{i} = robotPoses{i}(:,3)';
        KWvisualizeRobot(robotPoses{i}, color(i,:));
    end
    pause(0.0001);
%     i = -16:14;
%     j = -16:14;
%     
%     scolor = [color(1), 0, 0;
%         0, color(2), 0;
%         0, 0, color(3);
%         color(1), color(2), 0;
%         0, color(2), color(3);
%         ];
%     
%     for robot = 1:5
%         ix = coordix{robot};
%         iy = coordiy{robot};
% 
%         for cont = 1:length(ix) % 여러색은 킹쩔수 없음
%             showimg(uint16(ix(cont)+j)+1,uint16(iy(cont)+i)+1, 1) = scolor(robot, 1);
%             showimg(uint16(ix(cont)+j)+1,uint16(iy(cont)+i)+1, 2) = scolor(robot, 2);
%             showimg(uint16(ix(cont)+j)+1,uint16(iy(cont)+i)+1, 3) = scolor(robot, 3);
%         end
%     end
   
    % disp('showimg kw img');
end
