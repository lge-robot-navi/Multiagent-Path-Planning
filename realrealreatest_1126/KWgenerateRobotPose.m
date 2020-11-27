function robotPose = KWgenerateRobotPose(coordix,coordiy)

global binaryMap; 

robotPose=zeros(length(coordix),3);
robotPose(:,1)=round(coordix);
robotPose(:,2)=round(coordiy);
for i=1:size(robotPose,1)-1
    robotPose(i,3)=atan2(robotPose(i+1,2)-robotPose(i,2),robotPose(i+1,1)-robotPose(i,1));
end
robotPose(end,3)=atan2(robotPose(1,2)-robotPose(end,2),robotPose(1,1)-robotPose(end,1));

% delete invalid robotPos
invalid_idx=[];
for i=1:size(robotPose,1)
    if binaryMap(robotPose(i,1),robotPose(i,2))==0
        invalid_idx=[invalid_idx;i];
        disp('Delete invalid robotPos');
    end
end
robotPose(invalid_idx,:)=[];

end

