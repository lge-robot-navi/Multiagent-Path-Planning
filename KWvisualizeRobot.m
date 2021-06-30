function KWvisualizeRobot(robotPos, color)
    h=30;
    for i=1:size(robotPos,1)
        x1=robotPos(i,1)+h*cos(robotPos(i,3));
        y1=robotPos(i,2)+h*sin(robotPos(i,3));
        x2=robotPos(i,1)+h*cos(robotPos(i,3)+3*pi/4);
        y2=robotPos(i,2)+h*sin(robotPos(i,3)+3*pi/4);
        x3=robotPos(i,1)+h*cos(robotPos(i,3)-3*pi/4);
        y3=robotPos(i,2)+h*sin(robotPos(i,3)-3*pi/4);

        patch([y1,y2,y3,y1],[x1,x2,x3,x1],color);
    %     pause(0.5);
    end

    for i=1:size(robotPos,1)-1
        line([robotPos(i,2),robotPos(i+1,2)],[robotPos(i,1),robotPos(i+1,1)],'color',color);
    end

end

