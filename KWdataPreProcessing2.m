function [coordix, coordiy, coorditheta, robotPoses] = KWdataPreProcessing2(coordix, coordiy, coorditheta, robotPoses)

    for robot = 1:6
        if isnan(coordix{robot})
            coordix{robot} = [];
            coordiy{robot} = [];
            coorditheta{robot} = [];
            robotPoses{robot} = [];
        end
    end
end