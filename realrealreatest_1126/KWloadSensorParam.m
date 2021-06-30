function KWloadSensorParam()
global sensorParam binaryMap;

    sensorParam.fov = pi*120/180;        % 30, 50, 70, 90, 120
    sensorParam.dFOV = pi/720;          
    sensorParam.radius = 1000;           % 180, 240, 300, 360, 420
    sensorParam.dvec = 15;
    sensorParam.scolor = 100;
    sensorParam.dc = 1;       % delta color
    sensorParam.Rmin = 300/4; % 3[m] / 4[cm/pixel] = 75
    
    sensorParam.size = size(binaryMap);
    [sensorParam.H, sensorParam.W] = size(binaryMap);
    
    
    sensorParam.cs = 49; % sell size, must be odd
    sensorParam.mindis = (sensorParam.radius/2);
end
