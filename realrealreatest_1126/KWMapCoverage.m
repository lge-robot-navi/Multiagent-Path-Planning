function flag = KWMapCoverage(position, phase)
global sensorParam coveragedMap binaryMap;
    position = round(position);
    radius = sensorParam.radius;
    sensorParam.scolor = sensorParam.scolor + sensorParam.dc;
    sensorParam.dc = (sensorParam.scolor<51)+(sensorParam.scolor<51) - (sensorParam.scolor>169)-(sensorParam.scolor>169) + sensorParam.dc;

    if binaryMap( position(1), position(2) )==0
        % disp('error : 벽을 뚫는 경로가 있습니다! KW_MapCoverage.m,');
        flag = 1;
        return;
    end
    
    for theta = phase-sensorParam.fov/2 : sensorParam.dFOV : phase+sensorParam.fov/2
        x = position(1) + radius*cos(theta);
        y = position(2) + radius*sin(theta);
        
        index = kw_point2ind(position, [x, y]);               % Linear indices
        
        for s = index
            if (binaryMap(s) < 100)        % 장애물과 만남
                break;
            else
                coveragedMap(s) =  sensorParam.scolor;
            end
        end
    end
    flag = 0;
end