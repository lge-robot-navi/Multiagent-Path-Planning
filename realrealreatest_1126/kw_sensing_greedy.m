function [phase, flag] = kw_sensing_greedy(position, phase)
% 도르마무 위치 수정 중
    global coveragedMap binaryMap sensorParam;
    
%     if( frame(position(1), position(2))==0 )
%         phase = -phase_;
%         return;
%     end

    sensorParam.scolor = sensorParam.scolor+sensorParam.dc;
    if(sensorParam.scolor > 150)
        sensorParam.dc = -1;
    elseif(sensorParam.scolor < 70)
        sensorParam.dc = 1;
    end
    
    H = sensorParam.H;
    num =0; sum_y = 0; sum_x = 0; flag = 0;
    
    for theta = phase-sensorParam.fov/2 : sensorParam.dFOV : phase+sensorParam.fov/2
        x = position(1) + sensorParam.radius*cos(theta);
        y = position(2) + sensorParam.radius*sin(theta);
        
        index = kw_point2ind(position, [x, y]);               % Linear indices
        
        if isempty(index)
            disp('error : length(index) is zero');
            flag = 0;
            return;
        end
        
        for s = index
            if binaryMap(s) < 200        % 장애물과 만남
                % coveragedMap(s) = coveragedMap(s)+5;
                % num = num + 5;
                % phase = phase - theta*5;
                break;
            else
                %flag = flag + (coveragedMap(s) > 230);
                if coveragedMap(s) > 230
                    flag = flag + 1;
                    sum_y = sum_y+floor(s/H)*10;
                    sum_x = sum_x+mod(s,H)*10+10;
                    num = num + 10;
                end
                
                coveragedMap(s) = sensorParam.scolor;  % Set the line points to white
                sum_y = sum_y+floor(s/H);
                sum_x = sum_x+mod(s,H)+1;
                num = num + 1;
            end
        end
    end
    
    phase = atan2(sum_y/num-position(2), sum_x/num-position(1))-phase;
    if isnan(phase)
        %disp('error : phase is Nan, Dormammu!!');
        flag = 0;
        % phase = phase_;
    end
    
    flag = flag/200 > 0;
    % [phase sum_x/num-position(1), sum_y/num-position(2)]
end