function KWcalcMapCoverage(coordix, coordiy, show_flag)

    global sensorParam coveragedMap binaryMap;
    
    color=[1,0,0;0,1,0;0,0,1;1,1,0;0,1,1];
    dvec = sensorParam.dvec;    % 한번 이동하는 벡터의 길이(^2)
    coveragedMap = binaryMap;
    % imshow(coveragedMap);
%     coveragedMap = cat(3, binaryMap, binaryMap, binaryMap);
                        
    for robot = 1:5
        disp([num2str(robot), ' Robot']);
        ix = coordix{robot};
        iy = coordiy{robot};
        
        if isnan(ix)
            continue;
        elseif numel(ix) < 1
            continue;
        end
        position = [ix(2), iy(2)];
        phase = 0;

        for cont = 1:length(ix)
            % disp(['Node', num2str(cont)]);
            distans = [ix(cont), iy(cont)] - position;     
            delta = sqrt(sum(distans.^2))/dvec;
            
            if delta == 0
                 % disp([' ** KWcalcMapCoverage.m error : 중복노드 발생!! robot', int2str(robot)]);
                 position = [ix(cont), iy(cont)];
                 continue;
            end
            
            % 회전 해야할 각도
            cphase = atan2(distans(2),distans(1));   % 목표 각도
            theta = rem(cphase - phase, pi*2);
            if abs(theta) > pi
                theta = (theta-2*pi*sign(theta)); 
            end
            
            signFoV = sign(theta)*sensorParam.fov;
            for th = phase+signFoV : signFoV/(1+show_flag) : phase+theta
                KWMapCoverage(position, th);
            end
                
            phase = cphase;
            distans = distans/delta;
            for d = 1:delta
            	position = position+distans;
                %phase = phase+theta;
                flag = KWMapCoverage(position, phase);
                
%                 if flag % 로봇이 이동 중 벽에 닿으면 다음 위치로 순간이동 시킴
%                     position = [ix(cont), iy(cont)];
%                     break;
%                 end
                % show = kw_imshow(coveragedMap, coordix, coordiy, [200, 200, 200]);
                % title("The Robot number is " + robot,'FontSize', 22);
                if show_flag
                     x1= position(1) +30*cos(phase);
                     y1= position(2) +30*sin(phase);
                     x2= position(1) +30*cos(phase+3*pi/4);
                     y2= position(2) +30*sin(phase+3*pi/4);
                     x3= position(1) +30*cos(phase-3*pi/4);
                     y3= position(2) +30*sin(phase-3*pi/4);

                    % imshow(coveragedMap);
                    image(coveragedMap);
                    patch([y1,y2,y3,y1],[x1,x2,x3,x1],color(robot,:));
                    pause(0.0001);
                end
            end
        end
    end
    
    se = strel('disk',10);
    coveragedMap = imclose(coveragedMap,se);
    coveragedMap = imopen(coveragedMap,se);
    coveragedMap(coveragedMap<230 & coveragedMap>0)=50;
end

function wwww(coordix, coordiy, show_flag)

    global sensorParam coveragedMap binaryMap;
    
    color=[1,0,0;0,1,0;0,0,1;1,1,0;0,1,1];
    dvec = sensorParam.dvec;    % 한번 이동하는 벡터의 길이(^2)
    coveragedMap = binaryMap;
    imshow(coveragedMap);
%     coveragedMap = cat(3, binaryMap, binaryMap, binaryMap);
    
    max = 0;
    for robot = 1:5
        ix = coordix{robot};
        if numel(ix) > max
            max = numel(ix);
        end
    end
    %%
    zeros(1, 10)
    %%
    for robot = 1:5
        arrx = zeros(1, max);
        arry = zeros(1, max);
        ix = coordix{robot};
        iy = coordiy{robot};
        for k = 1 : numel(ix)
            arrx(k) = ix(k);
            arry(k) = iy(k);
        end
    end

    for robot = 1:5
        ix = coordix{robot};
        iy = coordiy{robot};

        position = [ix(1), iy(1)];
        phase = 0;

        for cont = 2:length(ix)
            % disp(['Node', num2str(cont)]);
            distans = [ix(cont), iy(cont)] - position;     
            delta = sqrt(sum(distans.^2))/dvec;
            
            if delta == 0
                 disp([' ** KWcalcMapCoverage.m error : 중복노드 발생!! robot', int2str(robot)]);
                 position = [ix(cont), iy(cont)];
                 continue;
            end
            
            % 회전 해야할 각도
            cphase = atan2(distans(2),distans(1));   % 목표 각도
            theta = rem(cphase - phase, pi*2);
            if abs(theta) > pi
                theta = (theta-2*pi*sign(theta)); 
            end
            
            signFoV = sign(theta)*sensorParam.fov;
            for th = phase+signFoV : signFoV/(1+show_flag) : phase+theta
                KWMapCoverage(position, th);
            end
                
            phase = cphase;
            distans = distans/delta;
            for d = 1:delta
            	position = position+distans;
                %phase = phase+theta;
                flag = KWMapCoverage(position, phase);
                
%                 if flag % 로봇이 이동 중 벽에 닿으면 다음 위치로 순간이동 시킴
%                     position = [ix(cont), iy(cont)];
%                     break;
%                 end
                % show = kw_imshow(coveragedMap, coordix, coordiy, [200, 200, 200]);
                % title("The Robot number is " + robot,'FontSize', 22);
                if show_flag
                     x1= position(1) +30*cos(phase);
                     y1= position(2) +30*sin(phase);
                     x2= position(1) +30*cos(phase+3*pi/4);
                     y2= position(2) +30*sin(phase+3*pi/4);
                     x3= position(1) +30*cos(phase-3*pi/4);
                     y3= position(2) +30*sin(phase-3*pi/4);

                    % imshow(coveragedMap);
                    image(coveragedMap);
                    patch([y1,y2,y3,y1],[x1,x2,x3,x1],color(robot,:));
                    pause(0.0001);
                end
            end
        end
    end
    
    se = strel('disk',10);
    coveragedMap = imclose(coveragedMap,se);
    coveragedMap = imopen(coveragedMap,se);
    coveragedMap(coveragedMap<230 & coveragedMap>0)=50;
end


