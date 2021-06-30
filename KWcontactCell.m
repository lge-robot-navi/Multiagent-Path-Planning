function [cll, robot, node, point, phase] = KWcontactCell(cluster, cell_index, num, coordix, coordiy, show_flag)
% cll은 cluster 번호    
global sensorParam binaryMap coveragedMap;
    
    cll = 0; robot=0; node=0; point=0; phase=0; celln = 0;
    min = 5000;
    
    for robotn = 1:6
        ix = coordix{robotn};
        iy = coordiy{robotn};
        len = length(ix);          
        for cont = 2:len
            
            if cont ~= len
                position_inode = [ix(cont-1), iy(cont-1)];
                position_fnode = [ix(cont), iy(cont)];
            
            for k = 1:num
                cells = cluster{k};
                for cn = 1:2:numel(cells)
                    position_cell = [cells(cn), cells(cn+1)];
                    
                    fphase = atan2(position_fnode(2),position_fnode(1));   % 목표 각도
                    iphase = atan2(position_inode(2),position_inode(1));   % 목표 각도
                    
                    theta = rem(fphase - iphase, pi*2);
                    if abs(theta) > pi
                        theta = (theta-2*pi*sign(theta)); 
                    end
                    
                    distans = sqrt(sum((position_fnode - position_cell).^2)) + abs(theta-30)*5;

                    if (min > distans) && (distans > sensorParam.mindis)
                        index = kw_point2ind(position_fnode, position_cell);

                        flag = 1;
                        for s = index
                            if (binaryMap(s) < 100)        % 장애물과 만남
                                flag = 0;
                                break;
                            end
                        end

                        if flag
                            min = distans;
                            cll = k;        % cluster 번호
                            celln = cn;     % cell 번호
                            robot = robotn;
                            node = cont; 
                            point = position_fnode;
                            % 중심을 바라보는 방향
                            %delta = [cell(1) + cell(3), cell(2) + cell(4)]/2-position_node;
                            % 좌상단, 우하단
                        end
                    end
                end
            end
        end
    end
    if cll ~= 0
        delta = [cluster{cll}(celln), cluster{cll}(celln+1)] - point;
        phase =  atan2(delta(2), delta(1));
%     Cells(Cells==nearCell(1),:) = [];

        % 가장 가까운 cell과 node 쌍을 출력
        if show_flag %% 출력 구분 %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %%
            Mat = cat(3, coveragedMap, coveragedMap, coveragedMap);
            
            cs_h = sensorParam.cs/2-.5;
            
            x = cluster{cll}(celln)  -cs_h : cluster{cll}(celln)  +cs_h;
            y = cluster{cll}(celln+1)-cs_h : cluster{cll}(celln+1)+cs_h;

            Mat( x, y, 1) =  Mat( x,  y, 1) + 100;
            Mat( x, y, 2) =  Mat( x,  y, 2) - 100;
            Mat( x, y, 3) =  Mat( x,  y, 3) - 100;

            i = -15:15;
            Mat( coordix{robot}(node)+i,coordiy{robot}(node)+i, 1) = 255;
            Mat( coordix{robot}(node)+i,coordiy{robot}(node)+i, 2) = 0;
            Mat( coordix{robot}(node)+i,coordiy{robot}(node)+i, 3) = 0;

            imshow(Mat);
        end
    end
end