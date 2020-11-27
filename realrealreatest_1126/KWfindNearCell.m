function [nearCell, robot, node, point, phase] = KWfindNearCell(Cells, coordix, coordiy, show_flag)
global sensorParam coveragedMap binaryMap;
    nearCell = []; robot=0; node=0; point=0; phase=0;
    min = 5000^2;    
    
    for robotn = 1:5
        ix = coordix{robotn};
        iy = coordiy{robotn};
                    
        for cont = 1:length(ix)
            position_node = [ix(cont), iy(cont)];
            for cell = Cells'
                 %position_cell = [cell(2) + cell(4), cell(1) + cell(5)]/2;
                 position_cell = [cell(1) + cell(3), cell(2) + cell(4)]/2;
                 distans = sum((position_node - position_cell).^2);

                 % 중심에 벽이있는지 검사
                 % (애커만) 이동가능한지 판단하는 함수 추가하기
                 if (min > distans) && (distans > 100^2)
                    index = kw_point2ind(position_node, position_cell);          

                    if isempty(index)
                        break;
                    end

                    flag = 1;
                    for s = index
                        if (binaryMap(s) < 100)        % 장애물과 만남
                            flag = 0;
                            break;
                        end
                    end

                    if flag
                        min = distans;
                        nearCell = cell;
                        robot = robotn;
                        node = cont; 
                        point = position_node;
                        % 중심을 바라보는 방향
                        %delta = [cell(1) + cell(3), cell(2) + cell(4)]/2-position_node;
                        % 좌상단, 우하단
                        distans = sum((position_node - [cell(1), cell(2)]).^2);
                        if distans > 200^2
                            delta = [cell(1), cell(2)]-position_node;
                        else
                            delta = [cell(3), cell(4)]-position_node;
                        end
                        phase =  atan2(delta(2), delta(1));
                    end
                end
            end
        end
    end
    
    % 가장 가까운 cell과 node 쌍을 출력
    if show_flag && ~isempty(nearCell)%% 출력 구분 %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %%
        Mat = cat(3, coveragedMap, coveragedMap, coveragedMap);
        
        x = nearCell(1):nearCell(3);
        y = nearCell(2):nearCell(4);

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