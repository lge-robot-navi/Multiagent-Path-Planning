function [coordix, coordiy, coorditheta, robotPoses] = Func_fovPathPlanning(img_name, coordix, coordiy)
%     img_name = 'map_outline.jpg';
    show_flag = 0; 
%     load('kumohResults.mat');
    %[coordix, coordiy, coorditheta, robotPoses] = Func_fovPathPlanning(img_name, coordix, coordiy, show_flag);
    %icoordix = coordix;
    %icoordiy = coordiy;
    global coveragedMap binaryMap sensorParam label figure;

     
    %% 데이터 전처리
    [coordix, coordiy] = KWdataPreProcessing(img_name, coordix, coordiy);
    
    %% 센서 파라미터 로드
    KWloadSensorParam();
    %% 커버리지 계산
    KWcalcMapCoverage(coordix, coordiy, 0);    % coverager Map이 binary Map의 3DoF으로 초기화 됨
%     kw_imshow(coordix, coordiy);
%     kw_dispCoveragePercent();
    %% KWfindUncoveragedArea()

    % [uncoveragedCluster, mode] = KWfindUncoveragedArea();
    % Robotnum = KWFindRobotnum(coordix, coordiy,uncoveragedCluster);   %Robotnum은 Cluster이랑 가장 가까운 Robot 번호

    %% KWcoverUncoveragedArea()
    % prevNearCell = [0, 0, 0, 0]; gain = 1;
    % Cells = KWUncoveredImg(coordix, coordiy, 0, 0); % empty img
    % while 1
    %     [nearCell, robot, node, point, phase] = KWfindNearCell(Cells, coordix, coordiy, 0);
    %     
    %     if isempty(nearCell)
    %         break;
    %     end
    %     if ~any(prevNearCell ~= nearCell)
    %         point = point/2 + (nearCell(1:2)' + nearCell(3:4)')/4;
    %         phase = phase*gain;
    %         gain = gain + 0.1;
    %     end
    %     prevNearCell = nearCell;
    %     
    %     path = kw_greedyCoverage(point, phase, 5, 2, 0);
    % 
    %     coordix{robot} = [coordix{robot}(1:node), path(1,:), coordix{robot}(node+1:end)];
    %     coordiy{robot} = [coordiy{robot}(1:node), path(2,:), coordiy{robot}(node+1:end)];
    %     
    %     Cells = KWUncoveredImg(coordix, coordiy, 0, 1);
    %     
    % end
    % 

    % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
    [cluster, cell_index, num] = kw_cellDecomposition(coordix, coordiy, show_flag);
    prev_cell = [0, 0, 0, 0, 0, 0];

    while ~isempty(cell_index)
        [nearClusterNum, robot, node, point, phase] = KWcontactCell(cluster, cell_index, num, coordix, coordiy, show_flag);
        if nearClusterNum == 0
            break;
        end
        if ~any([nearClusterNum, robot, node, point, phase] ~= prev_cell)
            point = point + 30*5*[cos(phase), sin(phase)];
        end
        prev_cell = [nearClusterNum, robot, node, point, phase];
        path = kw_CBGA(point, phase, cell_index{nearClusterNum}, 5, 2, show_flag);

        coordix{robot} = [coordix{robot}(1:node-1), path(1,:), coordix{robot}(node+1:end)];
        coordiy{robot} = [coordiy{robot}(1:node-1), path(2,:), coordiy{robot}(node+1:end)];

        [cluster, cell_index, num] = kw_cellDecomposition(coordix, coordiy, show_flag);
    end

%%  new coveraged calculate
     for robot = 1:6
         if isnan(coordix{robot})
             continue;
         end
        fx = coordix{robot};
        fy = coordiy{robot};
%         ix = icoordix{robot};
%         iy = icoordiy{robot};
        coordix{robot} = [fx(:,1:end), fliplr(fx(:,1:end-1))];
        coordiy{robot} = [fy(:,1:end), fliplr(fy(:,1:end-1))];
     end
     
    KWcalcMapCoverage(coordix, coordiy, show_flag);    % coverager Map이 binary Map의 3DoF으로 초기화 됨
    
    disp('알고리즘이 종료 되었습니다.');

    [robotPoses, coorditheta] = kw_imshow(coordix, coordiy);
    kw_dispCoveragePercent();
    
    [coordix, coordiy, coorditheta, robotPoses] = KWdataPreProcessing2(coordix, coordiy, coorditheta, robotPoses);
    % save('PrevkwResults.mat', 'coordix', 'coordiy', 'coorditheta', 'robotPoses');

end



