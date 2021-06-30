function [cells, cell_index, numCluster] = kw_cellDecomposition(coordix, coordiy, showing_flag)
    global sensorParam coveragedMap;
    % imresize(imresize(kw_BWfilter(coveragedMap),0.02,'nearest'),50,'nearest')
    cs = sensorParam.cs;
    cs_h = cs/2-.5;
    label = imresize(kw_BWfilter(coveragedMap),1/cs,'nearest');
    label=bwlabel(label); 
    [H, W] = size(label);
    cells = [];
    cell_index = [];
    %imresize(label,0.02,'nearest');
    for numCluster = 1:max(label,[],'all')  % 클러스터의 갯수
        cells{numCluster} = [];
        cell_index{numCluster} = [];
        for s = find(label==numCluster)'
            x = mod(s,H)*cs-cs_h;
            y = floor(s/H)*cs+cs_h;
            cells{numCluster} = [cells{numCluster}, x, y];
            cell_index{numCluster} = [cell_index{numCluster}, sub2ind(sensorParam.size, x, y)];
        end
    end
    %%
    if showing_flag && ~isempty(showing_flag) %% 출력 구분 %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %%
        %Mat = cat(3, coveragedMap, coveragedMap, coveragedMap);
        
        R = coveragedMap;
        G = coveragedMap;
        B = coveragedMap;

        for k = 1:numCluster
            cell = cells{k};
            for s = 1:2:numel(cell)
                x = cell(s)-cs_h:cell(s)+cs_h;
                y = cell(s+1)-cs_h:cell(s+1)+cs_h;
                
                R(x,y) =  R(x,y) - 100;
                G(x,y) =  G(x,y) - 100;
                B(x,y) =  B(x,y) + 100;
            end
%             index = cell_index{k};
%             for s = index
%                 R(s) =  R(s) + 200;
%                 G(s) =  G(s) - 100;
%                 B(s) =  B(s) - 100;
% %                 y = floor(s/sensorParam.H);
% %                 x = mod(s,sensorParam.H)+1;
% %                 x = x-cs_h:x+cs_h;
% %                 y = y-cs_h:y+cs_h;
%             end
        end
        
        Mat = cat(3, R, G, B);
        imshow(Mat);
        color=[1,0,0;0,1,0;0,0,1;1,1,0;0,1,1];

        for i=1:5
            if isnan(coordix{i});
                continue;
            end
            robotPoses=KWgenerateRobotPose(coordix{i},coordiy{i});
            KWvisualizeRobot(robotPoses, color(i,:));
        end
    end

    
    
end