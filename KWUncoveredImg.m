function cell  = KWUncoveredImg(coordix, coordiy, img_flag, showing_flag)
    global coveragedMap label;
    
    se = strel('disk',10);
    Mat = imclose(coveragedMap,se);
    Mat = imopen(Mat,se);
    Mat(Mat<230)=0;
    
    label=bwlabel(Mat);
    numCluster = max(label,[],'all');   %클러스터의 갯수   
    
    for i = 1:numCluster
        [r, c] = find(label==i); % [x y];
        Uncovered(i,:) = round(sum([r c], 1)/length([r c]));    %중점
        clusterNum.coord(i,:)= {i, [r, c]};
        clusterNum.COM(i,:) = {i, Uncovered(i,:)};              %center of mass\
    end
    
    cell =[];
    for pole=1:size(clusterNum.coord,1)
        max_x = max(clusterNum.coord{pole,2}(:,1));
        min_x = min(clusterNum.coord{pole,2}(:,1));
        max_y = max(clusterNum.coord{pole,2}(:,2));
        min_y = min(clusterNum.coord{pole,2}(:,2));
        if (max_x-min_x)*(max_y-min_y) > 4000
            cell = [ cell ; min_x, min_y, max_x, max_y];
        end
        
    end
    
    if showing_flag %% 출력 구분 %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %%
        
        Mat = cat(3, coveragedMap, coveragedMap, coveragedMap);
        
        for cll = cell' 
            %cell'
            x = cll(1):cll(3);
            y = cll(2):cll(4);

            Mat( x,  y, 1) =  Mat( x,  y, 1) - 100;
            Mat( x,  y, 2) =  Mat( x,  y, 2) - 100;
            Mat( x,  y, 3) =  Mat( x,  y, 3) + 100;
        end
        imshow(Mat);
    end
    
end