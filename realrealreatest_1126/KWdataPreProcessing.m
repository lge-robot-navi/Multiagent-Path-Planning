function [coordix, coordiy] = KWdataPreProcessing(file_name, coordix, coordiy)
    global binaryMap; % coveragedMap 
    size_img = size(imread(file_name));
    
     
    if(size(size_img,2) == 2)
        binaryMap = (imread(file_name));
    elseif(size(size_img,2) == 3)
        binaryMap = rgb2gray(imread(file_name));
    end
    
    binaryMap(binaryMap<230)=0;
    binaryMap(binaryMap>=230)=255;

%     se = offsetstrel('ball',5,3);
    se = strel('disk',10);
    binaryMap = imclose(binaryMap,se);
    binaryMap = imopen(binaryMap,se);
    
%     coveragedMap = cat(3, binaryMap, binaryMap, binaryMap);
    
    for robot = 1:5
        if robot > numel(coordix)
            coordix{robot} = nan;
            coordiy{robot} = nan;
            continue;
        end
        if isempty(coordix{robot})
            coordix{robot} = nan;
            coordiy{robot} = nan;
            continue;
        end
        
        coordix{robot} = round(coordix{robot});
        coordiy{robot} = round(coordiy{robot});
    end
    %imshow(binaryMap);
end