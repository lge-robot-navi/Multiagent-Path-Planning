function kw_dispCoveragePercent()
global binaryMap coveragedMap;
    se = strel('disk',10);
    Mat = imclose(coveragedMap,se);
    Mat = imopen(Mat,se);
    cover_percent = (1 - numel(find(Mat==255))/numel(find(binaryMap>230)))*100;
    disp([' coveraged parcent : ', num2str(cover_percent)]);
end
