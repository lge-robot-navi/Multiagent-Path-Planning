function Mat = kw_coverFilter(Mat)
    se = strel('disk',10);
    Mat = imopen(Mat,se);
    
    Mat(Mat>230)=255;
    Mat(Mat>0)=30;
end