function Mat = kw_BWfilter(Mat)   
    Mat(Mat<230)=0;
    Mat(Mat>=230)=255;

    se = strel('disk',10);
    Mat = imclose(Mat,se);
    Mat = imopen(Mat,se);
end