function Out_binaryimage = Image_load2(POI,count)


binaryImage0=imread('map_gw.bmp');%map_gj\kwangju2.bmp');

% binaryImage0=imread('map_gj\map2.pgm');

binaryImage2=imread('testmap1.jpg');
binaryImage1=imread('Obstacle_over50cm_probability.bmp');
% 
%  count = 0;
 str = sprintf('binaryImage%d',count);
Out_binaryimage = binaryImage0;
