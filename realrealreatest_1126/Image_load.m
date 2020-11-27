function Out_binaryimage = Image_load(POI,count)

binaryImage0=imread('map_gw.bmp');

binaryImage2=imread('testmap1.jpg');
binaryImage1=imread('Obstacle_over50cm_probability.bmp');
% 
%  count = 0;
 str = sprintf('binaryImage%d',count);
Out_binaryimage = binaryImage0;
if(count~=0)
    if(strcmp(str,'binaryImage1'))
       Out_binaryimage = binaryImage1;
    elseif(strcmp(str,'binaryImage2'))
       Out_binaryimage = binaryImage2;
    end
end
% for i=1:4   
%     
%          if(strcmp(str,'binaryImage0'))
%             J = ones(1500,2000)*255;
%             if( POI == 1)
%             %     J = zeros(1500,2000)*255;
%                 J(1080:1410,385:835) = floor(J(1080:1410,385:835)/2);
%                 binaryImage0 = binaryImage;
%                 binaryImage0(J > 200) = 0;
%                 binaryImage(J < 200) = 0;
%             end        
%         end
%     
% end
% hold on;
% 
% 
% 
% Out_binaryimage = str;


