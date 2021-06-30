function SaveRobotPose(coordix,coordiy)
     
origin = [-66.4, -82.775, 0.0000] ;
rescale = 0.05;
hold on;
% °æ·Î »Ì±â
fileID = fopen('robot1node.txt','w');
pathlengt1 = 0;
pathlengt2 = 0;
pathlengt3 = 0;
for j=1:size(coordiy{1},2)               
%         fprintf(fileID,'{"x": %f, "y": %f, "angle": %f},',coordiy{1}(j),coordix{1}(j),0);
       fprintf(fileID,'{"x": %f, "y": %f},',origin(1)+coordix{1}(j)*rescale,origin(2)+coordiy{1}(j)*rescale);
        fprintf(fileID,'\n');    
        if(j>1)
            pathlengt1 = pathlengt1+sqrt((coordiy{1}(j)-coordiy{1}(j-1))^2+(coordix{1}(j)-coordix{1}(j-1))^2);
        end
end
fclose(fileID);
fileID = fopen('robot1node2.txt','w');

for j=1:size(coordiy{2},2)               
         fprintf(fileID,'{"x": %f, "y": %f},',origin(1)+coordix{2}(j)*rescale,origin(2)+coordiy{2}(j)*rescale);
        fprintf(fileID,'\n');  
        if(j>1)
            pathlengt2 = pathlengt2+sqrt((coordiy{2}(j)-coordiy{2}(j-1))^2+(coordix{2}(j)-coordix{2}(j-1))^2);
        end
end
fclose(fileID);
fileID = fopen('robot1node3.txt','w');

for j=1:size(coordiy{3},2)               
%         fprintf(fileID,'{"x": %f, "y": %f, "angle": %f},',coordiy{3}(j),coordix{3}(j),0);
         fprintf(fileID,'{"x": %f, "y": %f},',origin(1)+coordix{3}(j)*rescale,origin(2)+coordiy{3}(j)*rescale);
        fprintf(fileID,'\n'); 
         if(j>1)
            pathlengt3 = pathlengt3+sqrt((coordiy{3}(j)-coordiy{3}(j-1))^2+(coordix{3}(j)-coordix{3}(j-1))^2);
        end
end
fclose(fileID);
fileID = fopen('robot1node4.txt','w');

for j=1:size(coordiy{4},2)               
%         fprintf(fileID,'{"x": %f, "y": %f, "angle": %f},',coordiy{3}(j),coordix{3}(j),0);
         fprintf(fileID,'{"x": %f, "y": %f},',origin(1)+coordix{4}(j)*rescale,origin(2)+coordiy{4}(j)*rescale);
        fprintf(fileID,'\n'); 
end
fclose(fileID);
fileID = fopen('robot1node5.txt','w');

for j=1:size(coordiy{5},2)               
%         fprintf(fileID,'{"x": %f, "y": %f, "angle": %f},',coordiy{3}(j),coordix{3}(j),0);
         fprintf(fileID,'{"x": %f, "y": %f},',origin(1)+coordix{5}(j)*rescale,origin(2)+coordiy{5}(j)*rescale);
        fprintf(fileID,'\n'); 
end
fclose(fileID);         

  