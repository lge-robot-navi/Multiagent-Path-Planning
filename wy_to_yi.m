function [y_dash] = wy_to_yi(y)

y_dash= ((3968-2100)*0.05-y)/0.05;

%     for i=1:size(coordix,2)
%           for j = 1:size(coordix{i},2)
%              posx{i}(1,j) = 0.05*(  coordiy{i}(1,j)-2180);
%              posy{i}(1,j) = 0.05*( 3968-2100 - coordix{i}(1,j));
%              if(j ==size(coordix{i},2) )
%                  posTheta{i}(1,j) = atan2(posy{i}(1,1)-posy{i}(1,j),posx{i}(1,1)-posx{i}(1,j));
%              else
%                  posTheta{i}(1,j) = atan2(0.05*(3968-2100 - coordix{i}(1,j+1))-posy{i}(1,j),0.05*(coordiy{i}(1,j+1)-2180)-posx{i}(1,j));
%              end
%           end
%     end