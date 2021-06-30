function [y_dash] = wy_to_yi_gj(y)
% offsety = 3254;

offsety = 3141 ;
y_dash= (y)/0.05+offsety;



%-66.4, -82.775 좌표계 변환결과
%0.05
%x'=0.05*x-66.4, y'=0.05*y-82.775
%x = (x'+66.4)/0.05, y = (x'+82.775)/0.05