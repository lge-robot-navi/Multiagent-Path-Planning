function [x_dash] = wx_to_xi_gj(x)
% offsetx = 3141 ;
offsetx = 3254;
x_dash = (x)/0.05+offsetx;



%y_dash= (y+175.349997)/0.05;
%-66.4, -82.775 좌표계 변환결과
%0.05
%x'=0.05*x-66.4, y'=0.05*y-82.775
%x = (x'+66.4)/0.05, y = (x'+82.775)/0.05