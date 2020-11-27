function D2 = distfunc(ZI,ZJ)


% function D2 = distfun(ZI,ZJ)
% % calculation of distance
% ...
% where
% ZI is a 1-by-n vector containing a single observation.
% 
% ZJ is an m2-by-n matrix containing multiple observations. distfun must accept a matrix ZJ with an arbitrary number of observations.
% 
% D2 is an m2-by-1 vector of distances, and D2(k) is the distance between observations ZI and ZJ(k,:).

count = 0;
binaryImage0=imread('map_outline.jpg');

binaryImage2=imread('testmap1.jpg');
binaryImage1=imread('Obstacle_over50cm_probability.bmp');
% 
 count = 0;
 str = sprintf('binaryImage%d',count);
Out_binaryimage = binaryImage0;
if(count~=0)
    if(strcmp(str,'binaryImage1'))
       Out_binaryimage = binaryImage1;
    elseif(strcmp(str,'binaryImage2'))
       Out_binaryimage = binaryImage2;
    end
end


dmat = [];
 nPoints = size(ZJ,1);
D2 = ones(nPoints,1)*realmax;


  
   for i=1:nPoints
       distance1 = sqrt((ZI(1,:) - ZJ(i,:))*(ZI(1,:) - ZJ(i,:))');
      
      if(distance1 <=  300)           
            D2(i,1) = distance1;
            x1 = ZI(1,1);
            y1 = ZI(1,2);
            x2 = ZJ(i,1);
            y2 = ZJ(i,2);
            
            if(x1==x2 && y1==y2)                 
                 continue;
            end
%             for k = 0:100
%                 if(x2==x1)
%                          dely = y1+k*(y2-y1)/100; 
%                          if(size(Out_binaryimage,2) >= dely && dely >=0)
%                              if(double(Out_binaryimage(x1,round(dely))) < 50)
%                                  D2(i,1) = realmax;                             
%                                  break;                                       
%                              end
%                          end
%                     elseif(y2==y1)
%                          delx = x1+k*(x2-x1)/100; 
%                          if(size(Out_binaryimage,1) >= delx && delx >=0)
%                              if(double(Out_binaryimage(round(delx),y1)) < 50)
%                                 D2(i,1) = realmax;                             
%                                  break;                                         
%                              end
%                          end                        
%                 else
%                             a = (y2-y1)/(x2-x1);
%                             x = (x1+k*(x2-x1)/100);                     
%                             b = y1-a*x1;      
%                             if(size(Out_binaryimage,1) >= round(x) && round(x) >=0 && round(a*round(x)+b) >=0 && size(Out_binaryimage,2) >= round(a*round(x)+b))
%                                  if(double(Out_binaryimage(round(x),round(a*round(x)+b))) < 50)
%                                      D2(i,1) = realmax;                             
%                                      break;   
%                                  end
%                             end
%                 end
%             end
      end
   end     
%  D2
%  pause;