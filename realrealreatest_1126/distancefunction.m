function dmat = distancefunction(unique_all_vertex, binaryImage,L_range,Rangeconstant,max_value)
dmat = [];
% L_range = 30;
% realmax
max_itr = 100;
%    if isempty(dmat)
        nPoints = size(unique_all_vertex,1);
        a = meshgrid(1:nPoints);
        dmat = reshape(max_value*ones(nPoints*nPoints,1),nPoints,nPoints) ;
        dmat = dmat - diag(diag(dmat));
%    end
   nPoints = size(unique_all_vertex,1);
   for i=1:size(unique_all_vertex,1) 
       [tndist,tnidx] = sort(sqrt((unique_all_vertex(i,1) -  unique_all_vertex(1:nPoints,1)).^2+(unique_all_vertex(i,2) -  unique_all_vertex(1:nPoints,2)).^2)); 
      idx =  tnidx(tndist < L_range+L_range*Rangeconstant); %% 일정거리 이내의 점 끼리 연결 센싱 range 변수 L_range 활용
%       pause;
        %% for문을 통해 dmat 함수(거리값)를 채움.
       for j=1:size(idx,1)%size(unique_all_vertex,1)   
%            xv = [unique_all_vertex(i,1);unique_all_vertex(i,1);unique_all_vertex(tnidx(j),1);unique_all_vertex(tnidx(j),1);unique_all_vertex(i,1)];
%            yv = [unique_all_vertex(i,2);unique_all_vertex(tnidx(j),2);unique_all_vertex(tnidx(j),2);unique_all_vertex(i,2);unique_all_vertex(i,2)];
%            plot(xv,yv);
%            plot(unique_all_vertex(48,1),unique_all_vertex(48,2),'or');
%            [in,on] = inpolygon(outlines(:,1),outlines(:,2),xv,yv);
           
%            if(numel(outlines(in))==0)
%                 ndist = (sqrt((unique_all_vertex(i,1) -  unique_all_vertex(idx(j),1)).^2+(unique_all_vertex(i,2) -  unique_all_vertex(tnidx(j),2)).^2));

                x1 = unique_all_vertex(i,1);
                y1 = unique_all_vertex(i,2);
                x2 = unique_all_vertex(idx(j),1);
                y2 = unique_all_vertex(idx(j),2);
                if(x1==x2 && y1==y2)
                     dmat(i,idx(j)) = tndist(j);
                     dmat(idx(j),i) = tndist(j);  
                     continue;
                end
                for k = 0:max_itr                    
                    if(x2==x1)
                         dely = y1+k*(y2-y1)/max_itr; 
                         dely = round(dely);
                         if(size(binaryImage,2) >= dely && dely >=0)
                         if(double(binaryImage(round(x1),round(dely))) < 50)
                             dmat(i,idx(j)) =max_value;
                             dmat(idx(j),i) =max_value;
                             break;
                        else
                             dmat(i,idx(j)) = tndist(j);
                             dmat(idx(j),i) = tndist(j);                    
                         end
                         end
                    elseif(y2==y1)
                         delx = x1+k*(x2-x1)/max_itr; 
                         delx = round(delx);
                         if(size(binaryImage,1) >= delx && delx >=0)                            
                             if(double(binaryImage(delx,round(y1))) < 50)
                                 dmat(i,idx(j)) =max_value;
                                 dmat(idx(j),i) =max_value;
                                 break;
                            else
                                 dmat(i,idx(j)) = tndist(j);
                                 dmat(idx(j),i) = tndist(j);                    
                             end
                         end                        
                    else
                            a = (y2-y1)/(x2-x1);
                            x = (x1+k*(x2-x1)/100);      
                            x = round(x);
                            b = y1-a*x1;      
                            if(size(binaryImage,1) >= round(x) && round(x) >=0 && round(a*round(x)+b) >=0 && size(binaryImage,2) >= round(a*round(x)+b))
                             if(double(binaryImage(round(x),round(a*round(x)+b))) < 50)
                                 dmat(i,idx(j)) =max_value;
                                 dmat(idx(j),i) =max_value;
                                 break;
                              else
                                 dmat(i,idx(j)) = tndist(j);
                                 dmat(idx(j),i) = tndist(j);                    
                             end
                            end
                    end  
                end                   
                                      
       end
   end