function [shortestPath,shortestPathLength] = onewaypath(binaryImage,x,y,dmat,idxx,L_range,Rangeconstant)
%% 각 로봇당 최종 이동 노드들에 대해 실제 순찰 TRAJECTORY 생성
% dmat = [];
% nPoints = 0;
% max_value = 100000;%realmax
% if isempty(dmat)
%     nPoints = size(x,1);
%     a = meshgrid(1:nPoints);
%     dmat = reshape(max_value*ones(nPoints*nPoints,1),nPoints,nPoints) ;
%     dmat = dmat - diag(diag(dmat));
% end   
% for i=1:nPoints 
%    [tndist,tnidx] = sort(sqrt((x(i) -  x(1:nPoints,1)).^2+(y(i) -  y(1:nPoints,1)).^2));
%    idx =  tnidx(tndist < L_range+L_range*Rangeconstant*2)   ;
%   
%   for j=1:size(idx,1)  
%     x1 = x(i,1);
%     y1 = y(i,1);
%     x2 = x(idx(j),1);
%     y2 = y(idx(j),1);
%                 
%    if(x1==x2 && y1==y2)             
%         dmat(i,idx(j)) = tndist(j);
%         dmat(idx(j),i) = tndist(j);
%         continue;
%    end
%       for k = 0:100                    
%         if(x2==x1)
%              dely = y1+k*(y2-y1)/100; 
%              if(size(binaryImage,2) >= dely && dely >=0)
%              if(double(binaryImage(round(x1),round(dely))) > 50 && double(binaryImage(round(x1),round(dely)))<190)
%                  dmat(i,idx(j)) =max_value;
%                  dmat(idx(j),i) =max_value;
%                  break;
%             else
%                  dmat(i,idx(j)) = tndist(j);
%                  dmat(idx(j),i) = tndist(j);                    
%              end
%              end
%         elseif(y2==y1)
%              delx = x1+k*(x2-x1)/100; 
%              if(size(binaryImage,1) >= delx && delx >=0)
%              if(double(binaryImage(round(delx),round(y1))) > 50 && double(binaryImage(round(delx),round(y1)))<190)
%                  dmat(i,idx(j)) =max_value;
%                  dmat(idx(j),i) =max_value;
%                  break;
%             else
%                  dmat(i,idx(j)) = tndist(j);
%                  dmat(idx(j),i) = tndist(j);                    
%              end
%              end                        
%         else
%                 a = (y2-y1)/(x2-x1);
%                 xtest = (x1+k*(x2-x1)/100);                     
%                 b = y1-a*x1;      
%                 if(size(binaryImage,1) >= round(xtest) && round(xtest) >=0 && round(a*round(xtest)+b) >=0 && size(binaryImage,2) >= round(a*round(xtest)+b))
%                  if(double(binaryImage(round(xtest),round(a*round(xtest)+b))) > 50 && double(binaryImage(round(xtest),round(a*round(xtest)+b)))<190)
%                      dmat(i,idx(j)) =max_value;
%                      dmat(idx(j),i) =max_value;
%                      break;
%                   else
%                      dmat(i,idx(j)) = tndist(j);
%                      dmat(idx(j),i) = tndist(j);                    
%                  end
%                 end
%         end  
%       end  
%                 
%   end     
% end

dmat = dmat(idxx,idxx);


shortestPath = [];
N_cities = size(x,1);
totalcount = 1;
   zerovector = zeros(size(x,1),1);
  %%algorithm
      shortestPathLength = realmax;
for i =  1:N_cities
    totalcount = 1;
    path = [];
  bool_unique_all_vertex = zerovector;
    startCity = i;
    path = [path;startCity];
    distanceTraveled = 0;
    distancesNew = dmat;
    currentCity = startCity; 
    bool_unique_all_vertex(startCity) = 1;
 while(isempty(find(bool_unique_all_vertex==0))==0)
    [minDist,nextCity] = sort(distancesNew(:,currentCity));
    inextCity = nextCity(1);
    iminDist = minDist(1);
    p_count = 2;
    while (inextCity == currentCity || iminDist==0)
        inextCity = nextCity(p_count);
        iminDist = minDist(p_count);   
        p_count = p_count+1;
    end
    nextCity_debug = inextCity;
    path(end+1,1) = inextCity;
    distanceTraveled = distanceTraveled +...
    dmat(currentCity,inextCity);
    distancesNew(currentCity,:) =   distancesNew(currentCity,:)*2;
%     distancesNew(:,currentCity) =   distancesNew(:,currentCity)*2;

    currentCity = inextCity;
    bool_unique_all_vertex(currentCity) = 1;              

      if (distanceTraveled > shortestPathLength)
          continue;
      end
      totalcount = totalcount+1;
      if(totalcount>10000)
          break;
      end
%               pause;
 end
    if (distanceTraveled < shortestPathLength)
        shortestPathLength = distanceTraveled;
        current_cityname = i;
        shortestPath = path; 
    end             
end