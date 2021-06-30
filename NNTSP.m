function [shortestPath,shortestPathLength]=NNTSP(dmat,unique_all_vertex,Debug_Plot,cost)
 N_cities = size(unique_all_vertex,1);
   zerovector = zeros(size(unique_all_vertex,1),1);
  %%algorithm
      shortestPathLength = 10^8;
        shortestPath = [];
        for i = 1:N_cities
          bool_unique_all_vertex = zerovector;
            startCity = i;
            path = startCity;
            distanceTraveled = 0;
            distancesNew = dmat;
            currentCity = startCity; 
            bool_unique_all_vertex(startCity) = 1;
         while(isempty(find(bool_unique_all_vertex==0))==0)
                [minDist,nextCity] = sort(distancesNew(:,currentCity));
                minDist_debug = minDist(1:5);
                nextCity_debug = nextCity(1:5);
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
                distancesNew(currentCity,:) =   distancesNew(currentCity,:)*cost;
                 distancesNew(:,currentCity) =   distancesNew(:,currentCity)*cost;

                 distancesNew(currentCity,inextCity) =   distancesNew(currentCity,inextCity)*cost;
                 distancesNew(inextCity,currentCity) =   distancesNew(inextCity,currentCity)*cost;
                currentCity = inextCity;
                bool_unique_all_vertex(currentCity) = 1;
              if (distanceTraveled > shortestPathLength)
                  continue;
              end
         end
            if (distanceTraveled < shortestPathLength)
                shortestPathLength = distanceTraveled;
                current_cityname = i;
                shortestPath = path; 
            end
        end       
        original = shortestPath;
%        if(Debug_Plot==1)
%             for i =2:size(shortestPath,1)
%                      x  = [unique_all_vertex(shortestPath(i-1),1);unique_all_vertex(shortestPath(i),1)];                 
%                      y = [unique_all_vertex(shortestPath(i-1),2);unique_all_vertex(shortestPath(i),2)];                  
%                      plot(y,x,'k','LineWidth',3);
%                      pause(0.5);
%             end
%        end