function [coordix,coordiy,current_robot,idx,PathLength] = pathdivision(dmat,unique_all_vertex,unique_shortestPath,patrol_distance)
%%전체 중복없는 경로 중 이어지는 경로들로 N개의 경로 Cluster 추출
%% dmat: graph, unique_all_vertex: 전체 경로 노드, shortestPath: 최단경로,  patrol_distance: 순찰 조건

num_of_robot = 1000;
current_robot = 1;
coordix{num_of_robot+1} = [];
coordiy{num_of_robot+1} = [];
idx{num_of_robot+1} = [];
PathLength=zeros(num_of_robot+1,1);
dist = 0;
for i = 1:size(unique_shortestPath,1)
    if(i > 1)        
        if(dmat(unique_shortestPath(i-1),unique_shortestPath(i)) > patrol_distance || dist > patrol_distance)        
            PathLength(current_robot) = dist;
            current_robot = current_robot+1;
            coordix{current_robot} = [coordix{current_robot} unique_all_vertex(unique_shortestPath(i),1)];
            coordiy{current_robot} = [coordiy{current_robot} unique_all_vertex(unique_shortestPath(i),2)];   
            idx{current_robot} = [idx{current_robot} unique_shortestPath(i)];    
            dist = 0;
        else
            dist = dist+dmat(unique_shortestPath(i-1),unique_shortestPath(i));         
             coordix{current_robot} = [coordix{current_robot} unique_all_vertex(unique_shortestPath(i),1)];
             coordiy{current_robot} = [coordiy{current_robot} unique_all_vertex(unique_shortestPath(i),2)]; 
             idx{current_robot} = [idx{current_robot} unique_shortestPath(i)];   
             PathLength(current_robot) = dist;
        end
        PathLength(current_robot) = dist;
    else
          idx{current_robot} = [idx{current_robot} unique_shortestPath(i)];   
          coordix{current_robot} = [coordix{current_robot} unique_all_vertex(unique_shortestPath(i),1)];
          coordiy{current_robot} = [coordiy{current_robot} unique_all_vertex(unique_shortestPath(i),2)]; 
          PathLength(current_robot) = dist;
    end
end
        
       
        