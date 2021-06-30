function [ccoordix,ccoordiy,idx2Region]=Kmeans_based_ClusteringII(coordix,coordiy,area_num) %%1x32, 60 %% 1x5 10
%% 경로로부터 각로봇당 경로 할당 알고리즘
%% cooridx: 분할할 x좌표, coordiy: 분할할 y좌표, area_num: 영역분할개수

        
idx2Region = kmeans([coordiy',coordix'],area_num); 
        
         ccoordix{area_num} = [];
         ccoordiy{area_num} = [];
         
        count = 1;
        for  i=1:size(coordiy,2)          
            assigned_num = idx2Region(count:count+size(coordiy{i},2)-1);
            max_num = 0;
            m=1
            for k=1:area_num
                each_robot_assigned =  sum(assigned_num(:) == k);
                if(each_robot_assigned > max_num)
                    max_num = each_robot_assigned;
                    assigned_robot = k;
                end
            end
            
             ccoordix{assigned_robot} = [ccoordix{assigned_robot} coordix{i}];
            ccoordiy{assigned_robot} = [ccoordiy{assigned_robot} coordiy{i}];
            count = count+ size(coordiy{i},2);
            
        end