function [coordiy,coordix,idxx,assign_ok]=region_assign(coordiy,coordix,idxx,i,target_robotnum1,assign_region_num,range)

         %n 개 vs m 개 사이 거리가 가장 가까운 노드 선택 
         
         for j=1:assign_region_num
            aa = repmat(coordiy{target_robotnum1}(:),size(coordiy{i},2),1)- kron(coordiy{i}(:),ones(size(coordiy{target_robotnum1},2),1));
            bb = repmat(coordix{target_robotnum1}(:),size(coordix{i},2),1)- kron(coordix{i}(:),ones(size(coordix{target_robotnum1},2),1));
            
         [sort_dist,sort_idx] = sort(sqrt(aa.^2+bb.^2));
         
          sort_idx = unique(sort_idx,'stable');
          target_node = floor(rem(sort_idx,size(coordiy{target_robotnum1}(:),1)));
                    target_node = unique(target_node,'stable');
          target_node = target_node(1:1);
          if(target_node == 0)
              target_node = size(coordiy{target_robotnum1}(:),1);
          end
          if(sort_dist(1) < range)
             coordiy{i} = [coordiy{i} coordiy{target_robotnum1}(target_node)];
             coordix{i} = [coordix{i} coordix{target_robotnum1}(target_node)];
             idxx{i} = [idxx{i} idxx{target_robotnum1}(target_node)];
              coordiy{target_robotnum1}(target_node) = [];
             coordix{target_robotnum1}(target_node) = [];
             idxx{target_robotnum1}(target_node) = [];
             assign_ok = true;
          else
              assign_ok = false;
          end
         
             
         end
        
%          [idx2Region,Center] = kmeans([coorditesty,coorditestx],area_division_num); 
        
                     
        
        





% 
%          %n 개 vs m 개 사이 거리가 가장 가까운 노드 선택 
%          
%          aa = repmat(coordiy{target_robotnum1}(:),size(coordiy{i},2),1)- kron(coordiy{i}(:),ones(size(coordiy{target_robotnum1},2),1));
%          bb = repmat(coordix{target_robotnum1}(:),size(coordix{i},2),1)- kron(coordix{i}(:),ones(size(coordix{target_robotnum1},2),1));
%          [sort_dist,sort_idx] = sort(sqrt(aa.^2+bb.^2));
%          
%           sort_idx = unique(sort_idx,'stable');
%          
% %           pause;
%           
% %          fix(sort_idx(1:assing_region_num)/size(coordiy{target_robotnum1}(:),1))       
%          target_node = floor(rem(sort_idx,size(coordiy{target_robotnum1}(:),1)));
%          
%          target_node = unique(target_node,'stable');
%          
%          target_node = target_node(1:assing_region_num);         
%         
%          coordiy{target_robotnum1}(target_node);
%         
% %          [idx2Region,Center] = kmeans([coorditesty,coorditestx],area_division_num); 
%          coordiy{i} = [coordiy{i} coordiy{target_robotnum1}(target_node)];
%          coordix{i} = [coordix{i} coordix{target_robotnum1}(target_node)];
%          idxx{i} = [idxx{i} idxx{target_robotnum1}(target_node)];
%         
%                      
%         
%          coordiy{target_robotnum1}(target_node) = [];
%          coordix{target_robotnum1}(target_node) = [];
%          idxx{target_robotnum1}(target_node) = [];  
%         