function [coordiy,coordix,idxx,assign_ok]=region_assign_probabilistic(coordiy,coordix,idxx,i,target_robotnum1,assign_region_num,range)

         %n 개 vs m 개 사이 거리가 가장 가까운 노드 선택 
          
           
                     
                     
         for j=1:assign_region_num
            aa = repmat(coordiy{target_robotnum1}(:),size(coordiy{i},2),1)- kron(coordiy{i}(:),ones(size(coordiy{target_robotnum1},2),1));
            bb = repmat(coordix{target_robotnum1}(:),size(coordix{i},2),1)- kron(coordix{i}(:),ones(size(coordix{target_robotnum1},2),1));
            
         [sort_dist,sort_idx] = sort(sqrt(aa.^2+bb.^2));
         
          sort_idx = unique(sort_idx,'stable');
          target_node = floor(rem(sort_idx,size(coordiy{target_robotnum1}(:),1)));
          target_node = unique(target_node,'stable');
          temp_node = target_node(2);
          target_node = target_node(1:1);   
          
          
          
          if(target_node == 0)
              target_node = size(coordiy{target_robotnum1}(:),1);
          end
          if(sort_dist(1) < range)
              tempy = coordiy;
              tempx = coordix;
              tempidx = idxx;             
              
              tempy{i} = [tempy{i} tempy{target_robotnum1}(target_node)];
             tempx{i} = [tempx{i} tempx{target_robotnum1}(target_node)];
             tempidx{i} = [tempidx{i} tempidx{target_robotnum1}(target_node)];
              tempy{target_robotnum1}(target_node) = [];
             tempx{target_robotnum1}(target_node) = [];
             tempidx{target_robotnum1}(target_node) = [];
             
             
             tempy1 = coordiy;
              tempx1 = coordix;
              tempidx1 = idxx;
               tempy1{i} = [tempy1{i} tempy1{target_robotnum1}(temp_node)];
             tempx1{i} = [tempx1{i} tempx1{target_robotnum1}(temp_node)];
             tempidx1{i} = [tempidx1{i} tempidx1{target_robotnum1}(temp_node)];
              tempy1{target_robotnum1}(temp_node) = [];
             tempx1{target_robotnum1}(temp_node) = [];
             tempidx1{target_robotnum1}(temp_node) = [];
             
%              optimalK = 4;
%                a = [tempx{target_robotnum1}(:) tempy{target_robotnum1}(:)];                   
%                [idx,C,sumd,D] = kmeans(a,1);%(a,'kmeans','CalinskiHarabasz','KList',[1:5])
%                computed_criteria = var(eva.CriterionValues(2:optimalK))/eva.CriterionValues(2:2);%-eva.CriterionValues(optimalK-3));
% 
%                 b = [tempx{i}(:) tempy{i}(:)];                   
%                 eva = evalclusters(b,'kmeans','CalinskiHarabasz','KList',[1:5]) ;                 
%                 computed_criteria2 = var(eva.CriterionValues(2:optimalK))/eva.CriterionValues(2:2);%abs(eva.CriterionValues(optimalK)-eva.CriterionValues(optimalK-3));
%                 
%                 
%                a = [tempx1{target_robotnum1}(:) tempy1{target_robotnum1}(:)];                   
%                eva = evalclusters(a,'kmeans','CalinskiHarabasz','KList',[1:5])
%                computed_criteria1 =var(eva.CriterionValues(2:optimalK))/eva.CriterionValues(2:2);%abs(eva.CriterionValues(optimalK)-eva.CriterionValues(optimalK-3));
% 
%                 b = [tempx1{i}(:) tempy1{i}(:)];                   
%                 eva = evalclusters(b,'kmeans','CalinskiHarabasz','KList',[1:5]) ;                 
%                 computed_criteria21 = var(eva.CriterionValues(2:optimalK))/eva.CriterionValues(2:2);%abs(eva.CriterionValues(optimalK)-eva.CriterionValues(optimalK-3));
%               
%               
               a = [tempx{target_robotnum1}(:) tempy{target_robotnum1}(:)];                   
               [idx,C,sumd,D] = kmeans(a,1);%(a,'kmeans','CalinskiHarabasz','KList',[1:5])
               computed_criteria = sumd;%var(eva.CriterionValues(2:optimalK))/eva.CriterionValues(2:2);%-eva.CriterionValues(optimalK-3));

                b = [tempx{i}(:) tempy{i}(:)];                   
                 [idx,C,sumd,D] = kmeans(b,1);
                 computed_criteria2 = sumd;
                
               a = [tempx1{target_robotnum1}(:) tempy1{target_robotnum1}(:)];                   
                 [idx,C,sumd,D] = kmeans(a,1);
                 computed_criteria1 = sumd;
                b = [tempx1{i}(:) tempy1{i}(:)];                   
                  [idx,C,sumd,D] = kmeans(b,1);
                 computed_criteria21 = sumd;
              if(computed_criteria >= computed_criteria1)
                 coordiy{i} = [coordiy{i} coordiy{target_robotnum1}(temp_node)];
                 coordix{i} = [coordix{i} coordix{target_robotnum1}(temp_node)];
                 idxx{i} = [idxx{i} idxx{target_robotnum1}(temp_node)];
                  coordiy{target_robotnum1}(temp_node) = [];
                 coordix{target_robotnum1}(temp_node) = [];
                 idxx{target_robotnum1}(temp_node) = [];
              else
                  coordiy{i} = [coordiy{i} coordiy{target_robotnum1}(target_node)];
                 coordix{i} = [coordix{i} coordix{target_robotnum1}(target_node)];
                 idxx{i} = [idxx{i} idxx{target_robotnum1}(target_node)];
                  coordiy{target_robotnum1}(target_node) = [];
                 coordix{target_robotnum1}(target_node) = [];
                 idxx{target_robotnum1}(target_node) = [];
              end
             
             
             
             
             
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