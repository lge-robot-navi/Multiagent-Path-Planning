function [ccoordix,ccoordiy]=Kmeans_based_Clustering(coordix,coordiy,D_RobotNum,Debug_Plot,w,h,L_range,Rangeconstant) %%1x32, 60 %% 1x5 10
%% 경로로부터 각로봇당 경로 할당 알고리즘
%% cooridx: 경로 클러스터 x좌표, coordiy: 경로 클러스터 y좌표, D_RobotNum: 로봇대수
avg_node = 0;
nodenum = [];
yindex = [];
xindex = [];        
total_node = 0; 
  for i=1:size(coordiy,2)
      total_node = total_node + size(coordiy{i},2);
      nodenum = [nodenum size(coordiy{i},2)];
      yindex = [yindex coordiy{i}];
      xindex = [xindex coordix{i}];
  end

avg_node = total_node/D_RobotNum;
[cluster_val,cluster_order] = sort(nodenum,'descend');                
idx2Region = kmeans([yindex',xindex'],D_RobotNum); 
        
         ccoordix{D_RobotNum} = [];
         ccoordiy{D_RobotNum} = [];
         cluster_save{D_RobotNum} = [];
              count = 1;
        for  i=1:size(coordiy,2)  
            assigned_num = idx2Region(count:count+size(coordiy{i},2)-1);
            max_num = 0;
            candidate_num = [];
            for k=1:D_RobotNum
                each_robot_assigned =  sum(assigned_num(:) == k);                
                if(each_robot_assigned > max_num)
                    max_num = each_robot_assigned;
                    assigned_robot = k;
                end
            end
            
            %%연결성을 봐야함
            
%             if(assigned_robot==1)
%                 plot(coordiy{i},coordix{i},'xb');
%             elseif(assigned_robot==2)
%                 plot(coordiy{i},coordix{i},'xr');
%             elseif(assigned_robot==3)                    
%                 plot(coordiy{i},coordix{i},'xg');
%             end
%             pause;
%             
%               count = count+ size(coordiy{i},2);
%         end
            
            
%             if(isempty(ccoordix{assigned_robot}))
                ccoordix{assigned_robot} = [ccoordix{assigned_robot} coordix{i}];
                ccoordiy{assigned_robot} = [ccoordiy{assigned_robot} coordiy{i}];             
                cluster_save{assigned_robot} = [cluster_save{assigned_robot} size(ccoordix{assigned_robot},2)];                    
              
%             else
%                 dist1 = 100000;
%                for jj = 1:size(coordix{i},2)
%                     tempdist = min(sqrt((ccoordix{assigned_robot}-coordix{i}(jj)).^2+(ccoordiy{assigned_robot}-coordiy{i}(jj)).^2));
%                     if(tempdist < dist1)
%                         tempdist = dist1;
%                     end
%                end
%                  if(dist1 < L_range+L_range*Rangeconstant*1.5 && size(ccoordix{assigned_robot},2) < avg_node*1.5) %%가깝다면, 
%                     ccoordix{assigned_robot} = [ccoordix{assigned_robot} coordix{i}];
%                     ccoordiy{assigned_robot} = [ccoordiy{assigned_robot} coordiy{i}];
%                  end
            
%             end
            m=2
            
           
            
           
            m=3
            %%Plot 작성
             if(Debug_Plot==1)
                 if(assigned_robot==1)
                      map_division_end = 0
                   %   scatter(ccoordiy{assigned_robot}(:),ccoordix{assigned_robot}(:),'o','MarkerEdgeColor',[0 0 0],'MarkerFaceColor',[1 0 0],'LineWidth',2);        

                    for drawi = 1:size(coordiy{i},2)
                        rectangle('Position',[coordiy{i}(drawi)-w,coordix{i}(drawi)-h,2*w,2*h],'FaceColor', [0.5 0 0 0.1],'EdgeColor','none');
                    end
                 %   pause(1);
                elseif(assigned_robot==2)
          %     %     scatter(ccoordiy{assigned_robot}(:),ccoordix{assigned_robot}(:),'o','MarkerEdgeColor',[0 0 0],'MarkerFaceColor',[0 1 0],'LineWidth',2);        
                    for drawi = 1:size(coordiy{i},2)
                        rectangle('Position',[coordiy{i}(drawi)-w,coordix{i}(drawi)-h,2*w,2*h],'FaceColor', [0 0.5 0 0.1],'EdgeColor','none');
                    end
                %    pause(1);
                elseif(assigned_robot==3)
                %    scatter(ccoordiy{assigned_robot}(:),ccoordix{assigned_robot}(:),'o','MarkerEdgeColor',[0 0 0],'MarkerFaceColor',[0 0 1],'LineWidth',2);       
                    for drawi = 1:size(coordiy{i},2)
                        rectangle('Position',[coordiy{i}(drawi)-w,coordix{i}(drawi)-h,2*w,2*h],'FaceColor', [0 0 0.3 0.1],'EdgeColor','none');
                    end
                %    pause(1);
                elseif(assigned_robot==4)
                    %scatter(ccoordiy{assigned_robot}(:),ccoordix{assigned_robot}(:),'o','MarkerEdgeColor',[0 1 1],'MarkerFaceColor',[0 1 1],'LineWidth',2);       
                    for drawi = 1:size(coordiy{i},2)
                        rectangle('Position',[coordiy{i}(drawi)-w,coordix{i}(drawi)-h,2*w,2*h],'FaceColor', [0 0.3 0.3 0.1],'EdgeColor','none');
                    end
               %     pause(1);
                elseif(assigned_robot==5)
                   % scatter(ccoordiy{assigned_robot}(:),ccoordix{assigned_robot}(:),'o','MarkerEdgeColor',[1 1 0],'MarkerFaceColor',[1 1 0],'LineWidth',2);       
                    for drawi = 1:size(coordiy{i},2)
                        rectangle('Position',[coordiy{i}(drawi)-w,coordix{i}(drawi)-h,2*w,2*h],'FaceColor', [0.3 0.3 0 0.1],'EdgeColor','none');
                    end
                %    pause(1);
                     elseif(assigned_robot==6)
                   % scatter(ccoordiy{assigned_robot}(:),ccoordix{assigned_robot}(:),'o','MarkerEdgeColor',[1 0 1],'MarkerFaceColor',[1 0 1],'LineWidth',2);        
                    for drawi = 1:size(coordiy{i},2)
                        rectangle('Position',[coordiy{i}(drawi)-w,coordix{i}(drawi)-h,2*w,2*h],'FaceColor', [0.3 0 0.3 0.1],'EdgeColor','none');
                    end
               %     pause(1);
                 end
             end
              m=4 
           
            count = count+ size(coordiy{i},2);
        end
        
        
        for i = 1:D_RobotNum
            cluster_save{assigned_robot} = [cluster_save{assigned_robot} size(ccoordix{assigned_robot},2)];  
        end
       