function [ccoordix,ccoordiy,idx,cluster_save]=Kmeans_based_Clustering(unique_all_vertex_base,coordix,coordiy,idxx,D_RobotNum,Debug_Plot,w,h,L_range,Rangeconstant) %%1x32, 60 %% 1x5 10
%% 경로로부터 각로봇당 경로 할당 알고리즘
%% cooridx: 경로 클러스터 x좌표, coordiy: 경로 클러스터 y좌표, D_RobotNum: 로봇대수
avg_node = 0;
nodenum = [];
yindex = [];
xindex = [];        
total_node = 0; 
% idx = [];
  for i=1:size(coordiy,2)
      total_node = total_node + size(coordiy{i},2);
      nodenum = [nodenum size(coordiy{i},2)];
      yindex = [yindex coordiy{i}];
      xindex = [xindex coordix{i}];
  end
yindex = [yindex unique_all_vertex_base(:,2)'];
xindex = [xindex unique_all_vertex_base(:,1)'];
avg_node = total_node/D_RobotNum;
[cluster_val,cluster_order] = sort(nodenum,'descend');                
% idx2Region = spectralcluster([yindex',xindex'],D_RobotNum,'Distance','mahalanobis');%kmeans([yindex',xindex'],D_RobotNum); 
idx2Region = kmeans([yindex',xindex'],D_RobotNum,'MaxIter',3); 
        
         ccoordix{D_RobotNum} = [];
         ccoordiy{D_RobotNum} = [];
         idx{D_RobotNum} = [];
         cluster_save{D_RobotNum} = [];
         clustersize_save{D_RobotNum} = [];
         count = 1;
        for  i=1:size(coordiy,2)  
            assigned_num = idx2Region(count:count+size(coordiy{i},2)-1);
            max_num = 0;
            candidate_num = [];
            for k=1:D_RobotNum
                each_robot_assigned =  sum(assigned_num(:) == k);                
                if(each_robot_assigned > max_num && (size(ccoordix{k},2) < avg_node)) %% 0824 수정
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
                idx{assigned_robot} = [idx{assigned_robot} idxx{i}];   
                
                cluster_save{assigned_robot} = [cluster_save{assigned_robot} i];   
                clustersize_save{assigned_robot} = [clustersize_save{assigned_robot} size(coordix{i},2)];   
              
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
            %%Plot 작성
             if(Debug_Plot==1)
                 if(assigned_robot==1)
                      map_division_end = 0;
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
            count = count+ size(coordiy{i},2);
        end
        
        
        %% 각 로봇에 할당된 모든 클러스터를 소환하면서 개별 클러스터간 차이가 sensing 반경 내에 있는 지 체크하는 구문
%         cluster_save
%         clustersize_save
%         ccoordiy
%         pause;
%         
%         for i = 1:D_RobotNum
%             yxis = [];
%             xxis = [];
%             compensation = 0; % false;
%             cluster_size = (clustersize_save{i});
%           %  size(cluster_save{i},2)
%             if(size(cluster_size,2)<2)
%                 continue;
%             end
%            for j=1:size(cluster_save{i},2)
% %                if(cluster_size(j) > 5)
% %                    continue;
% %                end
%                    
%                yxis = [coordiy{cluster_save{i}(j)}];
%                xxis = [coordix{cluster_save{i}(j)}];
%                cur_index = sum(cluster_size(1:end))-sum(cluster_size(j:end))+1;
%                
%                compx = ccoordix{i};
%                compy = ccoordiy{i};
%              
% %                cur_index+cluster_size(j)-1
%                compx(cur_index:cur_index+cluster_size(j)-1)=[];
%                compy(cur_index:cur_index+cluster_size(j)-1)=[];
%               
%            
%                 aa = repmat(yxis',size(compy,2),1)- kron(compy',ones(size(yxis,2),1));
%                 bb = repmat(xxis',size(compx,2),1)- kron(compx',ones(size(xxis,2),1));            
%                 cc = sqrt(aa.^2+bb.^2);          
%           
%              if(min(cc) >= L_range*4)
%                  min(cc) 
%                  i
%                  Long_dist = 1
% %                  yxis
% %                  xxis
% %                  ccoordix
% %                  pause;
%                  for k = 1:D_RobotNum
%                      if(i~=k)
%                          compx = ccoordix{k};
%                          compy = ccoordiy{k};
%                          
%                          aa = repmat(yxis',size(compy,2),1)- kron(compy',ones(size(yxis,2),1));
%                          bb = repmat(xxis',size(compx,2),1)- kron(compx',ones(size(xxis,2),1));
%                          cc = sqrt(aa.^2+bb.^2);
%                          
%                           if(min(cc) <= L_range*4)
%                               k
%                               ccoordix{k}=[ccoordix{k} xxis];
%                               ccoordiy{k}=[ccoordiy{k} yxis];
%                                 cluster_save{k} = [cluster_save{k} cluster_save{i}(j)]  ;
%                                 clustersize_save{k} = [clustersize_save{k} size(xxis,2)]     ;       
%                                     
%                                 ccoordix{i}(cur_index:cur_index+cluster_size(j)-1)=[];
%                                 ccoordiy{i}(cur_index:cur_index+cluster_size(j)-1)=[];               
%                                 cluster_save{i}(j) = [];
%                                 clustersize_save{i}(j) = [];
%                                 compensation = 1;
%                                 break;
%                           end
%                      end                   
%                  end
%                  if(compensation == 1)
%                      break;
%                  end
%              end             
%            end
%         end
%         cluster_save
%         clustersize_save
%         ccoordiy
%         pause;
%        
%         for i = 1:D_RobotNum
%          %%Plot 작성
%          assigned_robot = i;
%              if(Debug_Plot==1)
%                  if(assigned_robot==1)
%                       map_division_end = 0
%                    %   scatter(ccoordiy{assigned_robot}(:),ccoordix{assigned_robot}(:),'o','MarkerEdgeColor',[0 0 0],'MarkerFaceColor',[1 0 0],'LineWidth',2);        
% 
%                     for drawi = 1:size(coordiy{i},2)
%                         rectangle('Position',[coordiy{i}(drawi)-w,coordix{i}(drawi)-h,2*w,2*h],'FaceColor', [0.5 0 0 0.1],'EdgeColor','none');
%                     end
%                  %   pause(1);
%                 elseif(assigned_robot==2)
%           %     %     scatter(ccoordiy{assigned_robot}(:),ccoordix{assigned_robot}(:),'o','MarkerEdgeColor',[0 0 0],'MarkerFaceColor',[0 1 0],'LineWidth',2);        
%                     for drawi = 1:size(coordiy{i},2)
%                         rectangle('Position',[coordiy{i}(drawi)-w,coordix{i}(drawi)-h,2*w,2*h],'FaceColor', [0 0.5 0 0.1],'EdgeColor','none');
%                     end
%                 %    pause(1);
%                 elseif(assigned_robot==3)
%                 %    scatter(ccoordiy{assigned_robot}(:),ccoordix{assigned_robot}(:),'o','MarkerEdgeColor',[0 0 0],'MarkerFaceColor',[0 0 1],'LineWidth',2);       
%                     for drawi = 1:size(coordiy{i},2)
%                         rectangle('Position',[coordiy{i}(drawi)-w,coordix{i}(drawi)-h,2*w,2*h],'FaceColor', [0 0 0.5 0.1],'EdgeColor','none');
%                     end
%                 %    pause(1);
%                 elseif(assigned_robot==4)
%                     %scatter(ccoordiy{assigned_robot}(:),ccoordix{assigned_robot}(:),'o','MarkerEdgeColor',[0 1 1],'MarkerFaceColor',[0 1 1],'LineWidth',2);       
%                     for drawi = 1:size(coordiy{i},2)
%                         rectangle('Position',[coordiy{i}(drawi)-w,coordix{i}(drawi)-h,2*w,2*h],'FaceColor', [0 0.5 0.5 0.1],'EdgeColor','none');
%                     end
%                %     pause(1);
%                 elseif(assigned_robot==5)
%                    % scatter(ccoordiy{assigned_robot}(:),ccoordix{assigned_robot}(:),'o','MarkerEdgeColor',[1 1 0],'MarkerFaceColor',[1 1 0],'LineWidth',2);       
%                     for drawi = 1:size(coordiy{i},2)
%                         rectangle('Position',[coordiy{i}(drawi)-w,coordix{i}(drawi)-h,2*w,2*h],'FaceColor', [0.5 0.5 0 0.1],'EdgeColor','none');
%                     end
%                 %    pause(1);
%                      elseif(assigned_robot==6)
%                    % scatter(ccoordiy{assigned_robot}(:),ccoordix{assigned_robot}(:),'o','MarkerEdgeColor',[1 0 1],'MarkerFaceColor',[1 0 1],'LineWidth',2);        
%                     for drawi = 1:size(coordiy{i},2)
%                         rectangle('Position',[coordiy{i}(drawi)-w,coordix{i}(drawi)-h,2*w,2*h],'FaceColor', [0.5 0 0.5 0.1],'EdgeColor','none');
%                     end
%                %     pause(1);
%                  end
%              end
%         end

       