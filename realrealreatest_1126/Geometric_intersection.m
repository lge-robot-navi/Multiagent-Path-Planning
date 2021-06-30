function [ccoordix,ccoordiy,idx,cluster_save]=Geometric_intersection(unique_all_vertex_base,coordix,coordiy,idxx,D_RobotNum,Debug_Plot,w,h,L_range,Rangeconstant) %%1x32, 60 %% 1x5 10

label = [];
XX = [];
index_list = [];
n_reference = floor(size(coordix,2)/D_RobotNum);

 ccoordix{D_RobotNum} = [];
         ccoordiy{D_RobotNum} = [];
         idx{D_RobotNum} = [];
         cluster_save{D_RobotNum} = [];
         clustersize_save{D_RobotNum} = [];
         
 assigned_robot = 1;        
 count = 1;
 n = n_reference;
for i = 1:size(coordix,2)
    
    if( i >  n)
        count = count+1;
        i
        assigned_robot = assigned_robot+1
        n = n_reference*count
    end
    if(assigned_robot > D_RobotNum)
        assigned_robot = D_RobotNum;
    end
                ccoordix{assigned_robot} = [ccoordix{assigned_robot} coordix{i}];
                ccoordiy{assigned_robot} = [ccoordiy{assigned_robot} coordiy{i}];   
                idx{assigned_robot} = [idx{assigned_robot} idxx{i}];                   

            assigned_robot
            %%Plot 작성
             if(Debug_Plot==1)
                 if(assigned_robot==1)
                      map_division_end = 0;
                     scatter(ccoordiy{assigned_robot}(:),ccoordix{assigned_robot}(:),'o','MarkerEdgeColor',[0 0 0],'MarkerFaceColor',[1 0 0],'LineWidth',2);        

                    for drawi = 1:size(coordiy{i},2)
                        rectangle('Position',[coordiy{i}(drawi)-w,coordix{i}(drawi)-h,2*w,2*h],'FaceColor', [0.5 0 0 0.1],'EdgeColor','none');
                    end
                    hold on;
%                     pause
                 %   pause(1);
                elseif(assigned_robot==2)
                  scatter(ccoordiy{assigned_robot}(:),ccoordix{assigned_robot}(:),'o','MarkerEdgeColor',[0 0 0],'MarkerFaceColor',[0 1 0],'LineWidth',2);        
                    for drawi = 1:size(coordiy{i},2)
                        rectangle('Position',[coordiy{i}(drawi)-w,coordix{i}(drawi)-h,2*w,2*h],'FaceColor', [0 0.5 0 0.1],'EdgeColor','none');
                    end
%                     pause
                %    pause(1);
                elseif(assigned_robot==3)
                   scatter(ccoordiy{assigned_robot}(:),ccoordix{assigned_robot}(:),'o','MarkerEdgeColor',[0 0 0],'MarkerFaceColor',[0 0 1],'LineWidth',2);       
                    for drawi = 1:size(coordiy{i},2)
                        rectangle('Position',[coordiy{i}(drawi)-w,coordix{i}(drawi)-h,2*w,2*h],'FaceColor', [0 0 0.3 0.1],'EdgeColor','none');
                    end
%                     pause
                %    pause(1);
                elseif(assigned_robot==4)
                    scatter(ccoordiy{assigned_robot}(:),ccoordix{assigned_robot}(:),'o','MarkerEdgeColor',[0 1 1],'MarkerFaceColor',[0 1 1],'LineWidth',2);       
                    for drawi = 1:size(coordiy{i},2)
                        rectangle('Position',[coordiy{i}(drawi)-w,coordix{i}(drawi)-h,2*w,2*h],'FaceColor', [0 0.3 0.3 0.1],'EdgeColor','none');
                    end
%                     pause
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
%             count = count+ size(coordiy{i},2);
end
 

       