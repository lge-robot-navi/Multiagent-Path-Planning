function [coordix, coordiy, coorditheta] = Scheduling(robots)
 D_RobotNum = 4;
 Debug_Plot =1;
 w = 50;
h = 50;

load('Parameter','binaryImage','gx','gy','Rangeconstant','max_value','L_range','RGB','RGB2','POI','unique_all_vertex_base');
fprintf('Load Parameter Save completed\n');
load('precompute','dmat','unique_all_vertex','unique_shortestPath','patrol_distance');
patrol_distance = 2255;
% pause;
imshow(binaryImage ); 
fprintf('Load precompute\n');
%%dmat: graph 간 연결도, unique_all_vertex: graph node, shortestPath: TSP Solver결과, patrol_distance: 순찰 거리 조건 
[coordix,coordiy,current_robot,idxx,PathLength1] = pathdivision(dmat,unique_all_vertex,unique_shortestPath,patrol_distance);
   
coordiy= coordiy(~any(cellfun('isempty', coordiy), 1));
coordix= coordix(~any(cellfun('isempty', coordix), 1));
idxx= idxx(~any(cellfun('isempty', idxx), 1));
PathLength1 = PathLength1(1:size(coordix,2));      

%% 경로 길이 재계산 %%현재 의미 X
for k = 1:size(coordiy,2)
  PathLength1(k) = 0;
  if(size(coordiy{k},2) >=2)              
    for j = 2:size(coordiy{k},2)
      PathLength1(k) = PathLength1(k)+sqrt((coordiy{k}(j)-coordiy{k}(j-1))^2+(coordix{k}(j)-coordix{k}(j-1))^2);
    end
  end
end
size(coordix,2) 
% pause;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Overlapping path rejection 및 Unique Path Cluster%%%%%%%%%%%%%%%%%%%%%%%%%%% 

%debug
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%%%%%%%Draw  Path Cluster%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
 %% Draw Path Cluster
if(Debug_Plot==1)
 for i=1:size(coordix,2) 
   hold on;
   color = [rand(1),rand(1),rand(1)];
      color2 = [color 0.1];
     for drawi = 1:size(coordiy{i},2)
        plot(coordiy{i}(drawi),coordix{i}(drawi),'Marker','o','color',color,'LineWidth',4);
        rectangle('Position',[coordiy{i}(drawi)-w,coordix{i}(drawi)-h,2*w,2*h],'FaceColor', color2,'EdgeColor','none');
        pause(0.02);
     end
     pause(1);
 end
%  pause;
end
% pause;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%%%%%%%Draw  Path Cluster%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%         
        if(POI == 1)
             [POI_x,POI_y] = Temporary_Test(until_second_node_group0,L_range,RangeconstantPOI_x,POI_y);
        end
%%%%%%%%%%%%%%%%%%%%%%%%%% %%% Path Cluster 추가 변경 %%%%%%%%%%%%%%%%%%%%%%%%%%% 
    DebugPathlength = PathLength1;
     Debugcurrent_robot=current_robot;
     count = 1;       
        total_node = 0;      
         for i=1:size(coordiy,2)
              total_node = total_node + size(coordiy{i},2);             
         end
          itr = size(coordiy,2);       
           min_length = floor(total_node/(3*D_RobotNum));
 %%% 로봇의 개수에 따른 노드의 개수를 일괄적으로 나눔%%%%%%%%%%%%%%%%%%%%%%%%%%%       (path cluster가 너무 클경우 또 나눔)   
        if(D_RobotNum~=0)
            for i=1:itr
                 %% 0813 수정 /2.5 * -> /3*
                 if(size(coordiy{i},2) > min_length)
                      N = floor(size(coordiy{i},2)/min_length);
                      count = 1;
                      originalcoordy = coordiy{i};
                      originalcoordx = coordix{i};
                      originalidxx = idxx{i};
                      coordiy{i}=coordiy{i}(1:end-(N-1)*min_length);
                      coordix{i}=coordix{i}(1:end-(N-1)*min_length);
                      idxx{i}=idxx{i}(1:end-(N-1)*min_length); 
                while(count < N)
                                   
                      coordiy{end+1}=originalcoordy(end-(N-1)*min_length+1+min_length*(count-1):end-(N-1)*min_length+min_length+min_length*(count-1));
                      coordix{end+1}=originalcoordx(end-(N-1)*min_length+1+min_length*(count-1):end-(N-1)*min_length+min_length+min_length*(count-1));
                      idxx{end+1}=originalidxx(end-(N-1)*min_length+1+min_length*(count-1):end-(N-1)*min_length+min_length+min_length*(count-1));
                       count = count+1;
                     
                     %% 0811 수정                      
%                         itr2 = size(coordiy,2);
%                          while(size(coordiy{itr2},2) > total_node/(2.5*D_RobotNum))                             
%                               coordiy{end+1}=coordiy{itr2}(1+floor(end/2):end);
%                               coordiy{itr2}=coordiy{itr2}(1:floor(end/2));
%                               coordix{end+1}=coordix{itr2}(1+floor(end/2):end);
%                               coordix{itr2}=coordix{itr2}(1:floor(end/2));
%                           %% 0811 수정
%                               %idxx{current_robot} = [idx{current_robot} unique_shortestPath(i)];    
%                                 idxx{end+1}=idxx{itr2}(1+floor(end/2):end);
%                               idxx{itr2}=idxx{itr2}(1:floor(end/2));
%                                 itr2 = size(coordiy,2);
%                              %% 0811 수정
%                          end
                end
%                   itr = size(coordiy,2);  
                 end
            end
        end
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%           
 
%         pause;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Path Cluster Combination %%%%%%%%%%%%%%%%%%%%%%%%%%% 

originaly =  coordiy;
originalx =  coordix;
originalidx = idxx;


 %%%%%%%%%%%%%%%%%%%%%%% 나눠진 순찰 경로를 k-means 방법으로 결합 %%%%%%%%%%%%%%%%%%%%%     
 %% 0807변경
% coordix
% pause;
[ccoordix,ccoordiy,idx,cluster_save]=Kmeans_based_Clustering(unique_all_vertex_base,coordix,coordiy,idxx,D_RobotNum,Debug_Plot,w,h,L_range,Rangeconstant)    ;
 %%%%%%%%%%%%%%%%%%%%%%% 나눠진 순찰 경로를 k-means 방법으로 보정 %%%%%%%%%%%%%%%%%%%%%     

fprintf('Node Assignment completed\n');
 
coordiy = ccoordiy;
coordix = ccoordix;    
idxx = idx;
% coordiy = 


        
 %%%%%%%%%%%%%%%%%%%%%%% 개별 나눠진 순찰경로(nodes)를 기반으로 개별 로봇의 순찰 경로 할당 %%%%%%%%%%%%%%%%%%%%%%%
for jj = 1:size(coordix,2)
        %% 0807변경
         [shortestPath,shortestPathLength] = onewaypath(binaryImage,coordix{jj}(:),coordiy{jj}(:),dmat,idxx{jj}(:),L_range,Rangeconstant); 

%                  if(shortestPathLength > PathLength1(jj)*2)
%                      continue;
%                  end
         coordiy{jj} = coordiy{jj}(shortestPath);
         coordix{jj} = coordix{jj}(shortestPath);
         idxx{jj} = idxx{jj}(shortestPath);
         PathLength1(jj) = shortestPathLength;    
end

 
fprintf('Path Assignment completed\n');

    
 %%%%%%%%%%%%%%%%%%%%%%% 개별 나눠진 순찰경로(nodes)를 기반으로 개별 로봇의 순찰 경로 할당 %%%%%%%%%%%%%%%%%%%%%%%
   
%% Text로 로봇 경로 추출
%        SaveRobotPose(coordix,coordiy);

%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%실제 로봇 순찰 경로 시간에 따라 보여주기%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%                            
 %% 
%  figure(2)
 hold on;
   for j=1:1000000
                count = 1;
                if(POI == 1)
                    if(j < size(POI_y,2)+1)
                        plot(POI_y(j),POI_x(j),'Marker','o','color',[1 0 1],'LineWidth',4);   
                    end
                end
               for k = 1:size(coordiy,2)
               if(j < size(coordiy{k},2)+1 && k == 1)
                  plot(coordiy{k}(j),coordix{k}(j),'Marker','o','color',[1 0 0],'LineWidth',4);   
                  count = 2;
               end
%                if(j >   size(coordiy{1},2)+1)
%                      break;
%                end 
                if(j < size(coordiy{k},2)+1 && k == 2)
                        plot(coordiy{k}(j),coordix{k}(j),'Marker','o','color',[0 1 0],'LineWidth',4);   
                        count = 2;
                end
                 if(j < size(coordiy{k},2)+1 && k == 3)
                   plot(coordiy{k}(j),coordix{k}(j),'Marker','o','color',[0 0 1],'LineWidth',4);    
                   count = 2;
                 end
                if(j < size(coordiy{k},2)+1 && k == 4)
                    plot(coordiy{k}(j),coordix{k}(j),'Marker','o','color',[0 1 1],'LineWidth',4);    
                    count = 2;
                 end
                if(j < size(coordiy{k},2)+1 && k ==5)
                    plot(coordiy{k}(j),coordix{k}(j),'Marker','o','color',[1 1 0],'LineWidth',4);    
                    count = 2;
                 end
                if(j < size(coordiy{k},2)+1 && k == 6)
                    plot(coordiy{k}(j),coordix{k}(j),'Marker','o','color',[1 0 1],'LineWidth',4);    
                    count = 2;
                end
               end 
                if(count == 1)
                    break;
                end
               pause(0.3);
   end
           
    for k = 1:size(coordiy,2)
               if(k == 1)
                  plot(coordiy{k}(:),coordix{k}(:),'Marker','o','color',[1 0 0],'LineWidth',4);   
                  count = 2;
               end
%                if(j >   size(coordiy{1},2)+1)
%                      break;
%                end 
                if(k == 2)
                        plot(coordiy{k}(:),coordix{k}(:),'Marker','o','color',[0 1 0],'LineWidth',4);   
                        count = 2;
                end
                 if(k == 3)
                  plot(coordiy{k}(:),coordix{k}(:),'Marker','o','color',[0 0 1],'LineWidth',4);    
                   count = 2;
                 end
                if(k == 4)
                    plot(coordiy{k}(:),coordix{k}(:),'Marker','o','color',[0 1 1],'LineWidth',4);    
                    count = 2;
                 end
                if(j < size(coordiy{k},2)+1 && k ==5)
                    plot(coordiy{k}(j),coordix{k}(j),'Marker','o','color',[1 1 0],'LineWidth',4);    
                    count = 2;
                 end
                if(j < size(coordiy{k},2)+1 && k == 6)
                    plot(coordiy{k}(j),coordix{k}(j),'Marker','o','color',[1 0 1],'LineWidth',4);    
                    count = 2;
                end
    end
           
fprintf('Coverage Generation End\n');

        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 정현 파트
%% 정현 파트
% pause;
fprintf('kw Start\n');
save('kumohResults','coordix','coordiy');
pause;
%% 광운대학교 파트
[coordix, coordiy, coorditheta, ~] = Func_fovPathPlanning('map_outline.jpg', coordix, coordiy); % 1:출력, 2:생략
fprintf('kw End\n');
%% 재저장 %% distance map (dmat), unique nodes (unique_all_vertex) and paths (coordix,coordiy).