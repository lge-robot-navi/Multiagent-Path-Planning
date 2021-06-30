clear all;
load('FinalResults','unique_all_vertex','dmat','coordix','coordiy','idx');
excluded_robot_id=3;
L_range = 172 ;
for i=1:4
    coordix{i}=coordix{i}(:)';
    coordiy{i}=coordiy{i}(:)';
end
idxx = idx;
figure(1)
hold on;
%-66.4, -82.775 좌표계 변환결과
%0.05
%x'=0.05*x-66.4, y'=0.05*y-82.775
%x = (x'+66.4)/0.05, y = (x'+82.775)/0.05
%mat data 불러옴.
%load data.mat
%dmat
% 최종경로: coordix{id}, coordiy{id}
% 전체 노드 배열 생성
% Total_node = [Coordix{1}(1), coordiy{1}(1);Coordix{1}(2), coordiy{1}(2);…; coordix{5}(end), coordiy{5}(end)];
% Unique_Total_node = unique(Total_node','rows')’;
% Unique_Total_node;
% unique_all_vertex
% coordix, coordiy, node_size,dmat, 
D_RobotNum = size(coordix,2);
start_idx = zeros(D_RobotNum,2);
% %% 이전 waypoint, 현재 waypoint index 찾기
% count = 1;
% for i = 1:D_RobotNum
%     if(i == excluded_robot_id)
%         continue;
%     end
% %   [tndist,tnidx] = sort(sqrt((robots{i}(3) -  Unique_Total_node(count:count+node_size(i),1)-1).^2+(robots{i}(4) -  Unique_Total_node(count:count+node_size(i),2)-1).^2)); 
% %   [tndist1,tnidx1] =  sort(sqrt((robots{i}(6) -  Unique_Total_node(count:count+node_size(i),1)-1).^2+(robots{i}(7) -  Unique_Total_node(count:count+node_size(i),2)-1).^2)); 
% [tndist,tnidx] = sort(sqrt((wx_to_xi(robots{i}.curTargetx) -  unique_all_vertex(:,1)).^2+(wy_to_yi(robots{i}.curTargety) -  unique_all_vertex(:,2)).^2)); 
% [tndist1,tnidx1] =  sort(sqrt((wx_to_xi(robots{i}.prevTargetx) -  unique_all_vertex(:,1)).^2+(wy_to_yi(robots{i}.prevTargety) -  unique_all_vertex(:,2)).^2)); 
% 
% %     count = count+node_size(i);
%     start_idx(i,1) = tnidx;
%     start_idx(i,2) = tnidx1;
% end


target_robotnum = excluded_robot_id;  %% 임의 지정
plot(coordiy{target_robotnum}(:),coordix{target_robotnum}(:),'Marker','o','color',[0 0 0],'LineWidth',5);   
% if(Debug==1)
    pause;
% end
                  

% figure(2)
%  plot(coordiy{target_robotnum}(:),coordix{target_robotnum}(:),'Marker','o','MarkerSize',7,'color',[0 0 0],'LineStyle','none'); 
% s = [1 1 1 1 1];
% t = [2 3 4 5 6];
% weights = [5 5 5 6 9];
% G = graph(s,t,weights,10)
s = [];
t = [];
%% 거리 기반 Relational Graph 생성
for i=1:size(coordiy,2)
     for j=i:size(coordiy,2)
         if(i~=j)
            aa = repmat(coordiy{i}(:),size(coordiy{j},2),1)- kron(coordiy{j}(:),ones(size(coordiy{i},2),1));
            bb = repmat(coordix{i}(:),size(coordix{j},2),1)- kron(coordix{j}(:),ones(size(coordix{i},2),1));
            cc = sqrt(aa.^2+bb.^2);
            if(min(cc) <= L_range*2) %% 거리 기반 
                s = [s i];
                t = [t j];
            end
         end
     end
end
%% Robot Relational Graph
G=graph(s,t); 
TR = shortestpathtree(G,target_robotnum,'all');
% plot(TR);
area_allocation = zeros(size(coordiy,2),D_RobotNum);
TR1 = [];

for i=1:size(coordix,2)
    if(target_robotnum~=i)
        TR1 = shortestpath(G,target_robotnum,i);
        count = 2;
        while(count <= size(TR1,2))                        
            area_allocation(TR1(count),count-1) = area_allocation(TR1(count),count-1)+1;
            count = count+1;
        end
    end
end

%1st propagation
%% 하나씩 공평하게 나눔.

% connected_robot = target_robotnum;

fprintf('Robot Relational Graph Generation\n');
pause;


for t_count = 1:D_RobotNum
%     if(t_count == target_robotnum)
%         continue;
%     end
    area_division_num = sum(area_allocation(:,t_count));
    area_idx = (1:area_division_num);
    if(area_division_num==0)
        break;
    end    
    
    TR1 = shortestpath(G,target_robotnum,t_count);        
    coorditesty = coordiy{target_robotnum}(:);
    coorditestx = coordix{target_robotnum}(:);
    idx_test = idx{target_robotnum}(:);
    [idx2Region,Center] = spectralcluster([coorditesty,coorditestx],area_division_num,'Distance','mahalanobis'); 
    assing_region_num = size(coorditesty,1)/(D_RobotNum-1);

    %% t_count = 1;
   if(t_count == 1)
       for i=1:D_RobotNum
           if(i==target_robotnum)
               continue;
           end 
             if(area_allocation(i,1)~=0)
               for k=1:area_allocation(i,1)
                    minval = 100000;
                    for j=1:size(Center,1)
                         if(isempty(area_idx(area_idx==j))==0)
                            curmin = min(sqrt((coordiy{i}(:)-Center(j,1)).^2+(coordix{i}(:)-Center(j,2)).^2));
                            if(minval > curmin)
                                minval = curmin;
                                minidx = j;
                            end
                         end
                    end           
                    area_idx = area_idx(area_idx ~= minidx);
                    coordiy{i} = [coordiy{i} coorditesty(idx2Region==minidx)'];
                    coordix{i} = [coordix{i} coorditestx(idx2Region==minidx)'];      
                     idxx{i} = [idxx{i} idx_test(idx2Region==minidx)'];      
                    
                    if(i == 1)
                      plot(coordiy{i}(:),coordix{i}(:),'Marker','o','LineStyle','none','color',[1 0 0],'LineWidth',4);   
                      count = 2;

                    elseif(i == 2)
                            plot(coordiy{i}(:),coordix{i}(:),'Marker','o','LineStyle','none','color',[0 1 0],'LineWidth',4);   
                            count = 2;

                     elseif(i == 3)
                       plot(coordiy{i}(:),coordix{i}(:),'Marker','o','LineStyle','none','color',[0 0 1],'LineWidth',4);    
                       count = 2;

                    elseif(i == 4)
                        plot(coordiy{i}(:),coordix{i}(:),'Marker','o','LineStyle','none','color',[0 1 1],'LineWidth',4);    
                        count = 2;

                    elseif(i==5)
                        plot(coordiy{i}(:),coordix{i}(:),'Marker','o','LineStyle','none','color',[1 1 0],'LineWidth',4);    
                        count = 2;

                    elseif(i== 6)
                        plot(coordiy{i}(:),coordix{i}(:),'Marker','o','LineStyle','none','color',[1 0 1],'LineWidth',4);    
                        count = 2;
                    end
                    fprintf('1st Relational Robot''s Node Assign \n');
%                     pause;
               end
             end
       end
   else
       drawfirst = 0;
      for i=1:D_RobotNum   
          if(area_allocation(i,t_count)~=0)
             for k=1:area_allocation(i,t_count)
                 TR1 = shortestpath(G,target_robotnum,i);
                 target_robotnum1 = TR1(end-1)
    %              if(drawfirst==0)
    %                  drawfirst = 1;
    %                   plot(coordiy{target_robotnum1}(:),coordix{target_robotnum1}(:),'Marker','o','color',[0 0 0],'LineWidth',5);               
    %                   pause;
    %              end
                assing_region_num
                coordiy
                 [coordiy,coordix,idxx]=region_assign(coordiy,coordix,idxx,i,target_robotnum1,floor(assing_region_num));
                 coordiy
                 pause;

                   if(i == 1)
                      plot(coordiy{i}(:),coordix{i}(:),'Marker','o','LineStyle','none','color',[1 0 0],'LineWidth',4);   
                      count = 2;

                    elseif(i == 2)
                            plot(coordiy{i}(:),coordix{i}(:),'Marker','o','LineStyle','none','color',[0 1 0],'LineWidth',4);   
                            count = 2;

                     elseif(i == 3)
                       plot(coordiy{i}(:),coordix{i}(:),'Marker','o','LineStyle','none','color',[0 0 1],'LineWidth',4);    
                       count = 2;

                    elseif(i == 4)
                        plot(coordiy{i}(:),coordix{i}(:),'Marker','o','LineStyle','none','color',[0 1 1],'LineWidth',4);    
                        count = 2;

                    elseif(i==5)
                        plot(coordiy{i}(:),coordix{i}(:),'Marker','o','LineStyle','none','color',[1 1 0],'LineWidth',4);    
                        count = 2;

                    elseif(i== 6)
                        plot(coordiy{i}(:),coordix{i}(:),'Marker','o','LineStyle','none','color',[1 0 1],'LineWidth',4);    
                        count = 2;
                    end
                    debug = 93
%                     pause;


             end
          end
      end
   end
end

fprintf('1st end\n');
pause;
for jj = 1:size(coordix,2)
        %% 0807변경
         [shortestPath,shortestPathLength] = re_onewaypath(binaryImage,coordix{jj}(:),coordiy{jj}(:),dmat,idxx{jj}(:),L_range,Rangeconstant,start_idx(jj,:)); 
                                            
%                  if(shortestPathLength > PathLength1(jj)*2)
%                      continue;
%                  end
         coordiy{jj} = coordiy{jj}(shortestPath);
         coordix{jj} = coordix{jj}(shortestPath);
%                  idxx{jj} = idxx{jj}(shortestPath);
         PathLength1(jj) = shortestPathLength;    
end

save('kumohResults','coordix','coordiy','unique_all_vertex_base');

%% 광운대학교 파트
[coordix, coordiy, coorditheta, ~] = Func_fovPathPlanning('map_outline.jpg', coordix, coordiy); % 1:출력, 2:생략

pause;
%% 재저장 %% distance map (dmat), unique nodes (unique_all_vertex) and paths (coordix,coordiy).
clc;
clear all;
load('kwResults');
L_range_base = 25;
RGB = 150;
RGB2 = 230;
POI=2;
num_of_robot=4;
    binaryImage = Image_load(POI,0);
    binaryImage(binaryImage<230)=0;
    binaryImage(binaryImage>=230)=255;
    se = strel('disk',10);
    binaryImage = imclose(binaryImage,se);
    binaryImage = imopen(binaryImage,se);
gx = size(binaryImage,2);
gy = size(binaryImage,1);

unique_all_vertex_base = NodeConstruction_base(binaryImage,gx,gy,L_range_base,RGB,RGB2,POI);
overlapped_all_vertex = [];
unique_all_vertex = [];
node1 = 0;
coordiy= coordiy(~any(cellfun('isempty', coordiy), 1));
coordix= coordix(~any(cellfun('isempty', coordix), 1));

coordiy(:,cellfun(@any,cellfun(@isnan,coordiy(1,:),'uni',false)))=[];
coordix(:,cellfun(@any,cellfun(@isnan,coordix(1,:),'uni',false)))=[];

idxx= idxx(~any(cellfun('isempty', idxx), 1));
idx{num_of_robot+1} = [];

for i = 1:size(coordix,2)
    overlapped_all_vertex = [coordix{i}(:) coordiy{i}(:)];
    unique_overlapped_all_vertex = unique(overlapped_all_vertex,'rows','stable');
    unique_all_vertex = [unique_all_vertex;unique_overlapped_all_vertex];   
    coordix{i} = unique_overlapped_all_vertex(:,1)';
    coordiy{i} = unique_overlapped_all_vertex(:,2)';
    node_size = size(unique_overlapped_all_vertex,1);    
    idx{i} = (node1+1):node1+node_size; 
     node1 = node_size+node1;    
end
idx= idx(~any(cellfun('isempty', idx), 1));

fprintf('Node ReConstruction completed\n');
num_of_nodes = size(unique_all_vertex,1);
unique_all_vertex = [unique_all_vertex;unique_all_vertex_base];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%거리맵dmat 생성%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%Node간 연결 여부 및 거리 계산 (미연결일시 무한대로 둠)  
Rangeconstant = 2.5;
max_value = 10000;%
dmat = distancefunction(unique_all_vertex, binaryImage,L_range_base,Rangeconstant,max_value);
At = dmat.';
m  = tril(true(size(At)));
dv  = At(m).';
dv = dv(dv~=0);
size(dmat)
fprintf('Recompute Distance completed\n');
dmat = recalc_distancefunction(num_of_nodes,dmat); %% 계산량 소요문제 존재
fprintf('recalc_distancefunction completed\n');
unique_all_vertex = unique_all_vertex(1:num_of_nodes,:);
dmat = dmat(1:num_of_nodes,1:num_of_nodes);
L_range = 172; %% 9번지도는 20임 %%13번지 도 15
dmat(L_range*3 < dmat) = 100000;
save('FinalResults','unique_all_vertex','dmat','coordix','coordiy','idx');
