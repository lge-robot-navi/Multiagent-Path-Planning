%rescheduling
%lowBatRobots의 robotid:number를 int형의 변수로 저장하고 나머지 robots에 대한 내용은 cell 형태로 가면 좋을 것 같네요. robots{1}(1) = posx, robots{1}(2) = posy, robots{1}(3) = curTargetx,
function Rescheduling(excluded_robot_id, robots)

load('FinalResults','unique_all_vertex','dmat','coordix','coordiy','idx');
load('Parameter','binaryImage','gx','gy','Rangeconstant','max_value','L_range','RGB','RGB2','POI','unique_all_vertex_base');

%% 1=simul, 0=real
check_debug = 1; 

%% 총 로봇 대수
%% for simul
if(check_debug)
    num_of_robots = 4;%size(robots,1); %% for simul
    excluded_robot_id = 4; 
else
    num_of_robots = size(robots,1); %% for simul
end
robotlist = [];
D_RobotNum = size(coordix,2);
start_idx = zeros(D_RobotNum,2);

idxx = idx;

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



%% 이전 waypoint, 현재 waypoint index 찾기

for i = 1:D_RobotNum    
    if(i == excluded_robot_id)
        continue;
    end    
    if(isempty(coordix{i}))
        continue;
    end
     %% for simul
    if(check_debug)
        num_of_robots = 4;%size(robots,1); %% for simul
         start_idx(i,1) = idxx{i}(1); %% for simul
         start_idx(i,2) = idxx{i}(2);  %% for simul
    else
       %% for real
          [tndist,tnidx] = sort(sqrt((wx_to_xi(robots{i}.curTargetx) -  unique_all_vertex(:,1)).^2+(wy_to_yi(robots{i}.curTargety) -  unique_all_vertex(:,2)).^2)); 
          [tndist1,tnidx1] =  sort(sqrt((wx_to_xi(robots{i}.prevTargetx) -  unique_all_vertex(:,1)).^2+(wy_to_yi(robots{i}.prevTargety) -  unique_all_vertex(:,2)).^2)); 
          start_idx(i,1) = idxx{i}(tnidx1); 
          start_idx(i,2) = idxx{i}(tnidx);  
    end
%     [tndist,tnidx] = sort(sqrt((wx_to_xi(robots{i}.curTargetx) -  unique_all_vertex(:,1)).^2+(wy_to_yi(robots{i}.curTargety) -  unique_all_vertex(:,2)).^2)); 
%     [tndist1,tnidx1] =  sort(sqrt((wx_to_xi(robots{i}.prevTargetx) -  unique_all_vertex(:,1)).^2+(wy_to_yi(robots{i}.prevTargety) -  unique_all_vertex(:,2)).^2)); 
%     start_idx(i,1) = idxx{i}(tnidx1); 
%     start_idx(i,2) = idxx{i}(tnidx);  
   

end

% figure(1)
% hold on;
target_robotnum = excluded_robot_id;  %% 임의 지정
% plot(coordiy{target_robotnum}(:),coordix{target_robotnum}(:),'Marker','o','color',[0 0 0],'LineWidth',5);   

s = [];
t = [];
%% 거리 기반 Relational Graph 생성
for i=1:size(coordiy,2)
     for j=i:size(coordiy,2)
         if(i~=j && ~isempty(coordiy{j}) && ~isempty(coordiy{i}))
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
area_allocation = zeros(size(coordiy,2),size(coordiy,2));
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
                 target_robotnum1 = TR1(end-1);
    %              if(drawfirst==0)
    %                  drawfirst = 1;
    %                   plot(coordiy{target_robotnum1}(:),coordix{target_robotnum1}(:),'Marker','o','color',[0 0 0],'LineWidth',5);               
    %                   pause;
    %              end                
                 [coordiy,coordix,idxx]=region_assign(coordiy,coordix,idxx,i,target_robotnum1,floor(assing_region_num));                
             end
          end
      end
   end
end

coordiy{target_robotnum} = [];
coordix{target_robotnum} = [];
idxx{target_robotnum} = [];
fprintf('Final Result Check\n');

% for jj = 1:size(coordix,2)
        %% 0807변경        
        for i = 1:D_RobotNum
             if(isempty(coordiy{i}))
                 continue;
             end
             [shortestPath,shortestPathLength] = re_onewaypath(binaryImage,coordix{i}(:),coordiy{i}(:),dmat,idxx{i}(:),L_range,Rangeconstant,start_idx(i,:)); 
             coordiy{i} = coordiy{i}(shortestPath);
             coordix{i} = coordix{i}(shortestPath);
%              idxx{jj} = idxx{jj}(shortestPath);
             PathLength1(i) = shortestPathLength;    
        end
save('kumohResults','coordix','coordiy','unique_all_vertex_base');

%% 광운대학교 파트
[coordix, coordiy, coorditheta, ~] = Func_fovPathPlanning('map_outline.jpg', coordix, coordiy); % 1:출력, 2:생략

%% 재저장 %% distance map (dmat), unique nodes (unique_all_vertex) and paths (coordix,coordiy).

overlapped_all_vertex = [];
unique_all_vertex = [];
coordiy= coordiy(~any(cellfun('isempty', coordiy), 1));
coordix= coordix(~any(cellfun('isempty', coordix), 1));
coordiy(:,cellfun(@any,cellfun(@isnan,coordiy(1,:),'uni',false)))=[];
coordix(:,cellfun(@any,cellfun(@isnan,coordix(1,:),'uni',false)))=[];
idx{D_RobotNum+1} = [];

node1 = 0;
for i = 1:size(coordix,2)
    if(isempty(coordix{i}))
        continue;
        idx{i} = [];
    end
    overlapped_all_vertex = [coordix{i}(:) coordiy{i}(:)];
    unique_overlapped_all_vertex = unique(overlapped_all_vertex,'rows','stable');
    unique_all_vertex = [unique_all_vertex;unique_overlapped_all_vertex];   
    coordix{i} = unique_overlapped_all_vertex(:,1)';
    coordiy{i} = unique_overlapped_all_vertex(:,2)';
    node_size = size(unique_overlapped_all_vertex,1);    
    idx{i} = (node1+1):node1+node_size; 
    node1 = node_size+node1;    
end
fprintf('Node ReConstruction completed\n');
num_of_nodes = size(unique_all_vertex,1);
unique_all_vertex = [unique_all_vertex;unique_all_vertex_base];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%거리맵dmat 생성%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%Node간 연결 여부 및 거리 계산 (미연결일시 무한대로 둠)  
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
dmat(L_range*3 < dmat) = 100000;
fprintf('Recalculation & Coverage Generation End\n');
save('FinalResults','unique_all_vertex','dmat','coordix','coordiy','idx');
