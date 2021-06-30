%realrealtest
%rescheduling
%lowBatRobots의 robotid:number를 int형의 변수로 저장하고 나머지 robots에 대한 내용은 cell 형태로 가면 좋을 것 같네요. robots{1}(1) = posx, robots{1}(2) = posy, robots{1}(3) = curTargetx,
function [coordix, coordiy, coorditheta] = Rescheduler(excluded_robot_id, robots)
 % excluded_robot_id
 % size(robots,1)
 % robots(1).prevTargetx
 % robots(1).robotId
 % robots(2).robotId
 % robots(3).robotId
%% 2-abnormal, 4-lowbat
load('FinalResults','unique_all_vertex','dmat','ccoordix','ccoordiy','idx');
load('Parameter','binaryImage','gx','gy','Rangeconstant','max_value','L_range','L_range_base','RGB','RGB2','POI','unique_all_vertex_base');

%% 1=simul, 0=real
check_debug = 1; 
coordix = ccoordix;
coordiy = ccoordiy;
idxx = idx;

%% Draw Figures begin
axis image 
title('Input Map','color','w');
axis off
set(gcf, 'InvertHardCopy', 'off');
hold off;
x0=0;
y0=0;
width=1200;
%%변수선언
height=1000;
set(gcf,'position',[x0,y0,width,height])
hold on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%파라미터 초기화%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
binaryImage = Image_load(2,0); %% image load.최신 지도 (아직 수정필요)
%  binaryImage = Image_load(POI,2);  %% image load.
hold on;
set(gca,'YDir','normal')  
fprintf('Image load completed\n');

  
%     binaryImage = Image_load(POI,0);
    binaryImage(binaryImage<230)=0;
    binaryImage(binaryImage>=230)=255;

%     se = offsetstrel('ball',5,3);
    se = strel('disk',10);
    binaryImage = imclose(binaryImage,se);
    binaryImage = imopen(binaryImage,se);
    
    
    imagesc(binaryImage ); %% represent the image
imshow(binaryImage ); 
hold on;
% pause;
    
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
                if(j < size(coordiy{k},2)+1 && k >=7)
                    plot(coordiy{k}(j),coordix{k}(j),'Marker','o','color',[1 1 1].*rand(1,3),'LineWidth',4);    
                    count = 2;
                end
               end 
                if(count == 1)
                    break;
                end
%                pause(0.3);
   end
% pause;
%%  Draw Figures end


%% 총 로봇 대수
%% for simul
if(check_debug)
    num_of_robots = 3;%size(robots,1); %% for simul
%     excluded_robot_id = 1; %% 2는 잘됨
 excluded_robot_id = 2; %% 2는 잘됨
else
    num_of_robots = size(robots,1); %% for simul
end
robotlist = [];

D_RobotNum = size(coordix,2);

start_idx = zeros(D_RobotNum,2);



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
 if(check_debug)
    for i = 1:D_RobotNum    
        if(i == excluded_robot_id)
            continue;
        end    
        if(isempty(coordix{i}))
            continue;
        end
         %% for simul
             start_idx(i,1) = idxx{i}(1); %% for simul
             start_idx(i,2) = idxx{i}(2);%% for simul
    end
 else
    for i = 1:num_of_robots
          [tndist,tnidx] = sort(sqrt((wx_to_xi(robots(i).curTargetx) -  unique_all_vertex(:,1)).^2+(wy_to_yi(robots(i).curTargety) -  unique_all_vertex(:,2)).^2)); 
          [tndist1,tnidx1] =  sort(sqrt((wx_to_xi(robots(i).prevTargetx) -  unique_all_vertex(:,1)).^2+(wy_to_yi(robots(i).prevTargety) -  unique_all_vertex(:,2)).^2)); 
          if(~isempty(find(idxx{robots(i).robotId}==tnidx)) && ~isempty(find(idxx{robots(i).robotId}==tnidx1)))
              start_idx(robots(i).robotId,1) = tnidx1(1);
              start_idx(robots(i).robotId,2) = tnidx(1);
          else
               start_idx(robots(i).robotId,1) = idxx{robots(i).robotId}(1);
               start_idx(robots(i).robotId,2) = idxx{robots(i).robotId}(2);
          end
    end
 end
%     [tndist,tnidx] = sort(sqrt((wx_to_xi(robots(i).curTargetx) -  unique_all_vertex(:,1)).^2+(wy_to_yi(robots(i).curTargety) -  unique_all_vertex(:,2)).^2));
%     [tndist1,tnidx1] =  sort(sqrt((wx_to_xi(robots(i).prevTargetx) -  unique_all_vertex(:,1)).^2+(wy_to_yi(robots(i).prevTargety) -  unique_all_vertex(:,2)).^2)); 
%     start_idx(i,1) = idxx{i}(tnidx1); 
%     start_idx(i,2) = idxx{i}(tnidx);  
   

% end

% figure(1)
hold on;
target_robotnum = excluded_robot_id;  %% 임의 지정
plot(coordiy{target_robotnum}(:),coordix{target_robotnum}(:),'Marker','o','color',[0 0 0],'LineWidth',5);   
pause;
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
% figure(2)
% plot(TR);
% pause;
% figure(1)
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

    coorditesty = coordiy{target_robotnum}(:);
    coorditestx = coordix{target_robotnum}(:);
    idx_test = idx{target_robotnum}(:);
    assing_region_num = size(coorditesty,1)/(D_RobotNum-1);
    whole_size = size(coorditesty,1);

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

% plot(TR1);
% pause;
%     [idx2Region,Center] = kmeans([coorditesty,coorditestx],area_division_num);% spectralcluster([coorditesty,coorditestx],area_division_num,'Distance','mahalanobis')
%    plot(coorditesty(:,1),coorditestx(:,1),'ro');
%    hold on;
%    plot(Center(:,1),Center(:,2),'ob');
%     Center
%     pause;
    
   %% for debug
% %    hold on;
% %       for j=1:1000000
% %                 count = 1;
% %                for k = 1:size(coordiy,2)
% %                if(j < size(coordiy{k},2)+1 && k == 1)
% %                   plot(coordiy{k}(j),coordix{k}(j),'Marker','o','color',[1 0 0],'LineWidth',4);   
% %                   count = 2;
% %                end
% % %                if(j >   size(coordiy{1},2)+1)
% % %                      break;
% % %                end 
% %                 if(j < size(coordiy{k},2)+1 && k == 2)
% %                         plot(coordiy{k}(j),coordix{k}(j),'Marker','o','color',[0 1 0],'LineWidth',4);   
% %                         count = 2;
% %                 end
% %                  if(j < size(coordiy{k},2)+1 && k == 3)
% %                    plot(coordiy{k}(j),coordix{k}(j),'Marker','o','color',[0 0 1],'LineWidth',4);    
% %                    count = 2;
% %                  end
% %                 if(j < size(coordiy{k},2)+1 && k == 4)
% %                     plot(coordiy{k}(j),coordix{k}(j),'Marker','o','color',[0 1 1],'LineWidth',4);    
% %                     count = 2;
% %                  end
% %                 if(j < size(coordiy{k},2)+1 && k ==5)
% %                     plot(coordiy{k}(j),coordix{k}(j),'Marker','o','color',[1 1 0],'LineWidth',4);    
% %                     count = 2;
% %                  end
% %                 if(j < size(coordiy{k},2)+1 && k == 6)
% %                     plot(coordiy{k}(j),coordix{k}(j),'Marker','o','color',[1 0 1],'LineWidth',4);    
% %                     count = 2;
% %                 end
% %                end 
% %                 if(count == 1)
% %                     break;
% %                 end
% %                pause(0.3);
% %       end
% %       
% %       for j=1:size(Center,1)
% %           hold on;
% %           plot(Center(j,1),Center(j,2),'Marker','o','color',[0 0 0],'LineWidth',4);    
% %       end
% %       fprintf('testend');
% %       pause;
% %    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% t_count = 1;
   if(t_count == 1)% 
        count_region = zeros(D_RobotNum,1);%        
        min_area_size = 100000;
        min_area_size_index = 0;
       for i = 1:D_RobotNum
            count_region(i) = area_allocation(i,t_count)*floor(assing_region_num);
            if(area_allocation(i,t_count)~=0)
                add_size =size(coordiy{i},2)+count_region(i);
                if(add_size < min_area_size)
                    min_area_size = add_size;
                    min_area_size_index = i;
                end
            end
       end  
    
       %% 고민필요
       if(~(size(coordiy{target_robotnum},2)-sum(count_region)==0))
           count_region(min_area_size_index) = count_region(min_area_size_index)+(size(coordiy{target_robotnum},2)-sum(count_region));
       end
         assignment_region = count_region;
      assignment_region = assignment_region.*0;
      original_count_region = count_region;
        whole_size
       count_region
       pause;
       
       
       while(1)
           count_region_check = 0;
           for i=1:D_RobotNum
               if(count_region(i)<=0)
                   count_region_check = count_region_check+1;
               else
%                     TR1 = shortestpath(G,target_robotnum,i);
%                  target_robotnum1 = TR1(end-1);
                  [coordiy,coordix,idxx,assign_ok]=region_assign(coordiy,coordix,idxx,i,target_robotnum,1);        
                  if(assign_ok)
                     count_region(i) = count_region(i)-1;
                     assignment_region(i) = assignment_region(i) + 1;
                  else
                      %% 할당될 노드가 없으면 할당할 수 있는 로봇에 자신이 가져가야할 노드까지 할당
                     
                      a = find(count_region<0);
                      b = find(original_count_region~=0);
                      b = b(b~=i);
                      c = find(~ismember(b,a));
                      d = b(c(1));
                      count_region(d) = count_region(d)+count_region(i);
                      count_region(i) = -1*count_region(i);
                    
                  end
                     Draw(i,coordiy,coordix);
                    
%                      pause;
               end
           end
           if(count_region_check==D_RobotNum)
              
                diff = assignment_region - original_count_region;
                pause;
               while(sum(abs(diff)) > 0)
                   for i = 1:D_RobotNum
                       if((original_count_region(i)-assignment_region(i))>0)      
                            diff = assignment_region - original_count_region;
                            [an,or] = sort(diff);
                            for j=0:D_RobotNum-1
                             
                                if(or(end-j)~=target_robotnum)
                                    [coordiy,coordix,idxx,assign_ok]=region_assign(coordiy,coordix,idxx,i,or(end-j),1); 
                                      if(assign_ok)
                                          assignment_region(i) = assignment_region(i)+1;
                                          assignment_region(or(end-j)) = assignment_region(or(end-j))-1;
                                          Draw(i,coordiy,coordix);
                                          break;
                                      end
                                end
                            end
                       end
                   end                   
                   diff = assignment_region - original_count_region;
               end
               assignment_region
               count_region
               pause;
               break;
           end
       end
%        if(~(size(coordiy{excluded_robot_id},2) == 0))
%            
%        end
%        coordiy
   fprintf('first if\n');
   else
       pause;
       drawfirst = 0;
       count_region = zeros(D_RobotNum,1);
       for i = 1:D_RobotNum
            count_region(i) = area_allocation(i,t_count)*floor(assing_region_num);
       end
       while(1)
           count_region_check = 0;
           for i=1:D_RobotNum
               if(count_region(i)==0)
                   count_region_check = count_region_check+1;
               else
                    TR1 = shortestpath(G,target_robotnum,i);
                 target_robotnum1 = TR1(end-1);
                 [coordiy,coordix,idxx]=region_assign(coordiy,coordix,idxx,i,target_robotnum1,1);                
                     count_region(i) = count_region(i)-1;
                     
                     Draw(i,coordiy,coordix);
                    
%                      pause;
               end
           end
           if(count_region_check==D_RobotNum)
               break;
           end
       end
%       for i=1:D_RobotNum   
%           if(area_allocation(i,t_count)~=0)
%              for k=1:area_allocation(i,t_count)
%                  TR1 = shortestpath(G,target_robotnum,i);
%                  target_robotnum1 = TR1(end-1);
%     %                            
%                  [coordiy,coordix,idxx]=region_assign(coordiy,coordix,idxx,i,target_robotnum1,floor(assing_region_num));                
%              end
%           end
%       end
   fprintf('second else\n');
   end
   
end

coordiy{target_robotnum} = [];
coordix{target_robotnum} = [];
idxx{target_robotnum} = [];
fprintf('Final Result Check\n');
pause;
% for jj = 1:size(coordix,2)
        %% 0807변경        
        for i = 1:D_RobotNum
             if(isempty(coordiy{i}))
                 continue;
             end
             
             [shortestPath,shortestPathLength] = re_onewaypath(binaryImage,coordix{i}(:),coordiy{i}(:),dmat,idxx{i}(:),L_range,Rangeconstant,start_idx(i,:));
             coordiy{i} = coordiy{i}(shortestPath);
             coordix{i} = coordix{i}(shortestPath);

%              figure(10)
%              
%              plot(coordiy{1}(:),coordix{1}(:),'Marker','o','LineStyle','none','color',[1 0 0],'LineWidth',4);   

%% for debug
%             if(i==2)
%                  imshow(binaryImage ); 
%                  for k = 1:size(coordiy{i},2)-1
%                      hold on;   
%                          plot(coordiy{i}(k:k+1),coordix{i}(k:k+1),'Marker','o','color',[1 0 0],'LineWidth',k/3);  
%                         pause(0.5);
%                  end
%             end
%              idxx{jj} = idxx{jj}(shortestPath);
             PathLength1(i) = shortestPathLength;
%              pause;
        end
% save('kumohResults','coordix','coordiy','unique_all_vertex_base');

%% 광운대학교 파트

[coordix, coordiy, coorditheta, ~] = Func_fovPathPlanning('map_outline.jpg', coordix, coordiy); % 1:출력, 2:생략
  posx=coordix;
   posy= coordiy ;
   posTheta= coorditheta;
    for i=1:size(coordix,2)
          for j = 1:size(coordix{i},2)
             posx{i}(1,j) = 0.05*(  coordiy{i}(1,j)-1328);
             posy{i}(1,j) = 0.05*( 1655.5 - coordix{i}(1,j));
             if(j ==size(coordix{i},2) )
                 posTheta{i}(1,j) = atan2(posy{i}(1,1)-posy{i}(1,j),posx{i}(1,1)-posx{i}(1,j));
             else
                 posTheta{i}(1,j) = atan2(0.05*(1655.5 - coordix{i}(1,j+1))-posy{i}(1,j),0.05*(coordiy{i}(1,j+1)-1328)-posx{i}(1,j));
             end
          end
    end
    
    coordix = posx;
    coordiy = posy;
    coorditheta = posTheta;
fprintf('kw End\n');



%% 재저장 %% distance map (dmat), unique nodes (unique_all_vertex) and paths (coordix,coordiy).
% overlapped_all_vertex = [];
% unique_all_vertex = [];
% coordiy(:,cellfun(@any,cellfun(@isnan,coordiy(1,:),'uni',false)))=[];
% coordix(:,cellfun(@any,cellfun(@isnan,coordix(1,:),'uni',false)))=[];
% idx{D_RobotNum+1} = [];
% 
% node1 = 0;
% for i = 1:size(coordix,2)
%     if(isempty(coordix{i}))
%         idx{i} = [];
%         continue;        
%     end
%     overlapped_all_vertex = [coordix{i}(:) coordiy{i}(:)];
%     unique_overlapped_all_vertex = unique(overlapped_all_vertex,'rows','stable');
%     unique_all_vertex = [unique_all_vertex;unique_overlapped_all_vertex];   
%     ccoordix{i} = unique_overlapped_all_vertex(:,1)';
%     ccoordiy{i} = unique_overlapped_all_vertex(:,2)';
%     node_size = size(unique_overlapped_all_vertex,1);    
%     idx{i} = (node1+1):node1+node_size; 
%     node1 = node_size+node1;    
% end
% fprintf('Node ReConstruction completed\n');
% num_of_nodes = size(unique_all_vertex,1);
% unique_all_vertex = [unique_all_vertex;unique_all_vertex_base];
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%거리맵dmat 생성%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% %%Node간 연결 여부 및 거리 계산 (미연결일시 무한대로 둠)  
% dmat = distancefunction(unique_all_vertex, binaryImage,L_range_base,Rangeconstant,max_value);
% At = dmat.';
% m  = tril(true(size(At)));
% dv  = At(m).';
% dv = dv(dv~=0);
% size(dmat)
% fprintf('Recompute Distance completed\n');
% dmat = recalc_distancefunction(num_of_nodes,dmat); %% 계산량 소요문제 존재
% fprintf('recalc_distancefunction completed\n');
% unique_all_vertex = unique_all_vertex(1:num_of_nodes,:);
% dmat = dmat(1:num_of_nodes,1:num_of_nodes);
% dmat(L_range*3 < dmat) = 100000;
% fprintf('Recalculation & Coverage Generation End\n');
% save('FinalReschedulingResults','unique_all_vertex','dmat','ccoordix','ccoordiy','idx');

