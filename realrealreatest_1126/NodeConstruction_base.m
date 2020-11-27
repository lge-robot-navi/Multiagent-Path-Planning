function unique_all_vertex = NodeConstruction(binaryImage, gx,gy,L_range,RGB,RGB2,POI,min_size)
first_node_group = [];
second_node_group = [];
thrid_node_group = [];
fourth_node_group = [];
except_first_node_group = [];

min_radius_robot = L_range;
L_minradius = min_size;
Debug=1;

%% POI 사용할 경우
% for i=2:gx-1
%     for j=2:gy-1        
%         if(J(j,i) < RGB)
%             first_node_group = [first_node_group; j i];
%         end        
%     end
% end
% hold on;
% plot(first_node_group(:,2),first_node_group(:,1),'xr');
% pause;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% First Node Generation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=2:gx-1
    for j=2:gy-1        
        if(binaryImage(j-1,i)<10 && binaryImage(j,i) > RGB)
            first_node_group = [first_node_group; j i];
        elseif(binaryImage(j,i-1)<10 && binaryImage(j,i) > RGB) 
             first_node_group = [first_node_group; j i];
        elseif(binaryImage(j+1,i)<10 && binaryImage(j,i) > RGB)
             first_node_group = [first_node_group; j i];
        elseif(binaryImage(j,i+1)<10 && binaryImage(j,i) > RGB)
              first_node_group = [first_node_group; j i];
        end        
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Plot First Nodes %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hold on;
plot(first_node_group(:,2),first_node_group(:,1),'xr');

% pause;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Next Node Generation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% According to the normal vector direction

until_second_node_group = first_node_group;

outlines =  first_node_group;
first_node_group_size = size(first_node_group,1);
itr = 0;

while(itr<50)
    itr = itr+1;
    Nodenum=size(first_node_group,1);
%       L_range = 35;
        L_size = L_range;
    if(itr==1)
        L_range1 = L_range;
    else
        L_range1 = L_range;
    end
        
   for i=1:Nodenum    
%         if(first_node_group(i,1)> L_range && first_node_group(i,1) < gy-L_range&& first_node_group(i,2) > L_range&& first_node_group(i,2) <gx-L_range && first_node_group(i,1) < gy+L_range && first_node_group(i,2) < L_range+gx)
            if(numel(first_node_group(:,1)) < 3)
                break;
            end
            x = first_node_group(i,1);
            y = first_node_group(i,2);
            [ndist,nidx] = sort(sqrt((x -  first_node_group(:,1)).^2+(y -  first_node_group(:,2)).^2));             
            reidx = nidx(nidx~=i);
            angle = atan2(y-first_node_group(reidx(1),2),(x-first_node_group(reidx(1),1))) + atan2(y-first_node_group(reidx(2),2),(x-first_node_group(reidx(2),1)));
            angle = angle/2.0;
           
            for j = 1:4
               
                  if(itr==1)           
                        x1 = floor(x+L_minradius*cos(angle+j*pi/2));
                         y1 = floor(y+L_minradius*sin(angle+j*pi/2));
                  else
                       x1 = floor(x+L_range1*cos(angle+j*pi/2));
                     y1 = floor(y+L_range1*sin(angle+j*pi/2));
                  end
%                 pause(0.01);
%                 if(itr>1)
%                     pause;
%                 end 
                if(x1 > 0 && y1 > 0 && x1 <  size(binaryImage,1) &&  y1 <  size(binaryImage,2)) %% 맵 사이즈 밖인지
                    if(itr==1)
                         x1 = floor(x+L_minradius*cos(angle+j*pi/2));
                         y1 = floor(y+L_minradius*sin(angle+j*pi/2));
                         x1first = x1;
                         y1first = y1;
                         count = 2;
                         if(binaryImage(x1,y1) > RGB)
                             while(binaryImage(x1,y1) > RGB)
                                 x1last = x1;
                                 y1last = y1;
                                 x1 = floor(x+count*L_minradius*cos(angle+j*pi/2));
                                 y1 = floor(y+count*L_minradius*sin(angle+j*pi/2));
                                 if(~(x1 > 0 && y1 > 0 && x1 <  size(binaryImage,1) &&  y1 <  size(binaryImage,2)))
                                     break;
                                 end
                                 count = count+1;
                             end                             
                             x1= floor(x1first+x1last)/2;
                             y1= floor(y1first+y1last)/2;                             
                             if(min(sqrt((outlines(:,1)-(x1)).^2+(outlines(:,2)-y1).^2)) >= min_radius_robot)
                                if(numel(except_first_node_group)==0)
                                     second_node_group = [second_node_group; x1 y1];
                                     until_second_node_group = [until_second_node_group;x1 y1];
                                     except_first_node_group = [except_first_node_group;x1 y1];  
                                      
                                      plot(y1,x1,'og');
                                      hold on;
%                                       plot(y1first,x1first,'or');
%                                       plot(y1last,x1last,'ob');
%                                       pause;
                                elseif(min(sqrt((except_first_node_group(:,1)-(x1)).^2+(except_first_node_group(:,2)-y1).^2)) >= L_range)
                                     second_node_group = [second_node_group; x1 y1];
                                     until_second_node_group = [until_second_node_group;x1 y1];
                                     except_first_node_group = [except_first_node_group;x1 y1];
                                      plot(y1,x1,'og');
                                     hold on;
%                                       plot(y1first,x1first,'or');
%                                       plot(y1last,x1last,'ob');
%                                      pause;
        %                          continue;
                                end
                             end
                         end
                    elseif(binaryImage(x1,y1) > RGB2) %% 해당점이 unknown인지 free인지
                        if(min(sqrt((outlines(:,1)-(x1)).^2+(outlines(:,2)-y1).^2)) >= L_size) %% Lsize 밖에 있는 점인지
                            if(numel(except_first_node_group)==0)
                             second_node_group = [second_node_group; x1 y1];
                             until_second_node_group = [until_second_node_group;x1 y1];
                             except_first_node_group = [except_first_node_group;x1 y1];  
                              plot(y1,x1,'og');
                              hold on;
                            elseif(min(sqrt((except_first_node_group(:,1)-(x1)).^2+(except_first_node_group(:,2)-y1).^2)) >= L_range)
                             second_node_group = [second_node_group; x1 y1];
                             until_second_node_group = [until_second_node_group;x1 y1];
                             except_first_node_group = [except_first_node_group;x1 y1];
                              plot(y1,x1,'og');
                             hold on;
    %                          continue;
                            end
                        end
                    
                    end
                end
            end
   end
    
    first_node_group = second_node_group;
    hold on;
    if(isempty(second_node_group)==1)
        break;
    elseif(itr <= 10)
%         plot(second_node_group(:,2),second_node_group(:,1),'*','color',[.1*itr 0 0]);
%         viscircles([second_node_group(:,2),second_node_group(:,1)], L_range*ones(size(second_node_group,1),1),'color',[.1*itr 0 0],'LineStyle','--');
    elseif(itr <=20)        
%         plot(second_node_group(:,2),second_node_group(:,1),'*','color',[0 0 .05*itr]);     
%         viscircles([second_node_group(:,2),second_node_group(:,1)], L_range*ones(size(second_node_group,1),1),'color',[0 0 .05*itr],'LineStyle','--');
    else
%         plot(second_node_group(:,2),second_node_group(:,1),'*','color',[0 0.025*itr 0]);
%         viscircles([second_node_group(:,2),second_node_group(:,1)], L_range*ones(size(second_node_group,1),1),'color',[0 0.025*itr 0],'LineStyle','--');
    end  
    
    fprintf('%d차 노드생성\n',itr);
    second_node_group=[];
%     if(Debug==1)
%       pause;
%     end
end


if(POI == 1)
    gx0 = size(binaryImage0,2);
    gy0 = size(binaryImage0,1);

        for i=2:gx0-1
        for j=2:gy0-1        
            if(binaryImage0(j-1,i)<10 && binaryImage0(j,i) > RGB)
                first_node_group0 = [first_node_group0; j i];
            elseif(binaryImage0(j,i-1)<10 && binaryImage0(j,i) > RGB) 
                 first_node_group0 = [first_node_group0; j i];
            elseif(binaryImage0(j+1,i)<10 && binaryImage0(j,i) > RGB)
                 first_node_group0 = [first_node_group0; j i];
            elseif(binaryImage0(j,i+1)<10 && binaryImage0(j,i) > RGB)
                  first_node_group0 = [first_node_group0; j i];
            end        
        end
    end
    % pause;
    hold on;
    plot(first_node_group0(:,2),first_node_group0(:,1),'xr');
    
    until_second_node_group0 = first_node_group0;
    except_first_node_group0 = [];
    outlines0 =  first_node_group0;
    first_node_group_size0 = size(first_node_group0,1);
    itr = 0;
    RGB2 = 100;
    while(itr<30)
        itr = itr+1;
        Nodenum=size(first_node_group0,1);    
            L_size = L_range/2;
        if(itr==1)
            L_range1 = L_size+2;
        else
            L_range1 = L_range;
        end

       for i=1:Nodenum        
                if(numel(first_node_group0(:,1)) < 3)
                    break;
                end
                x = first_node_group0(i,1);
                y = first_node_group0(i,2);
                [ndist,nidx] = sort(sqrt((x -  first_node_group0(:,1)).^2+(y -  first_node_group0(:,2)).^2)); 
    %             pause;

                reidx = nidx(nidx~=i);
                angle = atan2(y-first_node_group0(reidx(1),2),(x-first_node_group0(reidx(1),1))) + atan2(y-first_node_group0(reidx(2),2),(x-first_node_group0(reidx(2),1)));
                angle = angle/2.0;

                for j = 1:4
                    x1 = floor(x+L_range1*cos(angle+j*pi/2));
                    y1 = floor(y+L_range1*sin(angle+j*pi/2));

    %                 pause(0.01);
    %                 if(itr>1)
    %                     pause;
    %                 end 
                    if(x1 > 0 && y1 > 0 && x1 <  size(binaryImage0,1) &&  y1 <  size(binaryImage0,2))
                        if(binaryImage0(x1,y1) > RGB2)
                            if(min(sqrt((outlines0(:,1)-(x1)).^2+(outlines0(:,2)-y1).^2)) >= L_size)
                                if(numel(except_first_node_group0)==0)
                                 second_node_group0 = [second_node_group0; x1 y1];
                                 until_second_node_group0 = [until_second_node_group0;x1 y1];
                                 except_first_node_group0 = [except_first_node_group0;x1 y1];  
%                                   plot(y1,x1,'og');
%                                   hold on;
                                elseif(min(sqrt((except_first_node_group0(:,1)-(x1)).^2+(except_first_node_group0(:,2)-y1).^2)) >= L_range)
                                 second_node_group0 = [second_node_group0; x1 y1];
                                 until_second_node_group0 = [until_second_node_group0;x1 y1];
                                 except_first_node_group0 = [except_first_node_group0;x1 y1];
%                                   plot(y1,x1,'og');
%                                  hold on;
        %                          continue;
                                end
                            end
                        end
                    end
                end
       end

        first_node_group0 = second_node_group0;
        hold on;
        if(isempty(second_node_group0)==1)
            break;
        elseif(itr <= 10)
            plot(second_node_group0(:,2),second_node_group0(:,1),'*','color',[.1*itr 0 0]);
    %         viscircles([second_node_group(:,2),second_node_group(:,1)], L_range*ones(size(second_node_group,1),1),'color',[.1*itr 0 0],'LineStyle','--');
        elseif(itr <=20)        
            plot(second_node_group0(:,2),second_node_group0(:,1),'*','color',[0 0 .05*itr]);     
    %         viscircles([second_node_group(:,2),second_node_group(:,1)], L_range*ones(size(second_node_group,1),1),'color',[0 0 .05*itr],'LineStyle','--');
        else
            plot(second_node_group0(:,2),second_node_group0(:,1),'*','color',[0 0.025*itr 0]);
    %         viscircles([second_node_group(:,2),second_node_group(:,1)], L_range*ones(size(second_node_group,1),1),'color',[0 0.025*itr 0],'LineStyle','--');
        end

       if(Debug_Plot==1)
    %     pause;
       end
        second_node_group0=[];
    %      pause;
    end
end



% pause;
unique_all_vertex = until_second_node_group(first_node_group_size+1:end,1:2);%except_first_node_group; %until_second_node_group(first_node_group_size+1:end,1:2);
% viscircles([unique_all_vertex(:,2),unique_all_vertex(:,1)],L_range*ones(size(unique_all_vertex,1),1),'Color','b');


%   hold on;
%            
%            fileID = fopen('poinode.txt','w');
% 
%             for j=1:size(unique_all_vertex,1)               
%                  fprintf(fileID,'{"x": %f, "y": %f, "angle": %f},',unique_all_vertex(j,2),unique_all_vertex(j,1),0);
%                     fprintf(fileID,'\n');                
%             end
%             fclose(fileID);

%    for i=1:first_node_group_size
% %         if(outlines(i,1)> L_range && first_node_group(i,1) < gy-L_range&& first_node_group(i,2) > L_range&& first_node_group(i,2) <gx-L_range )
%             x = floor((outlines(i,1)+outlines(:,1))/2);
%             y = floor((outlines(i,2)+outlines(:,2))/2);
% %             xx = repmat(x,size(unique_all_vertex,1),1);
% %             yy = repmat(y,size(unique_all_vertex,1),1);
%         for j = 1:first_node_group_size
% %             xxx = kron(unique_all_vertex(:,1),ones(first_node_group_size,1));
% %             yyy = kron(unique_all_vertex(:,2),ones(first_node_group_size,1));
%              if(binaryImage(x(j),y(j))>RGB2)
%                     if(min(sqrt((until_second_node_group(:,1)-x(j)).^2+(until_second_node_group(:,2)-y(j)).^2)) >= L_range2)                        
%                           unique_all_vertex = [unique_all_vertex;x(j) y(j)];       
%                           second_node_group = [second_node_group; x(j) y(j)];
%                          continue;
%                     end
%              end
%         end
%         if(numel(second_node_group)~=0)
%             plot(second_node_group(:,2),second_node_group(:,1),'x','color',[0 1 0]);
%         end
%            second_node_group=[];
%    end
