function Draw(k,coordiy,coordix)
   hold on;
      for j=1:1000000
                count = 1;
              
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
             
                if(count == 1)
                    break;
                end
%                pause(0.02);
      end
