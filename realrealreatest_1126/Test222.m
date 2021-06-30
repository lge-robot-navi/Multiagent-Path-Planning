clc;
clear all;

Test1 = [1 3 4];
Test2 = 5;
a = 'LOWBAT';

True_robotId_array = Test1 ;
  True_ragentId = Test2  ;
 count = 0;
       
 if (a == "INIT")
	for i=1:size(Test1,2)
	   Test1(i) = i;
	end
	Test2 = 0;
  else
	for i=1:size(Test1,2)
               if((Test2  > Test1(i)))                %^  [3] [1 4 5] [3] [1 4 5]
                    Test1(i) = i
               else
                   if(count == 0)
                       count = count+1;
                       Test2 = i %%스케쥴러일때 agentId는?
                   end
                   Test1(i) = i+1
               end
            end
             if(count == 0 &&  Test2~=0)
                Test2 = size(Test1 ,2)+1
            end
  end

            Test2
	Test1
True_robotId_array
 robotId = []; 
 count = 0;