% 여러 번 실행
% 실행 간격, 대기 충돌을 처리하는 방법
% 콜백 함수

% t = timer;
% % t.StartDelay = 3;
% t.TimerFcn = @(myTimerObj, thisEvent)disp('3 seconds have elapsed');
% start(t)




% function t = pingtest()
%     t = timer;
%     t.TimerFcn = @
%     t.Period = 5;
%     t.ExecutionMode = 'fixedSpacing';
% 
%     
%     
%     t = pingtest;
%     start(t);

t = timer('Period', 1, 'ExecutionMode', 'fixedSpacing');
t.TimerFcn = @ping_test;
start(t);
% delete(t);

function ping_test(obj, event)
	fprintf('Ping Test\n\n');
% 	cellArrayText{1} = sprintf('Ping Test');
%     app.TextArea.Value = cellArrayText;
            
    C = evalc('!ping -n 1 220.81.76.111');  
    loss = regexp(C, '(연결*)', 'tokens'); % loss = ''
	% isempty(loss) = 1
            
	if(isempty(loss) == 1)
        fprintf('대상 호스트에 연결할 수 있습니다.\n\n');
%         cellArrayText{2} = sprintf('대상 호스트에 연결할 수 있습니다.');
%         app.TextArea.Value = cellArrayText;
	elseif(isempty(loss) == 0)
        fprintf('대상 호스트에 연결할 수 없습니다.\n\n');
%         cellArrayText{2} = sprintf('대상 호스트에 연결할 수 없습니다.');
%         app.TextArea.Value = cellArrayText;
    end   
end