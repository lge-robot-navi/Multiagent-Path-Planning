% function tf = haveInet()
%   tf = false;
%   try
%     % address = java.net.InetAddress.getByName('www.google.de')
%     address = java.net.InetAddress.getByName('220.81.76.110')
%     tf = true;
%   end
% end

C = evalc('!ping -n 1 220.81.76.111');   
loss = regexp(C, '(연결*)', 'tokens') % loss = ''

% C = evalc('!ping -n 1 220.81.76.110')   
% loss = regexp(C, '(연결*)', 'tokens') % loss = '연결'