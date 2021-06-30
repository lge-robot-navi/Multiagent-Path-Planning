% MATLAB example using TCP/IP (matlab_tcpip_example.m)
% This simple code example demonstrates how you can use MATLAB to exchange data 
% with a remote application not developed in MATLAB. This code example is taken
% from a MATLAB Digest technical article written by Edward J. Mayhew from 
% George Mason University.  While HTTP was used as the higher-level protocol in 
% this example, you can use other protocols, as was the case in the project. 
% MATLAB supports TCP/IP using Instrument Control Toolbox.  Requires MATLAB and 
% Instrument Control Toolbox. 
%
% Copyright 2006 - 2014 The MathWorks, Inc.
%
% On line 14, substitute "www.EXAMPLE_WEBSITE.com" with an actual website with
% which you wish to communicate.

% Create TCP/IP object 't'. Specify server machine and port number. 
t = tcpip('www.naver.com', 80); 

% Set size of receiving buffer, if needed. 
set(t, 'InputBufferSize', 30000); 

% Open connection to the server. 
fopen(t); 

% Transmit data to the server (or a request for data from the server). 
fprintf(t, 'GET /'); 

% Pause for the communication delay, if needed. 
pause(1) 

% Receive lines of data from server 
while (get(t, 'BytesAvailable') > 0) 
    t.BytesAvailable 
    DataReceived = fscanf(t) 
end 

% Disconnect and clean up the server connection. 
fclose(t); 
delete(t); 
clear t 