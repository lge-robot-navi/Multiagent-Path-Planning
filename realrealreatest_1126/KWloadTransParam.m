function KWloadTransParam()
    global transParam;
    
    transParam.transhost = 'tcp://220.81.76.111';
    transParam.revhost = 'tcp://localhost';                                
    transParam.revtopic   = '/mams/ph/scheduler/req';
    transParam.transtopic  =  '/mams/ph/scheduler/res';
    transParam.portNum = '';
    transParam.sendMsg = ''; 
    transParam.subMQTT = '';
    transParam.KWsubscrible = '';
    transParam.recVal = '';
end