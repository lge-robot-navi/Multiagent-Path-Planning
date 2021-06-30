function index = kw_point2ind(point1, point2)
% point1 to point2
global sensorParam
    point_x = [point1(1), point2(1)];
    point_y = [point1(2), point2(2)];

    nPoints = max(abs(diff(point_x)), abs(diff(point_y)))+1;	% Number of points in line
    rIndex = round(linspace(point_x(1), point_x(2), nPoints));	% Row indices
    cIndex = round(linspace(point_y(1), point_y(2), nPoints));	% Column indices
    
    % delete index over
    nr = numel(rIndex(rIndex<sensorParam.H & rIndex>0));
    nc = numel(cIndex(cIndex<sensorParam.W & cIndex>0));
    n = min(nr, nc);
    rIndex = rIndex(1:n);
    cIndex = cIndex(1:n);

    index = sub2ind(sensorParam.size, rIndex(1:n), cIndex(1:n));               % Linear indices
end