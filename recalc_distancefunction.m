function dmat_out = recalc_distancefunction(num_of_nodes,dmat)
dmat_out = dmat;
for i = 1:num_of_nodes
    for j=1:num_of_nodes
        if(i~=j)
                [e L] = dijkstra(dmat,i,j);
                dmat_out(i,j) = e;
                dmat_out(j,i) = e;  
        end
    end
end