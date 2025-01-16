function dist = distLinSeg2PC(P,Q,PC)
    min_dist = inf;
    for row = 1:size(PC, 1) 
        N = PC(row, :); 
        PQ = Q - P;
        PN = N - P;
        t = dot(PN, PQ) / dot(PQ, PQ);
        t = max(0, min(1, t));
        closestPoint = P + t * PQ;
        dist = norm(N - closestPoint);
        if dist < min_dist
            min_dist = dist;
        end
    end
    dist = min_dist;    
end
