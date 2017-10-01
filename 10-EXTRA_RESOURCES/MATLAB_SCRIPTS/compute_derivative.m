
function [jumps] =  compute_derivative(scan, min_dist)

jumps = zeros(1, length(scan));

for i =2:length(scan)-1

    l = scan(i-1);
    r = scan(i+1);
    if(l > min_dist && r > min_dist)
        der = (r - l)/2.0;
        jumps(i) = der;
    else
        jumps(i) = 0;
    
    jumps.append(0);
    end 
end
end
