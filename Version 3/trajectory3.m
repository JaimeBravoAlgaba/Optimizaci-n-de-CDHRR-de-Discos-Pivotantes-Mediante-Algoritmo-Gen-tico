function [X, Y, Z] = trajectory3(start, goal, N, type)
%TRAJECTORY Divides the way between two points in a trayectory of N points
%   TYPES:
%       - 'RECT'
%       - 'PARAB'
    X = zeros(1, N);
    Y = zeros(1, N);
    Z = zeros(1, N);
    switch type
        case 'RECT'
            for i=0:N
                X(i+1) = start(1) + i*(goal(1) - start(1))/N; 
                Y(i+1) = start(2) + i*(goal(2) - start(2))/N; 
                Z(i+1) = start(3) + i*(goal(3) - start(3))/N;
            end
            
        case 'PARAB'
            m = (goal(2) - start(2))/(goal(1) - start(1));
            for i=0:N-1
                Z(i+1) = start(3) + i*(goal(3) - start(3))/N;
                X(i+1) = abs( sqrt( (start(3) - Z(i+1))/(1+m*m) ) + start(1) );
                if goal(1) < 0
                    X(i+1) = -X(i+1);
                end
                Y(i+1) = m*(X(i+1)-start(1)) + start(2);
            end
    end
end

