function [Q, Q_k] = ClosestPointOnTriangleToPoint(V, P)
% Inputs:   V is a 2x3 matrix of vertices of the triangle
%           P is the point we are trying to find
% Outputs:  Q is the point on the triangle closest to P
%           Q_k is a matrix containing the sides of the triangle the point
%           lies in. 

    % Add 0 to all points so that we can take the cross product later.
    V = [V; zeros(1,length(V))]; P = [P; 0];
    % Define all the edges as directional vectors
    AB = V(:,2) - V(:,1); BC = V(:,3) - V(:,2); CA = V(:,1) - V(:,3);
    BA = -AB; CB = -BC; AC = -CA;
    AP = P - V(:,1); BP = P - V(:,2); CP = P - V(:,3);
    
    if dot(AP, AB) <= 0 && dot(AP, AC) <= 0
        % Va region
        Q = V(:,1);
        Q_k = [1];
    elseif dot(BP, BA) <= 0 && dot(BP, BC) <= 0
        % Vb region
        Q = V(:,2);
        Q_k = [2];
    elseif dot(CP, CA) <= 0 && dot(CP, CB) <= 0
        % Vc region
        Q = V(:,3);
        Q_k = [3];
    elseif dot(cross(cross(BC, BA), BA), BP) >= 0 && dot(AP, AB) >= 0 && dot(BP, BA) >= 0
        % Eab region
        AQ = (dot(AP, AB)/dot(AB, AB)) * AB;
        Q = AQ + V(:,1);
        Q_k = [1,2];
    elseif dot(cross(cross(CB, CA), CA), CP) >= 0 && dot(AP, AC) >= 0 && dot(CP, CA) >= 0
        % Eac region
        AQ = (dot(AP, AC)/dot(AC, AC)) * AC;
        Q = AQ + V(:,1);
        Q_k = [1,3];
    elseif dot(cross(cross(BA, BC), BC), BP) >= 0 && dot(BP, BC) >= 0 && dot(CP, CB) >= 0
        % Ebc region
        BQ = (dot(BP, BC)/dot(BC, BC)) * BC;
        Q = BQ + V(:,2);
        Q_k = [2,3];
    else
        % Otherwise, the point is inside the triangle and we return the
        % origin.
        Q = [0; 0; 0];
        Q_k = [];
    end
    
    % Reduce back down to a 2x1 vector
    Q = Q(1:2);
end
