function G = ComputeStageCosts( stateSpace, controlSpace, map, gate, mansion, cameras )
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, controlSpace, map, gate, mansion,
%   cameras) computes the stage costs for all states in the state space
%   for all control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (K x 2)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       controlSpace:
%           A (L x 1)-matrix, where the l-th row represents the l-th
%           element of the control space.
%
%       map:
%           A (M x N)-matrix describing the terrain of the estate map.
%           Positive values indicate cells that are inaccessible (e.g.
%           trees, bushes or the mansion) and negative values indicate
%           ponds or pools.
%
%   	gate:
%          	A (1 x 2)-matrix describing the position of the gate.
%
%    	mansion:
%          	A (F x 2)-matrix indicating the position of the cells of the
%           mansion.
%
%    	cameras:
%          	A (H x 3)-matrix indicating the positions and quality of the 
%           cameras.
%
%   Output arguments:
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and 
%           apply control input l.

% put your code here
    P = ComputeTransitionProbabilities(stateSpace, controlSpace, map, gate, mansion, cameras);
    K = size(stateSpace, 1);
    L = size(controlSpace, 1);
    [M, N] = size(map);
    F = size(mansion, 1);
    H = size(cameras, 1);
    global p_c gamma_p pool_num_time_steps detected_additional_time_steps;
    for i=1:L
       if(controlSpace(i) == 'w')
           w_ind = i;
       elseif (controlSpace(i) == 'n')
           n_ind = i;
       elseif (controlSpace(i) == 'e')
           e_ind = i;
       elseif (controlSpace(i) == 's')
           s_ind = i;
       elseif (controlSpace(i) == 'p')
           p_ind = i;
       end
    end
    G = inf(K, L);
    gateInd = findPointInd(gate(2), gate(1), stateSpace);
    m = gate(2);
    n = gate(1);
    PC1 = 0;
    PC2 = 0;
    PC3 = 0;
    PC4 = 0;
    for u=m+1:M % up
        if (map(u, n) > 0)
           t_cam = findPointInd(u, n, cameras);
           if (t_cam)
               PC1 = cameras(t_cam, 3)/(u-m);
           end
           break;
        end
    end

    for d=1:m-1 % down
        if (map(m-d, n) > 0)
           t_cam = findPointInd(m-d, n, cameras);
           if (t_cam)
               PC2 = cameras(t_cam, 3)/d;
           end
           break;
        end
    end

    for l=1:n-1 % left
        if (map(m, n-l) > 0)
           t_cam = findPointInd(m, n-l, cameras);
           if (t_cam)
               PC3 = cameras(t_cam, 3)/l;
           end
           break;
        end
    end

    for r=n+1:N % right
        if (map(m, r) > 0)
           t_cam = findPointInd(m, r, cameras);
           if (t_cam)
               PC4 = cameras(t_cam, 3)/(r-n);
           end
           break;
        end
    end

    p_hide_at_gate = (1-PC1)*(1-PC2)*(1-PC3)*(1-PC4);
    
    for c=1:L
        for i=1:K
           con = controlSpace(c);
           [J_n, J_m] = PredictState(stateSpace(i, :), con, map);
           j = findPointInd(J_m, J_n, stateSpace);
           if(j == 0)
               continue;
           elseif (j == gateInd)
               if(con == 'p')
                   p_success_pic = 1 - sum(P(i,:, c));
                   G(i, c) = p_hide_at_gate * (1-p_success_pic) + 7 * (1-p_hide_at_gate) * (1-p_success_pic) + p_success_pic;
               else
                   G(i, c) = p_hide_at_gate + 7 * (1-p_hide_at_gate) + 1 - sum(P(i,:, c));
               end
           elseif (map(J_m, J_n) < 0)
               if (con == 'p')
                   G(i, c) = 1 * P(i, j, c) + 7 * P(i, gateInd, c) + 1 - sum(P(i,:, c));
               else
                   G(i, c) = 4 * P(i, j, c) + 10 * P(i, gateInd, c) + 1 - sum(P(i,:, c));
               end
               
           else
               G(i, c) = P(i, j, c) + 7 * P(i, gateInd, c) + 1 - sum(P(i,:, c));
           end    
       end
    end
        
end

function [n, m] = PredictState(state, control, map) % return [0, 0] for not valid move
    n_now = state(1);
    m_now = state(2);
    [m_max, n_max] = size(map);
    if (control == 'n' && m_now < m_max && map(m_now+1, n_now) <= 0)
        n = n_now;
        m = m_now + 1;
    elseif (control == 's' && m_now > 1 && map(m_now-1, n_now) <= 0)
        n = n_now;
        m = m_now - 1;
    elseif (control == 'w' && n_now > 1 && map(m_now, n_now-1) <=0)
        n = n_now - 1;
        m = m_now;
    elseif (control == 'e' && n_now < n_max && map(m_now, n_now+1) <=0)
        n = n_now + 1;
        m = m_now;
    elseif (control == 'p')
        n = n_now;
        m = m_now;
    else
       m = 0;
       n = 0;
    end
end

function t = findPointInd(m, n, points) % return 0 if not exit
    t = 0;
    for i=1:size(points, 1)
       if (points(i, 1) == n && points(i, 2) == m)
          t = i;
          return
       end
    end
end
