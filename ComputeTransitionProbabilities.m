function P = ComputeTransitionProbabilities( stateSpace, controlSpace, map, gate, mansion, cameras )
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, controlSpace,
%   map, gate, mansion, cameras) computes the transition probabilities
%   between all states in the state space for all control inputs.
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
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

% put your code here

    K = size(stateSpace, 1);
    L = size(controlSpace, 1);
    [M, N] = size(map);
    F = size(mansion, 1);
    H = size(cameras, 1);
    global p_c gamma_p;
    P = zeros(K, K, L);
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

function k = findStateSpaceInd(n, m, stateSpace) % return 0 if [n, m] not found
    for k=1:size(stateSpace, 1)
       if (stateSpace(k, 1) == n && stateSpace(k, 2) == m)
          return; 
       end
    end
    k = 0;
end
