function [ J_opt, u_opt_ind ] = PolicyIteration( P, G )
%POLICYITERATION Value iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space.

% put your code here
    [K, L] = size(G);
    J_opt = zeros(K, 1);
    u_opt_ind = ones(K, 1);
    while(1)
       g = diag(G([1:K, u_opt_ind]));P
       ind1 = repmat([1:K], [1, K]);
       ind2 = repmat()
       p = P([])
       [J_opt_run, u_opt_ind] = min(V_to_go, [], 2);
       if(max(abs(J_opt_run-J_opt)) < 1e-5)
          J_opt = J_opt_run;
          break;
       end
       J_opt = J_opt_run;
    end

end

