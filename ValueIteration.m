function [ J_opt, u_opt_ind ] = ValueIteration( P, G )
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by Value Iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
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
    J_opt = ones(K, 1);
    u_opt_ind = zeros(K, 1);
    V_to_go = zeros(K, L);
    while(1)
       for i=1:L
          V_to_go(:, i) = G(:, i) + P(:, :, i) * J_opt; 
       end
       [J_opt_run, u_opt_ind] = min(V_to_go, [], 2);
       if(max(abs(J_opt_run-J_opt)) < 1e-5)
          J_opt = J_opt_run;
          break;
       end
       J_opt = J_opt_run;
    end
end