function [ J_opt, u_opt_ind ] = LinearProgramming( P, G )
%LINEARPROGRAMMING Value iteration
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
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
    P_2d = permute(P, [1,3,2]);
    P_2d = reshape(P_2d, [K*L, K]);
    G_1d = reshape(G, [K*L, 1]);
    T = repmat(eye(K), [L, 1]);
    f = -ones(K, 1);
    A = T-P_2d;
    b = G_1d;
    b(isinf(b)) = 10000000000000000;
    J_opt = linprog(f, A, b);
    cost = G + reshape(P_2d*J_opt, [K, L]);
    [J, u_opt_ind] = min(cost, [], 2);
end

