

function plainRRT

createGraphic = [10; 50; 100; 500; 1000; 5000; 10000];

%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial configuration %
%%%%%%%%%%%%%%%%%%%%%%%%%

x0 = [0.5;0.5];
% Number of vertices in the tree
K = 10000;
% Incremental distance
deltaQ = 0.5;

% The grap will be represented as vectors of [x;y] points, where
Graph = repmat(randn(2,1),1,K);

f = figure;
% plot(x0);
hold on;

Graph(:,1) = x0;

for k = 1:K
    qrand = rand(2,1); % Pull a random configuration from the uniform distribution.
    dsq = sum((Graph - repmat(qrand,1,size(Graph,2))).^2,1);
    [~,i] = min(dsq);
    qnear_index = nearest_vertex(qrand, Graph);
    qnear = Graph(:,qnear_index);
    qnew = new_conf(qnear,qrand, deltaQ);
    % keyboard;
    line([Graph(1,i),qrand(1)], [Graph(2,i), qrand(2)], 'Color', 0.3*[1 1 1]);
    Graph(:,k) = qnew;
    if (k < 500 || mod(200,k) == 1)
        drawnow;
    end
    if (any(createGraphic == k))
        saveas(f, sprintf('plainRRT%d',k),'pdf');
    end
end

% Save picture generated for use in the thesis.
saveas(f, 'plainRRT', 'pdf');

%%%%%%%%%%%%%%%%%%%
% Local functions %
%%%%%%%%%%%%%%%%%%%

function i = nearest_vertex(qrand, G)
dsq = sum((G- repmat(qrand,1,size(G,2)).^2),1);
[~,i] = min(dsq);
end

function qnew = new_conf(qnear, qrand, deltaQ)
qnew = qrand; % Do not consider the configuration atm.
              % Neither do we prune the length of an arm yet.
end


end % file