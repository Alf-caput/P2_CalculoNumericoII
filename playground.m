A = [
    0, 1, 0, 0, 1;
    1, 0, 1, 0, 0;
    0, 1, 0, 1, 1;
    0, 0, 1, 0, 0;
    1, 0, 1, 0, 0;
];

G = digraph(A);
plot(G)

arista = [2, 3];
res = perteneceCiclo(A, arista);

disp(res);


function res = perteneceCiclo(A, arista)

    [fil, col] = size(A);
    % La matriz de adyacencia debe ser cuadrada y simétrica
    if ~(fil == col && isequal(A, A'))
        disp('Error: Matriz de adyacencia no válida');
        return
    end

    % La arista debe tener dos componentes enteras distintas que toman valores entre 1 y el número de nodos
    if ~(length(arista) == 2 && isinteger(arista) && arista(1) ~= arista(2) && all(v >= 1 & v <= fil))
        disp('Error: Arista no válida');
        return
    end
    
    % Eliminar arista
    A(arista, flip(arista)) = 0;
    
    % Una arista pertenece a un ciclo si se puede ir de uno de sus extremos al otro tras eliminarla del grafo inicial 
    res = dfs(A, arista(1), arista(2));
end

% function pertenece = perteneceCiclo(grafo, inicio, objetivo)    
%     visto = zeros(1, size(grafo, 1));
%     visto(inicio) = 1;
%     pertenece = dfs(grafo, inicio, objetivo, visto);
% end

function res = dfs(grafo, actual, objetivo, visto)
    vecinos = find(grafo(actual, :));

    for i = 1:length(vecinos)
        vecino = vecinos(i);

        if visto(vecino) == 0
            visto(vecino) = -visto(actual);

            if dfs(grafo, vecino, objetivo, visto)
                res = true;
                return;
            end

        elseif visto(vecino) == visto(actual)
            res = true;
            return;
        end

    end

    res = false;

end