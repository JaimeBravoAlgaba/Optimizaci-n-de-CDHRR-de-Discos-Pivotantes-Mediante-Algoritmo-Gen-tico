function vector = mutateVector(vector, prob, amp)
%MUTATEVECTOR Modifica las posiciones de un vector de forma aleatoria.
%   PARAMETROS:
%       - vector: vector a mutar.
%       - prob: probabiblidad de que se produzca una mutación en un
%               elemento (tanto por uno).
%       - amp: amplitud de la mutacion (tanto por uno de la norma del
%              vector).
    rng('shuffle');
    
    vectorLength = length(vector);  % Numero de elementos
    vectorNorm = norm(cast(vector, 'double'));      % Modulo del vector
    amp = amp * vectorNorm;         % Amplitud de la mutacion
    
    % Recorrer el vector modificando sus posiciones...
    for i = 1:vectorLength
        K = rand();
        % Probabilidad de que haya mutación...
        if K <= prob
            Q = rand();
            % 50% de probabiblidad de sumar o de restar...
            if Q <= 0.5
                vector(i) = vector(i) + amp;
            else
                vector(i) = vector(i) - amp;
            end
        end
    end

end

