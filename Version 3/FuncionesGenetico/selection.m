function [ps, scoresSel] = selection(p, scores)
%SELECTION Selecciona a los mejores individuos de una poblacion.
%
%   [ps, scoresSel] = selection(p, scores)
%
%   PARAMETROS:
%       - p: poblacion.
%       - scores: vector de puntuaciones.

    %% INICIALIZACIONES
    
    rngSeed = rng('shuffle');
    
    scoresSel = scores;
    meanScore = mean(scores);
    probs = scores / meanScore;
    nMembers = length(p);    
    ps = {};    
    j = 0;
    
    %% BUCLE DE SELECCION
    
    while j == 0
        for i = 1:nMembers
           rngSeed = rng(rngSeed);
           if ( rand() <= probs(i) )
               j = j + 1;
               ps{j} = p{i};
               scoresSel(j) = scores(i);
           end       
        end
    end
    
    scoresSel = scoresSel(1:j);
end

