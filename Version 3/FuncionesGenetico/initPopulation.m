function P = initPopulation(nMembers, nSections, totalLinks, varargin)
%INITPOPULATION Devuelve un cell array de Fenotipos.
%   P = initPopulation(nMembers, nSections, totalLinks, 'parametro', __, ...);
%
%   - nMembers: numero de individuos en la poblacion.
%   - nSections: numero de secciones de los robots.
%   - totalLinks: numero de enlaces de los robots.
%
%   PARAMETROS ADICIONALES:
%       - 'InitNLinks': solucion de partida para el genotipo nLinks.
%
%       - 'InitAlfas' : solucion de partida para el genotipo alfas.
%
%       - 'MutationProb': [Prob1 Prob2]
%           * Prob1: probabilidad de mutacion genotipo NLinks.
%           * Prob2: probabilidad de mutacion genotipo Alfas.
%
%       - 'MutationAmp': [Amp1 Amp2]
%           * Amp1: amplitud de mutacion genotipo NLinks.
%           * Amp2: amplitud de mutacion genotipo Alfas.

    %% INICIALIZACIONES
    
    P = cell(1, nMembers);
    
    rngSeed = rng('shuffle');
    nLinks = rand(1, nSections);
    nLinks = (totalLinks / sum(nLinks, 'double')) * nLinks; 
    alfas = 2 * pi * rand(1, nSections);
    
    mutationProb = [0.9 0.9];
    mutationAmp = [0.05 0.05];
    
    for i = 1:2:length(varargin)
        switch (varargin{i})
            case 'InitNLinks'
                nLinks = varargin{i+1};
                
            case 'InitAlfas'
                alfas = varargin{i+1};
                
            case 'MutationProb'
                mutationProb = varargin{i+1};
                
            case 'MutationAmp'
                mutationAmp = varargin{i+1};
        end
    end
      
     %% GENERACION DE FENOTIPO
     
     for i = 1:nMembers
         rngSeed = rng(rngSeed);
         % ASIGNACION DE VALORES NLINKS
         N = 0;
         while N ~= totalLinks
             nLinksAux = cast(mutateVector(nLinks, mutationProb(1), mutationAmp(1)), 'uint32');
             for j = 1:nSections
                if nLinksAux(j) == 0
                    nLinksAux(j) = 1;
                end
             end
             N = sum(nLinksAux);
         end

         % ASIGNACION DE VALORES ALFAS        
         alfasAux = mutateVector(alfas, mutationProb(2), mutationAmp(2));       
          
         P{i} = Phenotype(nLinksAux, alfasAux); 
     end
end

