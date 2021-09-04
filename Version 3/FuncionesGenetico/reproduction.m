function son = reproduction(f, m, varargin)
%REPRODUCTION Mezcla los genes de dos individuos y devuelve un nuevo
%fenotipo, añadiendo mutaciones en los genes.
%   
%   son = reproduction(f, m, 'parametro', __, ...);
%
%   - f: father
%   - m: mother
%
%   PARAMETROS ADICIONALES:
%       - 'wF': influencia del padre. Influencia de la madre wM = 1 - wF.
%
%       - 'MutationProb': [Prob1 Prob2]
%           * Prob1: probabilidad de mutacion genotipo NLinks.
%           * Prob2: probabilidad de mutacion genotipo Alfas.
%
%       - 'MutationAmp': [Amp1 Amp2]
%           * Amp1: amplitud de mutacion genotipo NLinks.
%           * Amp2: amplitud de mutacion genotipo Alfas.
    
    %% INICIALIZACIONES
    
    rng('shuffle');    
    wF = rand(); %Porcentaje de influencia del padre.
    
    mutationProb = [0.9 0.9];
    mutationAmp = [0.05 0.05];
    
    for i = 1:2:length(varargin)
        switch (varargin{i})
            case 'wF'
                wF = varargin{i+1};
                
            case 'MutationProb'
                mutationProb = varargin{i+1};
                
            case 'MutationAmp'
                mutationAmp = varargin{i+1};
        end
    end
    
    %% PONDERACION DE GENES
    
        nLinksF = cast(wF * f.nLinks, 'uint32');
        nLinksM = cast((1 - wF) * m.nLinks, 'uint32');
        alfasF = wF * f.alfas;
        alfasM = (1 - wF) * m.alfas;

        nLinksSon = nLinksF + nLinksM;   %GenHijo = Suma ponderada GenPadres
        alfasSon = alfasF + alfasM;

    %% MUTACION
    
        nLinksSon = cast(mutateVector(nLinksSon, mutationProb(1), mutationAmp(1)), 'uint32');
        nSections = length(nLinksSon);
        for j = 1:nSections
            if nLinksSon(j) == 0
                nLinksSon(j) = 1;
            end
        end
        
        alfasSon = mutateVector(alfasSon, mutationProb(2), mutationAmp(2));
    
    %% CREACION DEL HIJO
    
        son = Phenotype(nLinksSon, alfasSon);
end
