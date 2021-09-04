classdef Phenotype
    %PHENOTYPE Conjunto de caracteres visibles de un individuo.
    %   PROPIEDADES (Genotipos):
    %       - NLinks: numero de enlaces en cada seccion.
    %       - Alfas: angulo de montaje de los discos de cada seccion.
    
    properties
        nLinks;
        alfas;
    end
    
    methods
        function obj = Phenotype(nLinks,alfas)
            obj.nLinks = nLinks;
            obj.alfas = alfas;
        end
    end
end

