function p = mix(ps, scoresSel, nMembers, varargin)
%MIX Reproduce los individuos de una poblacion de forma aleatoria.
%   El mejor individuo de la poblacion siempre pasa a la siguiente
%   generacion.
%
%   p = mix(ps, scoresSel, nMembers)
%
%   - ps: poblacion seleccionada para la mezcla.
%   - scoresSel: puntuaciones de la poblacion seleccionada.
%   - nMembers: numero de miembro de la poblacion final.

    nMembersSel = length(ps);

    p = cell(1, nMembers);
    
    [~, best] = max(scoresSel);
    p{1} = ps{best};
    
    for i = 2:nMembers
        lovers= randi([1 nMembersSel], 1, 2);
        p{i} = reproduction(ps{lovers(1)}, ps{lovers(2)}, varargin{:});
    end
end

