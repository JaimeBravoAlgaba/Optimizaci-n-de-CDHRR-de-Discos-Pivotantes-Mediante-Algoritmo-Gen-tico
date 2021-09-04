addpath(genpath('FuncionesGenetico'));

%OBJETIVO:
%Optimización de la configuración del robot para alcanzar con éxito el
%punto [140, 20, 90] de manera que la manipulabilidad sea máxima.

global robotLength;

initNLinks = [11 30 23 18 18];         %Solucion de partida (nLinks).
initAlfas = [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2];   %Solucion de partida (alfas).

nSections = length(initNLinks);                  %Número de secciones de los robots.
totalLinks = 100;                %Número total de enlaces de los robots.
robotLength = 5 * totalLinks;
minAlpha = pi*10/180;           %Mínimo valor de alpha admisible.

nMembers = 50;                   %Número de muestras en la población.
maxIter = 50;                   %Máximo de iteraciones para el algoritmo.
iter = 0;

scores = zeros(1, nMembers);     %Vector de putuaciones.
meanScores = zeros(1, maxIter); %Historico de puntuaciones medias.
maxScores = zeros(1, maxIter);  %Historico de puntuaciones maximas.

%Posicion objetivo:
x0 = 0;
y0 = 200;
z0 = -100;

mutationProb = [0.9 0.0];
mutationAmp = [0.05 0.05];

logService('TRACE', 'PRIMERA POBLACION...');
fenotipos = initPopulation(nMembers, nSections, totalLinks, 'InitNLinks', initNLinks, 'InitALfas', initAlfas, 'MutationProb', mutationProb, 'MutationAmp', mutationAmp);

logService('TRACE', '... END PRIMERA POBLACION');

%% MAIN FLOW
logService('TRACE', 'MAIN LOOP...');
while iter < maxIter
    %EVALUACIÓN
    scores = evaluate(fenotipos, x0, y0, z0);
    %parfor i = 1:nMembers
        %scores(i) = evaluationWorkVolume(fenotipos{i});
    %end
    meanScores(iter+1) = mean(scores);
    maxScores(iter+1) = max(scores);
    logService('INFO', 'Mean Score:');
    logService('INFO', meanScores(iter+1));
    logService('INFO', 'Max Score:');
    logService('INFO', maxScores(iter+1));
    
    %SELECCIÓN
    [ps, scoresSel] = selection(fenotipos, scores);
    
    %CRUCE
    fenotipos = mix(ps, scoresSel, nMembers, 'MutationProb', mutationProb, 'MutationAmp', mutationAmp);
    
    iter = iter + 1;
    logService('INFO', "Progress: " + iter*100/maxIter + "%");
end
logService('TRACE', '... END MAIN LOOP');
%%
scores = evaluate(fenotipos, x0, y0, z0);
%parfor i = 1:nMembers
    %scores(i) = evaluationWorkVolume(fenotipos{i});
%end

logService('INFO', 'End score:');
logService('INFO', scores);

%% EVALUATION
function S = evaluate(fenotipos, x0, y0, z0)
    logService('DEBUG', 'INI - evaluate');
    global robotLength;
    
    nMembers = length(fenotipos);
    P = cell(1, nMembers);
    
    for i = 1:nMembers
        P{i} = HRRTree(fenotipos{i}.nLinks, fenotipos{i}.alfas);
    end
    
    S = zeros(1, nMembers);
    parfor i = 1:nMembers
        currentPos = tform2trvec(P{i}.fKine(P{i}.Config));
        [X, Y, Z] = trajectory3(currentPos, [x0 y0 z0], 6, 'RECT');
        [~, error, ~] = P{i}.move(X, Y, Z, 'Default', 'Error', 0.05 * robotLength);
        
        J = P{i}.jacobian();
        m = manipulability(J);
        
        if error{1, 1} >= 10
            S(i) = 0;
        else
            S(i) = m;
        end
    end
    logService('DEBUG', 'FIN - evaluate');
end