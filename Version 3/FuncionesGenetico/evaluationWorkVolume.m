function vW = evaluationWorkVolume(fenotipo)
    logService('TRACE', "Selección Volumen de Trabajo - Montecarlo");
    logService('TRACE', "_________________________________________");

    %% INICIALIZACIONES
        logService('DEBUG', "Inicializando variables...");
        
        robot = HRRTree(fenotipo.nLinks, fenotipo.alfas);
        robotLength = robot.TotalLinks*5;

        nX = 50;
        nY = 1;
        nZ = 100;
        %nPoints = nX*nY*nZ;
        nPoints = 25;
        nSuccess = 0;
        nFailed = 0;

        xMax = 3.0*robotLength/2.0;
        xMin = 0.0;
        
        %yMax = 3*robotLength/2;
        yMax = 0;
        yMin = -yMax;

        zMax = robotLength;
        zMin = -robotLength/2;

        x0 = xMax/2;
        y0 = yMax/2;
        z0 = zMax/2;

        v = abs((xMax-xMin)*(yMax-yMin)*(zMax-zMin));

        logService('DEBUG', "... variables inicializadas.");

    %%  BUCLE PRINCIPAL
        rngSeed = rng('shuffle');
        
        for i = 1:nPoints
            %Generación de un punto aleatorio
            rngSeed = rng(rngSeed);
            p = 2*rand(1,3)-1;
            p(1) = p(1)*xMax;
            p(2) = p(2)*yMax;
            p(3) = p(3)*zMax;

            %Cálculo de un primer punto de aproximación
            q = p;
            if p(1)>=0
                q(1) = x0;
            else
                q(1) = -x0;
            end

            if p(2)>=0
                q(2) = y0;
            else
                q(2) = -y0;
            end

            if p(3)>=0
                q(3) = z0;
            else
                q(3) = -z0;
            end

            %Posicionamiento hacia el primer punto
            robot.resetConfig();
            robot.move(q(1), q(2), q(3), 'GradientDescent','Error', 1);

            %Posicionamiento hacia el punto alatorio
            [~, e, ~] = robot.move(p(1), p(2), p(3), 'Default','Error', 1);
            if e{1}<=1
                nSuccess = nSuccess + 1;
                logService('DEBUG', "¡Punto " + p(1) + "," + p(2) + "," + p(3) + " alcanzado!");
            else
                nFailed = nFailed + 1;
                logService('DEBUG', "Punto " + p(1) + "," + p(2) + "," + p(3) + " no alcanzado");
            end
        end

    vW = (nSuccess/nPoints);

end

