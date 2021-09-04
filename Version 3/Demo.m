%% CREACIÓN DEL ROBOT:
    %1º. Crear un vector con el número de discos de cada sección.
        NLinks = [30 20 12 7];
        
    %2º. Array de vectores con los ángulos relativos entre discos.
        Alpha = [pi/2 pi/2 pi/2 pi/2];
    
    %3º. Se crea el robot, pasándole por parámetro los vectores anteriores.
        robot = HRRobot(NLinks, Alpha);
        
%% MOVIMIENTO A UNA POSICIÓN DESEADA:
    %1º. Se inicializa un vector configuración, cuyo valor inicial es el
    %   de la posición de reposo del robot. En realidad se trata de una
    %   estructura, la cual contiene por un lado el nombre de la
    %   articulación y por otro la posición de dicha articulación.
        homeConfig = homeConfiguration(robot);
        robot.setConfig(homeConfig);
        
    %2º. Declarar la posición deseada y sacar su matriz de transformación.
        targetPos = [100 -80 150];
        tform = trvec2tform(targetPos);
        
    %3º. Llamar al método iKine del robot. Este método devuelve la
    %   configuración deseada, el error cometidoy el numero de iteraciones.
        [newConfig, error, iter] = robot.iKineGradientDescent(tform);
        
    %4º. Establecer la configuración nueva y dibujar el robot.
        robot.setConfig(newConfig);
        %robot.plot();
        robot.plot2('BodyColor', 'b', 'EndLinkColor', 'g');
        
    %5º. Es posible mostrar la posicion actual del extremo:
        currentPos = tform2trvec(robot.fKine(robot.Config));
        
    %   O incluso de cualquier disco del robot:
        link6Pos = tform2trvec(getTransform(robot, robot.Config, 'L1.6'));
        