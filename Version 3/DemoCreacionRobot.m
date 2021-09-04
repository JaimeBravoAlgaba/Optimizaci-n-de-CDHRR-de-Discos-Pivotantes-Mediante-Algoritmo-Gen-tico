nLinks = [30, 22, 15, 7];
alfas = [pi/6, pi/3, pi/2, 2*pi/3];
robot1 = HRRobot(nLinks, alfas);
robot1.plot2('BodyColor', 'blue', 'EndLinkColor', 'green');

[newConfig, error, iter] = robot1.move(135, 80.34, 62.11, 'Default');
robot1.plot2();
disp(error);
disp(iter);
disp(robot1.currentPos());
