nPoints = 0;

cubo = collisionBox(delta, delta, delta);

for ii = 1:19
    for jj = 1:20
        for kk = 1:20
            if M(kk,jj,ii) == 'x'
                x = Xmin + (kk - 1) * delta;
                y = Ymin + (jj - 1) * delta;
                z = Zmin + (ii - 1) * delta;
                cubo.Pose = trvec2tform([x y z]);
                [~, p] = show(cubo);
                p.EdgeColor = 'none';
                p.FaceColor = 'r';
                %scatter3(x, y, z, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'b');
                nPoints = nPoints + 1;
                hold on;
            end
        end
    end
end
zlim([0, 258])
xlim([-200, 200])
ylim([-200, 200])
axis equal
lightangle(45, 30)
hold off;