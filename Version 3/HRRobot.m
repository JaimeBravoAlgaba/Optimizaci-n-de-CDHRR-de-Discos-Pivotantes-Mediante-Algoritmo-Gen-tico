%30/04/2021

classdef HRRobot < HRRTree
    %HRROBOT Objeto Robot Hiperredundante.
    
    properties
        BaseSTL_File;
        LinkSTL_File;
        BaseSTL;
        LinkSTL;
    end
    
    methods (Access = public)
        %% CONSTRUCTOR
        function obj = HRRobot(NLinks, Alpha)
            obj@HRRTree(NLinks, Alpha);
            %Default STL files:
            obj.BaseSTL_File = "Robot_Base.stl";
            obj.LinkSTL_File = "Mod_Boro_Triang.stl";
            obj.BaseSTL = 0;
            obj.LinkSTL = 0;
        end
        
        %% PLOT
        function plot(obj, varargin)
            obj.show(obj.Config, varargin{:});
            obj.tipAxes();
        end
        
        %% PLOT2
        function plot2(obj, varargin)
            baseColor = [0 1 0];
            bodyColor = [0 0 1];
            endLinkColor = [0 1 0];
            
            config = obj.Config;
            visuals = 'on';
            
            %Customized settings:
            for i = 1:2:length(varargin)
                switch varargin{i}
                    case 'Config'
                       config = varargin{i+1};
                       
                    case 'BaseColor'
                       baseColor = varargin{i+1};
                       
                    case 'BodyColor'
                       bodyColor = varargin{i+1};
                       
                    case 'EndLinkColor'
                       endLinkColor = varargin{i+1};
                       
                    case 'Visuals'
                       visuals = varargin{i+1};
                end
            end
            
            if strcmp(visuals, 'on')
                %Si no se han leído los STL, los lee:
                if ~((isempty(obj.BaseSTL))||(isempty(obj.LinkSTL)))
                    obj.addVisuals();
                end
                
                %Generar la malla a partir del STL
                base = collisionMesh(obj.BaseSTL.Points);
                link = collisionMesh(obj.LinkSTL.Points);

                base.Pose = getTransform(obj, config, obj.BaseName);
                [~, patchObj] = show(base);
                hold on;
                patchObj.EdgeColor = 'none';
                patchObj.FaceColor = baseColor;

                lastLink = 0;
                for j = 1:obj.NSections
                    for i = 1:obj.NLinks(j)
                        %Situar las mallas en la posición del disco
                        link.Pose = getTransform(obj, config, obj.BodyNames{1, lastLink+i});

                        %Graficar los discos, guardando el objeto "Patch"
                        [~, patchObj] = show(link);
                        hold on;
                        
                        %Editar las propiedas de los "Patch" para establecer el color
                        patchObj.EdgeColor = 'none';
                        if i < obj.NLinks(j)
                            patchObj.FaceColor = bodyColor;
                        else
                            patchObj.FaceColor = endLinkColor;
                        end
                    end
                    lastLink = lastLink + obj.NLinks(j);
                end
            
            else
                obj.plot('Visuals', 'off');
            end
            
            obj.tipAxes();
            
            hold off;
        end
        
        function tipAxes(obj)
            hold on;
            tform = getTransform(obj, obj.Config, obj.EndLink);
            endPos = tform2trvec(tform);
            endQuat = tform2quat(tform);
            plotTransforms(endPos, endQuat, 'FrameSize', 30);
            
            xlim([-obj.TotalLinks*5 obj.TotalLinks*5]);
            ylim([-obj.TotalLinks*5 obj.TotalLinks*5]);
            zlim([-5 (obj.TotalLinks+1)*5]);
            hold off;
        end
        
        %% ELLIPSOID
        function plotEllipsoid(obj, scale, J)
            eigenValues = eig(J*J');
            s1 = eigenValues(1)*scale;
            s2 = eigenValues(2)*scale;
            s3 = min(eigenValues)*scale;
            
            tform = getTransform(obj, obj.Config, obj.EndLink);
            pos = tform2trvec(tform);
            rot = tform2eul(tform, 'XYZ');
            
            [X, Y, Z] = ellipsoid(pos(1), pos(2), pos(3), s1, s2 , s3);
            surface = surf(X, Y, Z, 'FaceAlpha', 0.25);
            rotate(surface, [rot(1) rot(2) rot(3)], norm(rot)*180/pi, [pos(1) pos(2) pos(3)]);
        end
        
        %% ADD VISUALS
        function addVisuals(obj, varargin)
            %Default settings:
            baseSTL = obj.BaseSTL_File;
            linkSTL = obj.LinkSTL_File;
            
            %Read STL Files:
            obj.BaseSTL = stlread(obj.BaseSTL_File);
            obj.LinkSTL = stlread(obj.LinkSTL_File);
            
            %Customized settings:
            for i = 1:2:length(varargin)
                switch varargin{i}                       
                    case 'Base'
                       baseSTL = varargin{i+1};
                       
                    case 'Link'
                       linkSTL = varargin{i+1};
                end
            end
            
            obj.Base.addVisual("Mesh", baseSTL);
            
            for i = 1:obj.TotalLinks
                obj.Bodies{i}.addVisual("Mesh", linkSTL);
            end
        end
    end
end