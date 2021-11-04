% Copyright (C) 2017  Marco Hutter, Christian Gehring, C. Dario Bellicoso,
% Vassilios Tsounis
%
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.

% CLASSDEF obj = IRB120Visualization()
%
% -> generates a visualization of the IRB120 consisting of base and 6 moving
% links
%
% obj.load(), obj.load(fig_handle)
% -> loads the patchSets in its actual configuration in the figure
%
% obj.updat(q)
% with: * q_new = [q1,q2,q3,q4,q5,q6]  generalized joint coordinates [rad]
% -> updates the structure to the new joint angles
%

classdef IRB120Visualization < handle

    properties

        % Figure and Axes
        vizFig_
        vizAx_

        % Visualization elements
        tfs_
        links_
        frames_
        eePosVec_

        % start configuration
        rotCenterPoint_
        rotAxis_
        Iorigin_

    end

    methods

        function obj = IRB120Visualization()

            mat.FaceColor = [210,112,16]/255;
            mat.EdgeColor = 'none';
            mat.AmbientStrength = 0.3;
            mat.DiffuseStrength = 0.3;
            mat.SpecularStrength = 1;
            mat.SpecularExponent = 25;
            mat.SpecularColorReflectance = 0.5;

            % Define joint DoFs and patch offsets
            obj.rotCenterPoint_{1}=[0,0,0.145];
            obj.rotAxis_{1}=[0,0,1];
            obj.rotCenterPoint_{2}=[0,0,0.290];
            obj.rotAxis_{2}=[0,1,0];
            obj.rotCenterPoint_{3}=[0,0,0.560];
            obj.rotAxis_{3}=[0,1,0];
            obj.rotCenterPoint_{4}=[0.151,0,0.630];
            obj.rotAxis_{4}=[1,0,0];
            obj.rotCenterPoint_{5}=[0.302,0,0.630];
            obj.rotAxis_{5}=[0,1,0];
            obj.rotCenterPoint_{6}=[0.372,0,0.630];
            obj.rotAxis_{6}=[1,0,0];

            % Generate patches with correct zero offsets
            obj.links_{1} = genpatch('visualization/stl/base.stl',mat,[0 0 0]);
            for i=1:6
                obj.links_{i+1} = genpatch(['visualization/stl/link',num2str(i),'.stl'],mat,obj.rotCenterPoint_{i});
            end

        end

        function [] = reset(obj)
            for i=1:length(obj.links_)
                obj.links_{i}.reset();
            end
        end

        function [] = load(obj,scale,fontsize,Iorigin)

            % Create figure window and axes
            obj.vizFig_ = figure('Name','3D visualization IRB120','Position',[100 100 800 600],'NumberTitle', 'off');
            obj.vizAx_ = axes('parent', obj.vizFig_);
            set(obj.vizFig_,'Color',[1 1 1])
            set(obj.vizAx_,'Color',[1 1 1])
            axis(obj.vizAx_, 'equal');
            axis(obj.vizAx_, 'vis3d');
            axis(obj.vizAx_, 'off');
            axis(obj.vizAx_, 'auto');

            % Fix view range
            viewscale = 0.5;
            viewlim3d = [-1 1 -1 1 0 2];
            axis(obj.vizAx_, viewscale*viewlim3d);

            % Initialize body transforms for links which move
            NB = length(obj.links_);
            obj.tfs_ = cell(NB,1);
            obj.frames_ = cell(NB,1);

            % Set the graphics object hierarchy
            for i=1:NB
                obj.tfs_{i} = hgtransform('Parent', obj.vizAx_);
                set(obj.links_{i}.p_, 'Parent', obj.tfs_{i});
            end

            % Enable the visualization of the links
            obj.links_{1}.load();
            obj.frames_{1} = gencsframe(obj.vizAx_, '0', scale, fontsize);
            for j=1:numel(obj.frames_{i})
              set(obj.frames_{1}{j}, 'parent', obj.tfs_{1});
            end
            for i=2:NB
              obj.links_{i}.load();
              obj.frames_{i} = gencsframe(obj.vizAx_, num2str(i-1), scale, fontsize);
              for j=1:numel(obj.frames_{i})
                 set(obj.frames_{i}{j}, 'parent', obj.tfs_{i});
              end
            end


            % Initialize Inertial frame
            obj.tfs_{8} = hgtransform('Parent', obj.vizAx_);
            obj.frames_{8} = gencsframe(obj.vizAx_, 'I', scale, fontsize);
            for j=1:numel(obj.frames_{8})
              set(obj.frames_{8}{j}, 'parent', obj.tfs_{8});
            end
            obj.Iorigin_ = Iorigin(:);
            TI = eye(4); TI(1:3,4) = obj.Iorigin_;
            set(obj.tfs_{8}, 'matrix', TI);

            % Initialize joint position
            obj.setJointPositions(zeros(6,1));

        end

        function [] = setJointPositions(obj, q)

            if length(q)~=length(obj.rotAxis_)
                error('Wrong dimension of q, it should be length 6.');
            end

            % Initialize temporary body transform
            btf = eye(4);
            for i=1:numel(obj.eePosVec_)
                delete (obj.eePosVec_{i});
            end

            % Use succesive homogeneous-transformations to position each
            % body in the visualization
            btf = btf*jointTf01(q);
            set(obj.tfs_{2}, 'matrix', btf);
            btf = btf*jointTf12(q);
            set(obj.tfs_{3}, 'matrix', btf);
            btf = btf*jointTf23(q);
            set(obj.tfs_{4}, 'matrix', btf);
            btf = btf*jointTf34(q);
            set(obj.tfs_{5}, 'matrix', btf);
            btf = btf*jointTf45(q);
            set(obj.tfs_{6}, 'matrix', btf);
            btf = btf*jointTf56(q);
            set(obj.tfs_{7}, 'matrix', btf);

            % Create arrow
            obj.eePosVec_ = genarrowvec(obj.vizAx_, 'ee', 12, 1.0, 'magenta', obj.Iorigin_, btf(1:3,4)-obj.Iorigin_);

            % Update figure/axes data
            drawnow;

        end

    end

end

%
% HELPER FUNCTIONS
%

function T01 = jointTf01(q)
  if (length(q)>1)
      q = q(1);
  end
  T01 = [cos(q), -sin(q), 0,     0;
         sin(q),  cos(q), 0,     0;
              0,       0, 1, 0.145;
              0,       0, 0,     1];
end

function T12 = jointTf12(q)
  if (length(q)>1)
      q = q(2);
  end
  T12 = [ cos(q),  0, sin(q),     0;
               0,  1,      0,     0;
         -sin(q),  0, cos(q), 0.145;
               0,  0,      0,     1];
end

function T23 = jointTf23(q)
  if (length(q)>1)
      q = q(3);
  end
  T23 = [ cos(q), 0, sin(q),     0;
               0, 1,      0,     0;
         -sin(q), 0, cos(q), 0.270;
               0, 0,      0,     1];
end

function T34 = jointTf34(q)
  if (length(q)>1)
      q = q(4);
  end
  T34 = [1,      0,       0, 0.134;
         0, cos(q), -sin(q),     0;
         0, sin(q),  cos(q), 0.070;
         0,      0,       0,     1];
end

function T45 = jointTf45(q)
  if (length(q)>1)
      q = q(5);
  end
  T45 = [ cos(q), 0, sin(q), 0.168;
          0,        1,    0,     0;
         -sin(q), 0, cos(q),     0;
          0,        0,    0,     1];
end

function T56 = jointTf56(q)
  if (length(q)>1)
      q = q(6);
  end
  T56 = [1,      0,       0, 0.072;
         0, cos(q), -sin(q),     0;
         0, sin(q),  cos(q),     0;
         0,      0,       0,     1];
end
