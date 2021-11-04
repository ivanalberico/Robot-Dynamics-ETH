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
%
%%%

% CLASSDEF obj = genpatch(STLfileName,matProperty)
%
% -> creates a patch part class as with
% * STLfileName: file name including patch of the coresponding STL file
% * matProperty: struct e.g. matProperty.FaceColor = [1,0,0]
% * scale: (0...1) factor how the whole patch object is scaled
%
% Properties are:
% * name_:          STLfileName, s.t. it can be identified later
% * p_:             patch handle
% * x_,y_,z_,c_:    [x,y,z,c]- of STL file (at the point of constructing)
% * matProperty_:   struct e.g. matProperty.FaceColor = [1,0,0]
%
%
% Methods are:
% obj.load():                       loads the patch in gcf
% obj.reset():                      reset patch to initial values
%
% Created by Marco Hutter on 29.08.2015
% for Matlab 2013a

classdef genpatch < handle

    properties
        name_

        p_              % rotation point
        x_              % patch x data
        y_              % patch y data
        z_              % patch z data
        c_              % patch c data
        matProperty_    % material property

    end

    methods

        function obj = genpatch(varargin)
            % constructor
            if nargin == 3
                [x,y,z] = stlread(varargin{1});
                c = ones(1,length(x));
                name = varargin{1};
                matProperty = varargin{2};
                offsets = varargin{3};
            end
            fig = figure('visible','off');
            ax = axes('Parent',fig);

            % Correct for inport data scaling and offsets
            scale_factor = 1.0e-3;
            x = x*scale_factor - offsets(1);
            y = y*scale_factor - offsets(2);
            z = z*scale_factor - offsets(3);

            % Create and store patch data
            obj.p_ = patch(ax,x,y,z,c,'visible','off');
            obj.x_ = x;
            obj.y_ = y;
            obj.z_ = z;
            obj.c_ = c;
            obj.name_ = name;

            set(obj.p_,matProperty);
            obj.matProperty_ = matProperty;

            clear ax;
            clear fig;
        end

        function [] = load (obj)
            % enables patch object rendering
            set(obj.p_,'visible','on');
        end

        function [] = reset(obj)
            set(obj.p_,...
                'xData',obj.x_,...
                'yData',obj.y_,...
                'zData',obj.z_)
        end

    end

end
