%
%   function h = cmesh(varargin)
%
%   Modified version of MATLAB's MESH() function.
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%                   MathWorks Inc.
%
%   Date:           9/6/2016
%
%   Copyrigth(C) 2016, MathWorks Inc.
%

function h = cmesh(varargin)
%MESH   3-D mesh surface.
%   MESH(X,Y,Z,C) plots the colored parametric mesh defined by
%   four matrix arguments.  The view point is specified by VIEW.
%   The axis labels are determined by the range of X, Y and Z,
%   or by the current setting of AXIS.  The color scaling is determined
%   by the range of C, or by the current setting of CAXIS.  The scaled
%   color values are used as indices into the current COLORMAP.
%
%   MESH(X,Y,Z) uses C = Z, so color is proportional to mesh height.
%
%   MESH(x,y,Z) and MESH(x,y,Z,C), with two vector arguments replacing
%   the first two matrix arguments, must have length(x) = n and
%   length(y) = m where [m,n] = size(Z).  In this case, the vertices
%   of the mesh lines are the triples (x(j), y(i), Z(i,j)).
%   Note that x corresponds to the columns of Z and y corresponds to
%   the rows.
%
%   MESH(Z) and MESH(Z,C) use x = 1:n and y = 1:m.  In this case,
%   the height, Z, is a single-valued function, defined over a
%   geometrically rectangular grid.
%
%   MESH(...,'PropertyName',PropertyValue,...) sets the value of
%   the specified surface property.  Multiple property values can be set
%   with a single statement.
%
%   MESH(AX,...) plots into AX instead of GCA.
%
%   MESH returns a handle to a surface plot object.
%
%   AXIS, CAXIS, COLORMAP, HOLD, SHADING, HIDDEN and VIEW set figure,
%   axes, and surface properties which affect the display of the mesh.
%
%   See also SURF, MESHC, MESHZ, WATERFALL.

%-------------------------------
%   Additional details:
%
%   MESH sets the FaceColor property to background color and the EdgeColor
%   property to 'flat'.
%
%   If the NextPlot axis property is REPLACE (HOLD is off), MESH resets
%   all axis properties, except Position, to their default values
%   and deletes all axis children (line, patch, surf, image, and
%   text objects).

%   Copyright 1984-2015 The MathWorks, Inc.

%   J.N. Little 1-5-92
%   Modified 2-3-92, LS.

    [~, cax, args] = parseplotapi(varargin{:},'-mfilename',mfilename);
    [reg, prop]=parseparams(args);
    nargs=length(reg);

    % Error if given uiaxes
    if ~isempty(cax) && isa(cax,'matlab.ui.control.UIAxes')
        error(message('MATLAB:ui:uiaxes:general'));
    end

    if nargs < 1
        error(message('MATLAB:narginchk:notEnoughInputs'));
    elseif nargs > 4
        error(message('MATLAB:narginchk:tooManyInputs'));
    end
    if rem(length(prop),2)~=0,
        error(message('MATLAB:mesh:ExpectedPropertyValuePairs'))
    end

    % do input checking
    regdata = reg;
    if numel(reg{end}) == 2
        regdata(end) = [];
    end

    user_view = 0;
    if isempty(cax)
        cax = newplot(cax);
        parax = cax;
        hold_state = ishold(cax);
    else
        parax = cax;
        cax = ancestor(cax,'Axes');
        hold_state = true;
    end

    hparent = get(cax,'Parent');
    fc = get(cax,'Color');
    if strcmpi(fc,'none')
        if isprop(hparent,'Color')
            fc = get(hparent,'Color');
        elseif isprop(hparent,'BackgroundColor')
            fc = get(hparent,'BackgroundColor');
        end
    end

    if nargs == 1
        z=reg{1};
        hh = matlab.graphics.chart.primitive.Surface('ZData',z,'FaceColor',fc,'EdgeColor','flat', ...
            'FaceLighting','none','EdgeLighting','flat','Parent',parax);
        
    elseif nargs == 2
        [x,y]=deal(reg{1:2});
        if ischar(y)
            error(message('MATLAB:mesh:InvalidArgument'));
        end
        [my, ny] = size(y);
        [mx, nx] = size(x);
        if mx == my && nx == ny
            hh = matlab.graphics.chart.primitive.Surface('ZData',x,'CData',y,'FaceColor',fc,'EdgeColor','flat', ...
                'FaceLighting','none','EdgeLighting','flat','Parent',parax);
        else
            if my*ny == 2 % must be [az el]
                hh = matlab.graphics.chart.primitive.Surface('ZData',x,'FaceColor',fc,'EdgeColor','flat', ...
                    'FaceLighting','none','EdgeLighting','flat','Parent',parax);
                set(cax,'View',y);
                user_view = 1;
            else
                error(message('MATLAB:mesh:InvalidInputs'));
            end
        end
        
    elseif nargs == 3
        [x,y,z]=deal(reg{1:3});
        if ischar(y) || ischar(z)
            error(message('MATLAB:mesh:ArgumentInvalid'));
        end
        if min(size(y)) == 1 && min(size(z)) == 1 % old style
            hh = matlab.graphics.chart.primitive.Surface('ZData',x,'FaceColor',fc,'EdgeColor','flat', ...
                'FaceLighting','none','EdgeLighting','flat','Parent',parax);
            set(cax,'View',y);
            user_view = 1;
        else
            hh = matlab.graphics.chart.primitive.Surface('XData',x,'YData',y,'ZData',z,...
                'FaceColor',fc,'EdgeColor','flat', ...
                'FaceLighting','none','EdgeLighting','flat','Parent',parax);
        end
        
    elseif nargs == 4
        [x,y,z,c]=deal(reg{1:4});
        hh = matlab.graphics.chart.primitive.Surface('XData',x,'YData',y,'ZData',z,'CData',c,...
            'FaceColor',fc,'EdgeColor','flat', ...
            'FaceLighting','none','EdgeLighting','flat','Parent',parax);
    end

    if ~isempty(prop),
        set(hh,prop{:})
    end

    if ~hold_state && ~user_view
        view(cax,3); grid(cax,'on');
    end

    if nargout == 1
        h = hh;
    end

end