%
%   [vecarrow, csflabels] = mbCreateVectorArrow(axes,objgroup,name,rgbcolor,width,startpos,endpos,scale,fontsize)
%
%   Plots a vector arrow with parametrized characteristics.
%
%   See example "examples/basic_shapes_demo.m" for more info on how to use this function.
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%
%   Date:           9/6/2016
%
%   Copyrigth(C) 2016, Vassilios Tsounis
%

function [vecarrow, csflabels] = mbCreateVectorArrow(axes,objgroup,name,rgbcolor,width,startpos,endpos,scale,fontsize)

    % Process input arguments
    vec_length = norm(startpos - endpos,2);
    vec = (endpos - startpos);
    vec_u = vec(1);
    vec_v = vec(2);
    vec_w = vec(3);
    
    % Create initial transfor group for pose initialization
    vectf = hgtransform('Parent',axes);
    
    % Add & initialize graphics objects to figure
    hold on;
    vecarrow = quiver3('Parent',vectf,startpos(1),startpos(2),startpos(3),vec_u,vec_v,vec_w,1);
    hold off;

    % Configure quiver frame format properties
    vecarrow.Color = rgbcolor;
    vecarrow.LineWidth = width;
    vecarrow.MaxHeadSize = 0.1;

    % Render latex labels for csf arrows
    %randmax = vec_length/20;
    %randoff = (randmax)*rand(3).';
    vec_label_pos = [startpos(1) startpos(2) startpos(3)].' + (vec/2);
    
    % Create labels
    vec_label = strcat('$\vec{\bf{r}}_{',name,'}$');
    
    % Render the csf lebels
    axis_limits = axis(axes);
    if ~isempty(fontsize)
        csflabels = text(axes,vec_label_pos(1),vec_label_pos(2),vec_label_pos(3),vec_label,'Interpreter','latex','FontSize',fontsize);
    else    
        csflabels = text(axes,vec_label_pos(1),vec_label_pos(2),vec_label_pos(3),vec_label,'Interpreter','latex','FontSize',80*scale/(axis_limits(4)-axis_limits(3)));
    end
     
    % Attach graphics objects to external transform parent object group
    vecarrow.Parent = objgroup;
    csflabels.Parent = objgroup;
     
end
