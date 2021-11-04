%
%   mbPlotLatexExpression (string, fontsize)
%
%   Generates latex from raw string and plots results.
%
%   Inputs:
%       string      Latex strings for rendering
%       fontsize    Font size of test visualization
%
%   Authors:        Martin Waser (ezsym original author)
%                   Vassilios Tsounis, tsounisv@ethz.ch (fixes, extensions)
%
%   Date:           June 2016
%
%   Copyrigth(C) 2016, Vassilios Tsounis
%

function mbPlotSymbolicExpression(expr, fontsize)

    string = strcat('$$ ',latex(expr),' $$');

    figure('Color','white','Menu','none','units','normalized','outerposition',[0 0 1 1]);
    text(0.5, 0.5, string, 'FontSize',fontsize, 'Color','k','HorizontalAlignment','Center', ...
            'VerticalAlignment','Middle', 'Interpreter','latex');
    axis off;

end

