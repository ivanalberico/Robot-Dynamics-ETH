%
%   mbSimulationOverview(time, data, datainfo, conf)
%
%   This function provides some nicely bundled functionality for generating
%   plots of data series which the user may also associate with symbolic
%   arrays.
%
%   Inputs:
%       time            Array of X-axis data to plot.
%       data            Array of Y-axis data to plot.
%       datainfo        Symbolic array of variables used for the legend
%                       names.
%       
%       conf.FontSize   Fontsize to use globally. Titles, labels and ticks
%                       scale appropriately.
%       conf.Scale      Scale can be 'log' or 'lin'(default).
%       conf.TitleX     Custom Xlabel for each subplot. This is set as an
%                       cell array for multiple subplots.
%       conf.TitleY     Custom Ylabel for each subplot. This is set as an
%                       cell array for multiple subplots.
%       conf.Title      Custom title for each subplot. This is set as an
%                       cell array for multiple subplots.
%       conf.DataSet    Matrix which holds the indeces of the subset of the
%                       'data' argument to plot. Each row is a subplot.
%       conf.Print      Set to 'pdf' if an export of the figure at the end
%                       of the simulation is desired. 
%       
%       
%
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%
%   Date:           9/6/2016
%
%   Copyrigth(C) 2016, Vassilios Tsounis
%

function mbSimulationOverview(time, data, datainfo, conf)

    % Tool console output
    display('MechBox::Simulation Overview: ');
    display('    Generating simulation data overview...');
    
    if ~isempty(conf.FontSize)
        fontsize = conf.FontSize;
    else
        fontsize = 18;
    end
    
    % Size of conf.DataSet and data-subset to plot
    Ns = numel(datainfo);
    if ~isempty(conf.DataSet)
        
        % Check dimensions of the dataset to plot
        Nss = size(conf.DataSet);
        for j = 1:Nss(2)
            for i = 1:Nss(1)
                if conf.DataSet(i,:)> Ns
                    error('error: MechBox: mbSimulationOverview: "conf.DataSet" values are out of range for the index of the system state vector.');
                end
            end
        end
        
        % The number of rows in the DataSet variable sets the number of
        % subplots
        Nsubplots = Nss(1);
        dataset = conf.DataSet;
    else
        dataset = 1:Ns;
        Nsubplots = 1;
    end
    % Create visualiaztionfigure
    overview_figure = figure('units','normalized','outerposition',[0 0 1 1]);
    overview_figure.NumberTitle = 'off';
    overview_figure.Name = 'MechBox : Simulation Overview';
    
    % Plot tha data - with subplots if necessary
    if Nsubplots > 1
        
        for i = 1:Nsubplots
            overview_axes{i} = axes(overview_figure);
            if isfield(conf, 'SubPlotOrientation')
                switch conf.SubPlotOrientation
                    case 'vertical'
                        subplot(Nsubplots,1,i, overview_axes{i});
                    case 'horizontal'
                        subplot(1,Nsubplots,i, overview_axes{i});
                    otherwise
                        error('Error: mbSimulationOverview: Incorrect orientation for "SubPlotOrientation" parameter.');
                end
            else
                subplot(Nsubplots,1,i, overview_axes{i});
            end
            
            axesdataset = dataset(i,:);
            plotconf = conf;
            if isfield(conf,'Title') && ~isempty(conf.Title{i})
                plotconf.Title = conf.Title{i};
            end
            if isfield(conf,'TitleX') && ~isempty(conf.TitleX{i})
                plotconf.TitleX = conf.TitleX{i};
            end
            if isfield(conf,'TitleY') && ~isempty(conf.TitleY{i})
                 plotconf.TitleY = conf.TitleY{i};
            end
            
            plotOverviewData(overview_axes{i}, time, data, datainfo, axesdataset, fontsize, plotconf);
            
        end
        
    else
        overview_axes = axes(overview_figure);
        axesdataset = dataset;
        plotOverviewData(overview_axes, time, data, datainfo, axesdataset, fontsize, conf);
    end
    
    % Export image
    if isfield(conf, 'Print') && ~isempty(conf.Print)
       switch  conf.Print
           case 'pdf'
                set(overview_figure,'Units','Inches');
                figdimensions = get(overview_figure,'Position');
                set(overview_figure,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[figdimensions(3), figdimensions(4)]);
                filedatetime = datestr(now,'HH_MM_SS_mm_dd_yy');
                print(overview_figure,filedatetime,'-dpdf','-r0');
           otherwise
               error('Error: mbSimulationOverview: Incorrect export type');
       end
        
    end
    
    % Tool console output
    display('    Done');
    
end

function plotOverviewData (axes, time, data, datainfo, dataset, fontsize, conf)

    % Plot the data for specific axes
    if ~isempty(conf) && isfield(conf,'Scale') && ~isempty(conf.Scale)
        switch conf.Scale
            case 'lin'
                plot(axes, time, data(dataset,:));
            case 'log'
                loglog(axes, time, data(dataset,:));
            otherwise
                plot(axes, time, data(dataset,:));
        end
    else
        plot(axes, time, data(dataset,:));
    end   
    
    % Enable plot grid on axes
    grid on;
    
    % Configure the axes
    
    set(axes,'TickLabelInterpreter', 'latex','FontSize',fontsize - 4);
    
    if ~isempty(conf) && isfield(conf,'Title') && ~isempty(conf.Title)
        title(axes, conf.Title,'Interpreter','latex','FontSize',fontsize - 2);
    else
        title(axes, 'System Response - ${\bf x}_{s}(t)$','Interpreter','latex','FontSize',fontsize - 2)
    end
    
    if ~isempty(conf) && isfield(conf,'TitleX') && ~isempty(conf.TitleX)
        xlabel(axes, conf.TitleX,'Interpreter','latex','FontSize',fontsize - 2);
    else
        xlabel(axes, '$t$ [s]','Interpreter','latex','FontSize',fontsize - 2);
    end
    
    if ~isempty(conf) && isfield(conf,'TitleY') && ~isempty(conf.TitleY)
        ylabel(axes, conf.TitleY,'Interpreter','latex','FontSize',fontsize - 2);
    else
        ylabel(axes, '${\bf x}_{s}$ [units]','Interpreter','latex','FontSize',fontsize - 2);
    end
    
    % Generate the legend data
    datacounter = 1;
    for i = 1:numel(datainfo)
        for j = 1:numel(dataset)
           if i == dataset(j)
              datanames{datacounter} = mbSymbolic2Latex([],[],datainfo(i),fontsize - 2,'latex');
              datacounter = datacounter + 1;
              break;
           end 
        end
    end
    
    % Generate the plot legend
    datalegend = legend(axes,'show');
    datalegend.FontSize = fontsize;
    datalegend.Interpreter = 'latex';
    datalegend.Box = 'on';
    datalegend.String = datanames;
    
end
