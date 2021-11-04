%
%   latexstring = mbSymbolic2Latex (string, sym, fontsize, output)
%
%   This function provides functionality to generate latex output from
%   symbolic expressions. The output can either be a string or also be
%   plotted in a figure. 
%
%   Inputs:
%       pos         Position of 'string' argument as prefix or suffix
%       string      Additional latex strings for rendering
%       sym         Sumbolic expression to be converted to latex
%       fontsize    Font size of test visualization
%       outputmode  Set the output to either create plot figure or to
%                   return string data as raw or latex.
%
%   Return:
%       latexstring String with either raw latex or fixed wth '$$ $$'
%                   characters.
%
%   Notes on Use:
%
%       1. If a symbolic variables is prefixed with a 'D' character, it
%       will be interpreted as an absolute derivative w.r.t. time and will
%       be rendered with a '\dot' accent. Example:
%
%       syms Dx_f -> \dot{x}_{f}
%
%       2. Greek lower case and higher case are handled automatically as
%       long as the latinized spelling is correct.
%
%       3. Support for automatically recognizing multi-character subscripts
%       exists. Example:
%
%       Dx_foobar -> \dot{x}_{foobar}
%
%
%
%   Authors:        Martin Waser (ezsym original author)
%                   Vassilios Tsounis, tsounisv@ethz.ch (fixes, extensions)
%
%   Date:           June 2016
%
%   Copyrigth(C) 2016, Vassilios Tsounis
%

function latexstring = mbSymbolic2Latex(pos, string, sym, fontsize, outputmode)

    % Check if sum argument is a symbolic expression
    if ~isa(sym,'sym')
        error('Input must be a symbolic expression');
    end

    % Check font size
    if isempty(fontsize) 
        fontsize = 18;
    end

    % Table of greek symbols
    greeks = {  'alpha',   'beta',    'gamma', 'Gamma',  'delta',  ...
                'Delta',   'epsilon', 'Zeta', 'zeta',  'theta',  'Theta',  ... 
                'eta',     'iota',    'kappa', 'lambda', 'Lambda', ... 
                'mu',      'nu',      'xi',    'Xi',     'pi',     ...
                'Pi',      'rho',     'sigma', 'Sigma',  'tau',    ...
                'upsilon', 'Upsilon', 'phi',   'Phi',    'chi'     ...
                'psi',     'Psi',     'omega', 'Omega'};

    % Parsing corrections
    parsing_corrections  = {'{\b{\eta}}', '{\th{\eta}}', '{\Th{\eta}}', '{\u{\psilon}}', '{\U{\psilon}}', '{\Z{\eta}}', '{\z{\eta}}', '{\e{\psilon}}' ; 
                            '{\beta}',    '{\theta}',    '{\Theta}',    '{\upsilon}',    '{\Upsilon}',    '{Z}' ,       '{\zeta}' ,  '{\epsilon}'};
    
    % Latin symbols
    small_latins = {'a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z'};
    large_latins = {'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z'};
    charlist = {small_latins{1,:},large_latins{1,:}};
    
    % Arabic numerals
    numbers = {'0','1','2','3','4','5','6','7','8','9'};
    
    % Variable name and expression delimiters
    delims = {'(',')','{','}','\',',',' ','_','^','/','*','.'};
    
    % Get the raw conversion to latex
    latexexprin = latex(sym);
   
    % Modify for multi-character subscripts
    symvars = symvar(sym);
    for i = 1:numel(symvars)
        
        symrawstring{i} = char(symvars(i));
        foundsubscript = false;
        
        % Find start of subscript
        for j = 1:numel(symrawstring{i})
            if symrawstring{i}(j) == '_';
             subscriptstart = j+1;
             foundsubscript = true;
             break;
            end
        end
        
        if foundsubscript == true
            symstringparts{i,1} = symrawstring{i}(1:subscriptstart-2);
            symstringparts{i,2} = symrawstring{i}(subscriptstart);
            symstringparts{i,3} = symrawstring{i}(subscriptstart+1:end);
            
            if numel(symstringparts{i,1}) > 1
                if numel(symstringparts{i,3}) > 1
                    searchterm = strcat('\mathrm{',symstringparts{i,1},'}_{',symstringparts{i,2},'}\mathrm{',symstringparts{i,3},'}');
                elseif numel(symstringparts{i,3}) == 1
                    searchterm = strcat('\mathrm{',symstringparts{i,1},'}_{',symstringparts{i,2},'}',symstringparts{i,3});
                elseif numel(symstringparts{i,3}) == 0
                    searchterm = strcat('\mathrm{',symstringparts{i,1},'}_{',symstringparts{i,2},'}');
                end
                replaceterm = strcat('\mathrm{',symstringparts{i,1},'}_{',symstringparts{i,2},symstringparts{i,3},'}');
                
            elseif numel(symstringparts{i,1}) == 1
                if numel(symstringparts{i,3}) > 1
                    searchterm = strcat(symstringparts{i,1},'_{',symstringparts{i,2},'}\mathrm{',symstringparts{i,3},'}');
                elseif numel(symstringparts{i,3}) == 1
                    searchterm = strcat(symstringparts{i,1},'_{',symstringparts{i,2},'}',symstringparts{i,3});
                elseif numel(symstringparts{i,3}) == 0
                    searchterm = strcat(symstringparts{i,1},'_{',symstringparts{i,2},'}'); 
                end
                replaceterm = strcat(symstringparts{i,1},'_{',symstringparts{i,2},symstringparts{i,3},'}');
            end
            
            latexexprin = strrep(latexexprin, searchterm, replaceterm);
        end
        
    end
    
    % Replace special subscripts
    latexexprin = strrep(latexexprin, '_{tm1}', '_{t-1}');
    latexexprin = strrep(latexexprin, '_{tp1}', '_{t+1}');
    
    % Replace "hat_" prefix with "\hat{}"
    % TODO
    
    % Replace diff() time derivatives with D operator 
    % TODO
    
    % Modify to substitute the "D" prefix with an time derivative "dot"
    symvars = symvar(sym);
    for i = 1:numel(symvars)
        
        symrawstring{i} = char(symvars(i));
        found_diff = false;
        found_double_diff = false;
        
        strlength = numel(symrawstring{i});
        if strlength >= 5
           if (strcmp(symrawstring{i}(1:5), 'Delta'))
               isdelta = true;
           else
               isdelta = false;
           end
        else
            isdelta = false;
        end
        
        % Find starting diff operator 'D'
        if (numel(symrawstring{i}) > 1) && (~isdelta)
            if (strcmp(symrawstring{i}(1),'D')) && (~strcmp(symrawstring{i}(1:2),'DD')) && (~strcmp(symrawstring{i}(1:3),'DDD'))
                for j = 2:numel(symrawstring{i})
                    for k = 1:numel(delims)
                       if ((symrawstring{i}(j) == delims{k}) || (j == numel(symrawstring{i}))) && (j >= 2) 
                            if numel(symrawstring{i}) == 2
                                symvarname{i} = symrawstring{i}(2);
                            else
                                if symrawstring{i}(j) == delims{k}
                                    symvarname{i} = symrawstring{i}(2:j-1);
                                elseif j == numel(symrawstring{i})
                                    symvarname{i} = symrawstring{i}(2:j);
                                end
                            end
                            found_diff = true;
                            break; 
                       end
                    end
                    if found_diff == true
                       break; 
                    end
                end
            elseif (strcmp(symrawstring{i}(1:2),'DD')) && (~strcmp(symrawstring{i}(1:3),'DDD'))
                for j = 3:numel(symrawstring{i})
                    for k = 1:numel(delims)
                       if ((symrawstring{i}(j) == delims{k}) || (j == numel(symrawstring{i}))) && (j >= 3) 
                            if numel(symrawstring{i}) == 3
                                symvarname{i} = symrawstring{i}(3);
                            else
                                if symrawstring{i}(j) == delims{k}
                                    symvarname{i} = symrawstring{i}(3:j-1);
                                elseif j == numel(symrawstring{i})
                                    symvarname{i} = symrawstring{i}(3:j);
                                end
                            end
                            found_double_diff = true;
                            break; 
                       end
                    end
                    if found_double_diff == true
                       break; 
                    end
                end
            end
        end
        
        if found_diff == true
            if numel(symrawstring{i}) == 2
                searchterm = strcat('D',symvarname{i});
            else
                searchterm = strcat('\mathrm{D',symvarname{i},'}');
            end
            
            replaceterm = strcat('\dot{',symvarname{i},'}');
            latexexprin = strrep(latexexprin, searchterm, replaceterm);
            
        elseif found_double_diff == true
            if numel(symrawstring{i}) == 3
                searchterm = strcat('DD',symvarname{i});
            else
                searchterm = strcat('\mathrm{DD',symvarname{i},'}');
            end
            
            replaceterm = strcat('\ddot{',symvarname{i},'}');
            latexexprin = strrep(latexexprin, searchterm, replaceterm);
        end
        
        
    end
    
    for k=1:numel(greeks)                         
        latexexprin = strrep(latexexprin, greeks{k}, ['{\',greeks{k},'}']);
        %latexexprin = strrep(latexexprin, greeks{k}, ['\',greeks{k}]);
    end
    
    for k=1:numel(parsing_corrections)/2                       
        latexexprin = strrep(latexexprin, parsing_corrections{1,k}, parsing_corrections{2,k});
    end
    
    for k=1:numel(greeks)                         
        latexexprin = strrep(latexexprin, ['\\',greeks{k}], ['{\',greeks{k},'}']);
    end
    
    % Check font size
    if ~isempty(string)
        
        % Default to prefix for string argument
        if isempty(pos)
            pos = 'prefix';
        end
        
        switch pos
            case 'prefix'
                latexexprin = strcat(string,latexexprin);
            case 'suffix'
                latexexprin = strcat(latexexprin,string);
            otherwise
                error('error: mbSymbolic2Latex: incorrect type specified for "pos" argument.');
        end
    end
    
    % Add the inline-latex markers\
    if ~isempty(outputmode) && (strcmp(outputmode,'latex') || strcmp(outputmode,'plot'))
        latexexprin = ['$$ ',latexexprin,' $$'];
        latexstring = latexexprin;
    elseif ~isempty(outputmode) && strcmp(outputmode,'raw')
        latexstring = latexexprin;
    else
        error('error: mbSymbolic2Latex: incorrect output mode specified.');
    end
    
    if ~isempty(outputmode) && strcmp(outputmode,'plot')
        % Write output to a text-only test-figure
        %figure('Color','white','Menu','none','units','normalized','outerposition',[0 0 1 1]);
        figure('Color','white','units','normalized','outerposition',[0 0 1 1]);
        text(0.5, 0.5, latexstring, 'FontSize',fontsize, 'Color','k','HorizontalAlignment','Center', ...
                'VerticalAlignment','Middle', 'Interpreter','latex');
        axis off;
    end

end

