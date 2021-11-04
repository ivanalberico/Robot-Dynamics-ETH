% proNEu: A tool for rigid multi-body mechanics in robotics.
% 
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

function [] = proneulogo()

% Parameters
versionnumber = '2.0';
yeardate = '2017';

% Print data
plogoalt = '                                         ____==========_______\n                              _--____   |    | ""  " "|       \\\n                             /  )8}  ^^^| 0  |  =     |  o  0  |\n                           </_ +-==B vvv|""  |  =     |    "" "|\n                              \\_____/   |____|________|________|\n                                      (_(  )\\________/___(  )__)\n                                        |\\  \\            /  /\\\n                  _   _ ______          | \\  \\          /  /\\ \\\n                 | \\ | |  ____|         | |\\  \\        /  /  \\ \\\n  _ __  _ __ ___ |  \\| | |__  _   _     (  )(  )      (  \\   (  )\n |  _ \\|  __/ _ \\| . ` |  __|| | | |     \\  / /        \\  \\   \\  \\\n | |_) | | | (_) | |\\  | |___| |_| |      \\|  |\\        \\  \\  |  |\n | .__/|_|  \\___/|_| \\_|______\\__,_|       |  | )___     \\  \\ \\  )__\n | |                                       (  ) /  /     (  )  (/  /\n |_|  by RSL                               /___\\__/      /___\\ /__/\n';
header = '=======================================================================';
footer = '=======================================================================';
copyright = ['Copyright(C) ' yeardate ', Robotic Systems Lab, ETH Zurich\nAll rights reserved.\nhttp://www.rsl.ethz.ch'];
pversion = ['Version: \t ' versionnumber '\n'];
description = '\n\nMultiple Rigid-Body Dynamics using the Projected Newton-Euler Method\n\n';
authors = 'Authors: \t Marco Hutter \n \t\t Christian Gehring \n \t\t Dario C. Bellicoso \n \t\t Vassilios Tsounis\n';

% Print outputs
fprintf([header '\n']);
fprintf([plogoalt '\n']);
fprintf([description '\n']);
fprintf([authors '\n']);
fprintf([pversion '\n']);
fprintf([copyright '\n']);
fprintf([footer '\n\n']);

end
