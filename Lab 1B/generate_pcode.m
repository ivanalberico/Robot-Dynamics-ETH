% Generate pcode for all solution files
workfolder = pwd;
mfilesfolder = [pwd '/solutions/mfiles'];
pcodesfolder = [pwd, '/solutions/pcodes'];

% Add the folders to path
addpath(mfilesfolder);
addpath(pcodesfolder);

% Generate pcode for all scripts
cd(pcodesfolder)
delete('*.p')
pcode(mfilesfolder)
cd(workfolder)