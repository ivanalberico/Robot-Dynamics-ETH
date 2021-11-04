%%

%%

name = 'B';
syms(strcat('p_',name,'w'),strcat('p_',name,'x'),strcat('p_',name,'y'),strcat('p_',name,'z'));
eval(['p_IB = [' strcat('p_',name,'w') ' ' strcat('p_',name,'x') ' ' strcat('p_',name,'y') ' ' strcat('p_',name,'z') '];']);
eval('p_IB = p_IB(:);');

%%

