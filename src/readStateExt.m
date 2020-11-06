% Copyright (C) 2017 Istituto Italiano di Tecnologia (IIT)
% All rights reserved.
%
% This software may be modified and distributed under the terms of the
% GNU Lesser General Public License v2.1 or any later version.

function [q, dq, d2q, m, dm, d2m, time] = readStateExt(n, filename)
% READSTATEEXT reads data from robot acqisizion. 
%
% Inputs 
% - n       : is the number of degrees of freedom of the part of the robot;
% - filename: is the name of the file containing the data from stateEXT:o 
%             port.

format = '%d %f ';
fid    = fopen(filename);

data   = textscan(fid,'%s','delimiter','\n');
text=data{1};

nlines = length(text);
time=[];
q=[];
dq=[];
d2q=[];
m=[];
dm=[];
d2m=[];
for j = 1 : 10
   format = [format, '('];
   for i = 1 : n
      if j < 9
         format = [format, '%f '];
      else
         format = [format, '%d '];
      end
   end
   format = [format, ') [ok] '];
end
for i=1:nlines
    line = text{i};
    C    = textscan(line, format);

    time = [time; C{1, 2}];

    q    = [q; cell2mat(C(1, 3    :3+  n-1))]; % n columns of "q" value
    dq   = [dq; cell2mat(C(1, 3+  n:3+2*n-1))]; % n columns of "dq" value
    d2q  = [d2q; cell2mat(C(1, 3+2*n:3+3*n-1))]; % n columns of "d2q" value

    m    = [m; cell2mat(C(1, 3+3*n:3+4*n-1))]; % n columns of "m" value
    dm   = [dm; cell2mat(C(1, 3+4*n:3+5*n-1))]; % n columns of "dm" value
    d2m  = [d2m; cell2mat(C(1, 3+5*n:3+6*n-1))]; % n columns of "d2m" value

end


if fclose(fid) == -1
   error('[ERROR] there was a problem in closing the file')
end
end
