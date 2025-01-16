function [x,y,z] = xyzread(filename,varargin)


























narginchk(1,inf) 
nargoutchk(3,3)
assert(isnumeric(filename)==0,'Input error: filename must ba a string.') 
assert(exist(filename,'file')==2,['Cannot find file ',filename,'.'])



fid = fopen(filename); 
T = textscan(fid,'%f %f %f',varargin{:}); 
fclose(fid);



x = T{1}; 
y = T{2}; 
z = T{3}; 

end