function varargout = xyz2grid(varargin)































































narginchk(1,inf) 


if isnumeric(varargin{1})
   x = varargin{1}; 
   y = varargin{2}; 
   z = varargin{3}; 
else
   [x,y,z] = xyzread(varargin{:}); 
end

assert(isequal(size(x),size(y),size(z))==1,'Dimensions of x,y, and z must match.') 
assert(isvector(x)==1,'Inputs x,y, and z must be vectors.') 




[xs,~,xi] = unique(x(:),'sorted'); 
[ys,~,yi] = unique(y(:),'sorted'); 



if numel(xs)==numel(z)
   warning 'It does not seem like the xyz dataset is gridded. You may be attempting to grid scattered data, but I will try to put it into a 2D matrix anyway. Check the output spacing of X and Y.';
end


Z = accumarray([yi xi],z(:),[],[],NaN); 


Z = flipud(Z); 



switch nargout 
   case 1 
      varargout{1} = Z; 
      
   case 3


      [varargout{1},varargout{2}] = meshgrid(xs,flipud(ys)); 
      varargout{3} = Z; 
      
   otherwise
      error('Wrong number of outputs.') 
end
      

end
