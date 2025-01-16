




























x = [1 1 2 2 2 3 3];
y = [1 2 1 2 3 1 3];
z = [1 2 4 5 6 7 9]; 

scatter(x,y,500,z,'filled') 
axis([0 4 0 4])
colorbar






[X,Y,Z] = xyz2grid(x,y,z)






h = imagesc(X(1,:),Y(:,1),Z); 
axis xy
axis([0 4 0 4])
set(h,'alphadata',~isnan(Z))
colorbar




