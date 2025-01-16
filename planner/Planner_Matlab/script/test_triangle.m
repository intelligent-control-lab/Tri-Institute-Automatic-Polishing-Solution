TR = stlread('figure/wp_inSurf.STL')
visual_plane_setting;

triangles = {};
for i=1:1:size(TR.ConnectivityList,1)
    plane = TR.Points(TR.ConnectivityList(i,:),:);
    plane(:,2) = plane(:,2);
    plane = setVertice(plane,M1);
    x = plane(:,1);
    y = plane(:,2) - 0.20563;
    z = plane(:,3);
    triangle = [x,y,z];
    triangle = triangle';
    triangles = [triangles, {triangle}];
end

for i = 1:size(triangles)
    triangle = cell2mat(triangles(i));
    x = triangle(1,:);
    y = triangle(2,:);
    z = triangle(3,:);
    patch('XData', x,'YData', y, 'ZData',z, 'FaceColor',"cyan", "EdgeColor", "blue", "LineStyle", "-");
    hold on;
end
axis equal
lighting=camlight('left');

view(-20,3);
    xlim=[1,2.5];
    ylim=[-0.5,0.5];
    zlim=[0,2];