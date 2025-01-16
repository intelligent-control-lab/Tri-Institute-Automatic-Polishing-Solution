load('data/weld.mat')

planes = [-0.13677271909286323, -0.001307522919873557, -1, 0.35681409561655236;
          -0.09944386985869111, 0.9507137174883696, -1, 0.4148449626591891;
          -0.0028754998304916695, -1, -0.00048562669056519125, -0.1531872239587;
          -0.028656068479931213, -0.9169732738103323, -1, -0.02635994915558708;
          -0.04227291092492677, -0.0017900871745648592, -1, 0.04253270252138967;
          -0.027539864123062554, 0.9174694697754313, -1, -0.02670323898692613;
          0.003517293221011959, -1, -0.0026534115250074763, 0.1535545078421211;
          -0.09670160529976317, -0.9547657737231579, -1, 0.4150487079383617];
      
point_anchor = [0.2137; 0; 0.1701];
theta = 0;
P1P2 = 0;

origin_axis3 = [0;0;0];
arr_axis3 = weld' - origin_axis3;
point_anchor_axis3 = point_anchor - origin_axis3;
abc = planes(:,1:3);
planePoints = [];
for i = 1:size(planes,1)
    a = planes(i,1);
    b = planes(i,2);
    c = planes(i,3);
    d = planes(i,4);
    zaxis3 = -d/c;
    point_axis3 = [0,0,zaxis3] - origin_axis3';
    planePoints = [planePoints; point_axis3];
end
save('data/axis3Relative_wpframe.mat','abc','planePoints','point_anchor_axis3','arr_axis3')