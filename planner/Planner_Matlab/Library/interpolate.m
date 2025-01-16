function interpolated_theta = interpolate(safe_theta,num_points)
dim = size(safe_theta, 1);
x = safe_theta';


d = sqrt(sum(diff(x).^2, 2));


s = [0; cumsum(d)];


s_interp = linspace(0, s(end), num_points);
x_interp = zeros(num_points, dim);
for i = 1:dim
    x_interp(:,i) = interp1(s, x(:,i), s_interp, 'linear');
end

interpolated_theta = x_interp';
end

