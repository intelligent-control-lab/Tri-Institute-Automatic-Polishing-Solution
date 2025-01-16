
load('results/measure.mat');
len = size(safe_theta,2);
aug = [];
step = safe_theta(:,1) - safe_theta(:,2);
step = norm(step);
step = step / 3;
for i=1:len-1
    pre = safe_theta(:,i);
    next = safe_theta(:,i+1);
    diff = next - pre;
    auglen = norm(diff) / step + 1;
    for j = 1:auglen
        tmp = pre + (j-1)*diff/auglen
        aug = [aug tmp];
    end
end
safe_theta = aug;