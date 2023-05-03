

data = csvread('FT_data.csv');

x = data(:,3);
x = lowpass(x,1,500);

windowSize = 10; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;

%z = filter(b,a,y);

a = 0.8;
z = zeros(1,length(x));
z(1) = x(1);

for i = 2:length(x)

    z(i) = a*z(i-1)+(1-a)*x(i);


end

hold on
%plot(x)
%plot(y)
plot(z)



