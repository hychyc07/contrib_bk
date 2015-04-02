function [] = printTrajectory(fileName, range, dimension)

data = load(fileName);

x = 1:1:size(data, 1);
dimension = dimension * 3 - 2;
size(data, 1)

%plotting position
figure,
plot(x, data(:, dimension));
ylim([-range range]);
hold on

%plotting velocity
p = plot(x, data(:, dimension + 1));
ylim([-range range]);
set(p, 'Color', 'red')
hold on

%plotting acceleration
p= plot(x, data(:, dimension + 2));
ylim([-range range]);
set(p, 'Color', 'green')

xlabel('t')
ylabel('trajectory');
legend('position', 'velocity', 'acceleration', 'Location', 'NorthOutside');

end

