function [] = printVector(fileName)

data = load(fileName);

x = 1:1:size(data, 1);
size(data, 1)

figure,
plot(x, data(:, 1));

end