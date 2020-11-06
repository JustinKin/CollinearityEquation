name = 'result.xlsx';
X1 = xlsread(name,'A3:A22');
Y1 = xlsread(name,'B3:B22');
plot(X1,Y1,'.');
axis manual
hold on
X2 = xlsread(name,'C3:C22');
Y2 = xlsread(name,'D3:D22');
plot(X2,Y2,'.');
axis manual
hold on
X3 = xlsread(name,'E3:E22');
Y3 = xlsread(name,'F3:F22');
plot(X3,Y3,'.');
X4 = xlsread(name,'G3:G22');
Y4 = xlsread(name,'H3:H22');
plot(X4,Y4,'.');
legend('无噪声','像素噪声','世界噪声','同时有噪声');
