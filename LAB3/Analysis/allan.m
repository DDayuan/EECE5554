bag = rosbag('LocationA.bag');
bSel = select(bag,'Topic','/vectornav');

msgStructs = readMessages(bSel,'DataFormat','struct');
%msgStructs{1};
maxNumM = 100;
frenquency = 40;
t0 = 1 / frenquency;
L = size(msgStructs, 1);

maxM = 2.^floor(log2(L/2));
m = logspace(log10(1), log10(maxM), maxNumM).';
m = ceil(m); % m must be an integer.
m = unique(m); % Remove duplicates.
tau = m*t0;
gyrox = [];
gyroy = [];
gyroz = [];
for i = 1:L
    line = msgStructs{1}.Data;
    a = string(line);
    cell_str = strsplit(a, ',');
    x = str2double(cell_str{2});
    y = str2double(cell_str{3});
    z = str2double(cell_str{4});
    gyrox = [gyrox, x];
    gyroy = [gyroy, y];
    gyroz = [gyroz, z];
end
data = gyrox;
dataatype = "gyrox";
theta = cumsum(data, 1)*t0;
type="all";

[avarx, taux] = allanvar(gyrox,m, L);
adevx = sqrt(avarx);
[avary, tauy] = allanvar(gyroy,m, L);
adevy = sqrt(avary);
[avarz, tauz] = allanvar(gyroz,m, L);
adevz = sqrt(avarz);

figure
loglog(taux, [adevx, adevy,adevz] )
legend('gyrox', 'gyroy', 'gyroz')
title('Allan Deviation')
xlabel('\tau');
ylabel('\sigma(\tau)')
grid on
axis equal

figure
plot(gyrox, 'Linewidth', 2)
legend('gyrox')
title('gyro x')
xlabel('time')
ylabel('rad/s')
grid on

figure
plot(gyroy, 'Linewidth', 2)
legend('gyroy')
title('gyro y')
xlabel('time')
ylabel('rad/s')
grid on

figure
plot(gyroz, 'Linewidth', 2)
legend('gyroz')
title('gyro z')
xlabel('time')
ylabel('rad/s')
grid on
Nx = calculateN(taux, adevx)
Ny = calculateN(tauy, adevy)
Nz = calculateN(tauz, adevz)

Kx = calculateK(taux, adevx)
Ky = calculateK(tauy, adevy)
Kz = calculateK(tauz, adevz)

Bx = calculateB(taux, adevx)
By = calculateB(tauy, adevy)
Bz = calculateB(tauz, adevz)

function N = calculateN(tau, adev)
    slope = -0.5;
    logtau = log10(tau);
    logadev = log10(adev);
    dlogadev = diff(logadev) ./ diff(logtau);
    [~, i] = min(abs(dlogadev - slope));

    % Find the y-intercept of the line.
    b = logadev(i) - slope*logtau(i);

    % Determine the angle random walk coefficient from the line.
    logN = slope*log(1) + b;
    N = 10^logN;
end

function K = calculateK(tau, adev)
    slope = 0.5;
    logtau = log10(tau);
    logadev = log10(adev);
    dlogadev = diff(logadev) ./ diff(logtau);
    [~, i] = min(abs(dlogadev - slope));

    % Find the y-intercept of the line.
    b = logadev(i) - slope*logtau(i);

    % Determine the rate random walk coefficient from the line.
    logK = slope*log10(3) + b;
    K = 10^logK;
end

function B = calculateB(tau, adev)
    slope = 0;
    logtau = log10(tau);
    logadev = log10(adev);
    dlogadev = diff(logadev) ./ diff(logtau);
    [~, i] = min(abs(dlogadev - slope));

    % Find the y-intercept of the line.
    b = logadev(i) - slope*logtau(i);

    % Determine the bias instability coefficient from the line.
    scfB = sqrt(2*log(2)/pi);
    logB = b - log10(scfB);
    B = 10^logB;
end