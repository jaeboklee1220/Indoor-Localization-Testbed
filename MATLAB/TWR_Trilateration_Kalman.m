clc; clear all; close all

set(gcf,'Color','w')

set(gcf, 'name', 'Position of Alphabot', 'menubar', 'none','numbertitle', 'off', 'toolbar', 'none', 'units', 'pixels')%, 'windowstate', 'fullscreen');
set(gcf,'units','centimeters','outerposition',[10 10 50 25]);


X_axis = 567; Y_axis = 567;
L_anchor = [0 0; 60 7800; 0 15260; 5400 15520; 6420 7200; 10310 4020; 7040 0];

bb = [4000*ones(1,21)];
k = 0;

samples = 10000;

delete(instrfindall)

s = serial('/dev/cu.usbmodem142201');    % Specify the port number
% s = serial('COM14');    % Specify the port numberccccccc

set(s,'BaudRate',115200)    % Specify baudrate
fopen(s);   % Start communication with Arduino
    subplot(1,2,1)
    hold on
    img1 = imread('lab1.jpg', 'jpg');
    imshow(img1,'border','tight')
    
        subplot(1,2,2)
    hold on
    img1 = imread('lab1.jpg', 'jpg');
    imshow(img1,'border','tight')
    
% while 1
% a = fscanrf(s);   % Reading serial value
% eval(['c = [' a ])
% end
tic
while k < samples
    
    a = fscanf(s);   % Reading serial value
    eval(['b = [' a ])
    
    
    b(b>100000) = bb(b>100000);
    b(b==0) = bb(b==0);
    
    b = sqrt(b.^2-1000^2);
    bb = zeros(7,3);
    bb(:) = b;
    bb = bb';
    x_hat = tri(bb,L_anchor)/10;
    x_hat1 = x_hat*207/704+[78 50];
    xh = x_hat1(:,1); yh = x_hat1(:,2);
    
    [xh1 yh1] = tri_Kalman1(xh(1),yh(1));
    [xh2 yh2] = tri_Kalman2(xh(2),yh(2));
    [xh3 yh3] = tri_Kalman3(xh(3),yh(3));
    
    xh_a = [xh1 ; xh2 ; xh3 ];
    yh_a = [yh1;yh2;yh3];
    bb = b;
    drawnow
    
    subplot(1,2,1)
    hold on
%     img1 = imread('lab1.jpg', 'jpg');
%     imshow(img1,'border','tight')
    scatter(xh,567-yh,10,'black','filled','LineWidth',1)
    %     scatter(L_anchor(:,1)/1000,L_anchor(:,2)/1000,100,'black','filled','LineWidth',2)
    
    %     text(xh(1),yh(1),'Tag 1','FontSize',15)
    
    
    xlabel('X axis [m]')
    ylabel('Y axis [m]')
    
    axis([0 X_axis 0 Y_axis])
    box on
    grid on
    %     set(gcf, 'units', 'normalized', 'outerposition', [1/2 1/8-0.05 2/4 6/8+0.1]);
    hold off
    
    
    
    
    subplot(1,2,2)
    hold on
%     img1 = imread('lab1.jpg', 'jpg');
%     imshow(img1,'border','tight')
    scatter(xh_a,567-yh_a,10,'black','filled','LineWidth',1)
    
    xlabel('X axis [m]')
    ylabel('Y axis [m]')
    
    axis([0 X_axis 0 Y_axis])
    box on
    grid on
    %     set(gcf, 'units', 'normalized', 'outerposition', [1/2 1/8-0.05 2/4 6/8+0.1]);
    hold off
    %
    %     result(:,k+1) = xh+1j*yh;
    
    %     F = getframe(gca);
    %     writeVideo(v,F);
    
    %         result(:,k+1) = xh+yh*1j;
    
    k = k+1;
    toc
    tic
end


% close(v)

% result_m = abs(result-L(1:V,1)-L(1:V,2)*1j);
% result_m = mean(result_m(:))

fclose(s);

% close all
function y = vb1(x, a)
% x = column vector
y = transpose(ones(a, 1)*x');
end

function y = mrv(x,V,A)
% mrv : making ranging vector
X = [x zeros(1,V)];
X1 = zeros(V+A,V);
X1(:) = X(:);
X2 = [zeros(1,V); X1];
X3 = X2(:);
X3 = X3(1:end-V);
y = zeros(V+A,V);
y(:) = X3;
y = y';

end