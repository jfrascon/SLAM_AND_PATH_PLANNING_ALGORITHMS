clc;
close all;
format longG

% % 1D Gaussian PDF
% x=-5:0.1:15;
% mean = 5;
% sigma1 = 1;
% sigma2 = 2;
% sigma3 = 3;
% z0 = (1/sqrt(2*pi)) * exp(-1/2);
% plot(x, normpdf(x, mean, sigma1))
% hold on
% plot(x, normpdf(x, mean, sigma2))
% plot(x, normpdf(x, mean, sigma3))
% plot([mean - sigma1, mean + sigma1], [z0*1/sigma1, z0*1/sigma1], 'ro', 'markerfacecolor', 'r')
% plot([mean - sigma2, mean + sigma2], [z0*1/sigma2, z0*1/sigma2], 'ro', 'markerfacecolor', 'r')
% plot([mean - sigma3, mean + sigma3], [z0*1/sigma3, z0*1/sigma3], 'ro', 'markerfacecolor', 'r')
% 
% xlabel('x')
% ylabel('N(5, \sigma_i^2)')
% title('2D Gaussian PDF')
% legend('\sigma_1^2 = 1', '\sigma_2^2 = 4', '\sigma_3^2 = 9')
% grid on
% 
% x=-5:0.1:15;
% figure;
% plot(x, normpdf(x, mean, sigma1))
% hold on
% plot([mean - sigma1, mean + sigma1], [z0*1/sigma1, z0*1/sigma1], 'ro', 'markerfacecolor', 'r')
% plot([mean - 2*sigma1, mean + 2*sigma1], [(1/ (sqrt(2*pi)*sigma1))*exp(-2), (1/ (sqrt(2*pi)*sigma1))*exp(-2)], 'ro', 'markerfacecolor', 'r')
% plot([mean - 3*sigma1, mean + 3*sigma1], [(1/ (sqrt(2*pi)*sigma1))*exp(-4.5), (1/ (sqrt(2*pi)*sigma1))*exp(-4.5)], 'ro', 'markerfacecolor', 'r')
% 
% xlabel('x')
% ylabel('N(5, 1)')
% title('2D Gaussian PDF')
% legend('\sigma_1^2 = 1')
% grid on
% 
% 
% 
% 
% 
% 
% % ----- ----- ----- ----- -----
% % ----- ----- ----- ----- -----
% % ----- ----- ----- ----- -----
% % ----- ----- ----- ----- -----
% % ----- ----- ----- ----- -----






numpoints = 100;
K = [1, 2, 3];

% % ----- ----- ----- ----- -----
% 
% mu = [15;15];
% s1 = 2;
% s2 = 3;
% p = 5/6;
% s12 = p*s1*s2;
% S = [s1^2 s12; s12 s2^2];
% 
% % Calculate the orthonormal eigenvectors and the eigenvalues of the
% % covariance matrix.
% % Compare the result of my expressions with the result given by Matlab.
% [U, L] = eig(S)
% L1 = 0.5*(S(1,1) + S(2,2) + sqrt((S(1,1) + S(2,2))^2 - 4*S(1,1)*S(2,2)*(1-p^2)))
% L2 = 0.5*(S(1,1) + S(2,2) - sqrt((S(1,1) + S(2,2))^2 - 4*S(1,1)*S(2,2)*(1-p^2)))
% b = (S(2,2) - S(1,1) + sqrt((S(1,1) + S(2,2))^2 - 4*S(1,1)*S(2,2)*(1-p^2)))/(2*p*sqrt(S(1,1))*sqrt(S(2,2)))
% c = (S(2,2) - S(1,1) - sqrt((S(1,1) + S(2,2))^2 - 4*S(1,1)*S(2,2)*(1-p^2)))/(2*p*sqrt(S(1,1))*sqrt(S(2,2)))
% u1 = [1/sqrt(1 + b^2); b/sqrt(1 + b^2)]
% u1_prime = [-1/sqrt(1 + b^2); -b/sqrt(1 + b^2)]
% u2 = [1/sqrt(1 + c^2); c/sqrt(1 + c^2)]
% u2_prime = [-1/sqrt(1 + c^2); -c/sqrt(1 + c^2)]
% 
% u1.' * u2 % 0
% u1_prime.' * u2_prime % 0
% u1.' * u1 % 1
% u1_prime.' * u1_prime % 1
% u2.' * u2 % 1
% u2_prime.' * u2_prime % 1
% 
% [x1, x2] = meshgrid(mu(1)-10:0.1:mu(1)+10, mu(2)-10:0.1:mu(2)+10);
% %y_prime = (((U(1,1)^2)/L(1,1)) + ((U(2,1)^2)/L(2,2)))*(x1 - mu(1)).^2 + (((U(1,2)^2)/L(1,1)) + ((U(2,2)^2)/L(2,2)))*(x2 - mu(2)).^2 + 2*(((U(1,1)*U(1,2))/L(1,1)) + ((U(2,1)*U(2,2))/L(2,2)))*(x1 - mu(1)).*(x2 - mu(2));
% y = (1/det(S)) * (S(2,2)*(x1 - mu(1)).^2 + S(1,1)*(x2 - mu(2)).^2 - 2*S(1,2)*(x1 - mu(1)).*(x2 - mu(2)));
% G = (1/((2*pi)*sqrt(det(S))))*exp(-0.5*y);
% 
% figure;
% subplot(121);
% meshc(x1, x2, y);
% view([-55 13]);
% xlabel('x_1');
% ylabel('x_2');
% title('$\left(\vec{x} - \vec{\mu}\right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} - \vec{\mu}\right)$', 'Interpreter', 'latex');
% grid on; 
% subplot(122);
% meshc(x1, x2, G);
% xlabel('x_1');
% ylabel('x_2');
% title('$p\left(\vec{x}\right) \, = \, \frac{1}{2\,\pi \, | \Sigma |^{1/2} } \, \cdot \, e^{-\frac{1}{2} \, \cdot \, \left(\vec{x} \, - \, \vec{\mu} \right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} \, - \, \vec{\mu}\right)}$', 'Interpreter', 'latex');
% grid on; 
% 
% ellipses = ellipsedata(S, mu, numpoints, K);
% % Check if my hypothesis about the scaling factor of K in the length of
% % each principal axis.
% (1/det(S)) * (S(2,2)*(ellipses(:, 1) - mu(1)).^2 + S(1,1)*(ellipses(:, 2) - mu(2)).^2 - 2*S(1,2)*(ellipses(:, 1) - mu(1)).*(ellipses(:, 2) - mu(2)))
% (1/det(S)) * (S(2,2)*(ellipses(:, 3) - mu(1)).^2 + S(1,1)*(ellipses(:, 4) - mu(2)).^2 - 2*S(1,2)*(ellipses(:, 3) - mu(1)).*(ellipses(:, 4) - mu(2)))
% (1/det(S)) * (S(2,2)*(ellipses(:, 5) - mu(1)).^2 + S(1,1)*(ellipses(:, 6) - mu(2)).^2 - 2*S(1,2)*(ellipses(:, 5) - mu(1)).*(ellipses(:, 6) - mu(2)))
% 
% figure;
% plot(ellipses(:, 1), ellipses(:, 2), 'r');
% hold on;
% plot(ellipses(:, 3), ellipses(:, 4), 'g');
% plot(ellipses(:, 5), ellipses(:, 6), 'b');
% grid on;
% xlabel('x_1');
% ylabel('x_2');
% title('$\left(\vec{x} - \vec{\mu}\right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} - \vec{\mu}\right) \, = \, K^2$', 'Interpreter', 'latex');
% legend('K = 1', 'K = 2', 'K = 3');
% axis([5 25 5 25]);
% %text(p2(1),p2(2), sprintf('(%.0f,%.0f)',p2))
% 
% 
% alpha1 = atan2(U(2,1), U(1,1));
% p11 = [mu(1)-0.5*cos(alpha1), mu(2)-0.5*sin(alpha1)];
% p12 = [mu(1)+0.5*cos(alpha1), mu(2)+0.5*sin(alpha1)];
% 
% alpha2 = atan2(U(2,2), U(1,2));
% p21 = [mu(1)-0.5*cos(alpha2), mu(2)-0.5*sin(alpha2)];
% p22 = [mu(1)+0.5*cos(alpha2), mu(2)+0.5*sin(alpha2)];
% 
% arrow(p11,p12, 3);
% arrow(p21,p22, 3);
% 
% % ----- ----- ----- ----- -----
% 
% mu = [15;15];
% s1 = 2;
% s2 = 3;
% p = -5/6;
% s12 = p*s1*s2;
% S = [s1^2 s12; s12 s2^2];
% 
% [x1, x2] = meshgrid(mu(1)-10:0.1:mu(1)+10, mu(2)-10:0.1:mu(2)+10);
% y = (1/det(S)) * (S(2,2)*(x1 - mu(1)).^2 + S(1,1)*(x2 - mu(2)).^2 - 2*S(1,2)*(x1 - mu(1)).*(x2 - mu(2)));
% G = (1/((2*pi)*sqrt(det(S))))*exp(-0.5*y);
% 
% figure;
% subplot(121);
% meshc(x1, x2, y);
% %view([-55 9])
% xlabel('x_1');
% ylabel('x_2');
% title('$\left(\vec{x} - \vec{\mu}\right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} - \vec{\mu}\right)$', 'Interpreter', 'latex');
% grid on; 
% subplot(122);
% meshc(x1, x2, G);
% xlabel('x_1');
% ylabel('x_2');
% title('$p\left(\vec{x}\right) \, = \, \frac{1}{2\,\pi \, | \Sigma |^{1/2} } \, \cdot \, e^{-\frac{1}{2} \, \cdot \, \left(\vec{x} \, - \, \vec{\mu} \right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} \, - \, \vec{\mu}\right)}$', 'Interpreter', 'latex');
% grid on;
% 
% ellipses = ellipsedata(S, mu, numpoints, K);
% 
% figure;
% plot(ellipses(:, 1), ellipses(:, 2), 'r');
% hold on;
% plot(ellipses(:, 3), ellipses(:, 4), 'g');
% plot(ellipses(:, 5), ellipses(:, 6), 'b');
% grid on;
% xlabel('x_1');
% ylabel('x_2');
% title('$\left(\vec{x} - \vec{\mu}\right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} - \vec{\mu}\right) \, = \, K^2$', 'Interpreter', 'latex');
% legend('K = 1', 'K = 2', 'K = 3');
% axis([5 25 5 25]);
% 
% [U, L] = eig(S)
% alpha1 = atan2(U(2,1), U(1,1));
% p11 = [mu(1)-0.5*cos(alpha1), mu(2)-0.5*sin(alpha1)];
% p12 = [mu(1)+0.5*cos(alpha1), mu(2)+0.5*sin(alpha1)];
% 
% alpha2 = atan2(U(2,2), U(1,2));
% p21 = [mu(1)-0.5*cos(alpha2), mu(2)-0.5*sin(alpha2)];
% p22 = [mu(1)+0.5*cos(alpha2), mu(2)+0.5*sin(alpha2)];
% 
% arrow(p11,p12, 3);
% arrow(p21,p22, 3);
% 
% % ----- ----- ----- ----- -----
% 
% mu = [15;15];
% s1 = 3;
% s2 = 2;
% p = 5/6;
% s12 = p*s1*s2;
% S = [s1^2 s12; s12 s2^2];
% 
% [x1, x2] = meshgrid(mu(1)-10:0.1:mu(1)+10, mu(2)-10:0.1:mu(2)+10);
% y = (1/det(S)) * (S(2,2)*(x1 - mu(1)).^2 + S(1,1)*(x2 - mu(2)).^2 - 2*S(1,2)*(x1 - mu(1)).*(x2 - mu(2)));
% G = (1/((2*pi)*sqrt(det(S))))*exp(-0.5*y);
% 
% figure;
% subplot(121);
% meshc(x1, x2, y);
% %view([-55 9])
% xlabel('x_1');
% ylabel('x_2');
% title('$\left(\vec{x} - \vec{\mu}\right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} - \vec{\mu}\right)$', 'Interpreter', 'latex');
% grid on; 
% subplot(122);
% meshc(x1, x2, G);
% xlabel('x_1');
% ylabel('x_2');
% title('$p\left(\vec{x}\right) \, = \, \frac{1}{2\,\pi \, | \Sigma |^{1/2} } \, \cdot \, e^{-\frac{1}{2} \, \cdot \, \left(\vec{x} \, - \, \vec{\mu} \right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} \, - \, \vec{\mu}\right)}$', 'Interpreter', 'latex');
% grid on;
% 
% ellipses = ellipsedata(S, mu, numpoints, K);
% 
% figure;
% plot(ellipses(:, 1), ellipses(:, 2), 'r');
% hold on;
% plot(ellipses(:, 3), ellipses(:, 4), 'g');
% plot(ellipses(:, 5), ellipses(:, 6), 'b');
% grid on;
% xlabel('x_1');
% ylabel('x_2');
% title('$\left(\vec{x} - \vec{\mu}\right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} - \vec{\mu}\right) \, = \, K^2$', 'Interpreter', 'latex');
% legend('K = 1', 'K = 2', 'K = 3');
% axis([5 25 5 25]);
% 
% [U, L] = eig(S)
% alpha1 = atan2(U(2,1), U(1,1));
% p11 = [mu(1)-0.5*cos(alpha1), mu(2)-0.5*sin(alpha1)];
% p12 = [mu(1)+0.5*cos(alpha1), mu(2)+0.5*sin(alpha1)];
% 
% alpha2 = atan2(U(2,2), U(1,2));
% p21 = [mu(1)-0.5*cos(alpha2), mu(2)-0.5*sin(alpha2)];
% p22 = [mu(1)+0.5*cos(alpha2), mu(2)+0.5*sin(alpha2)];
% 
% arrow(p11,p12, 3);
% arrow(p21,p22, 3);
% 
% % ----- ----- ----- ----- -----
% 
% mu = [15;15];
% s1 = 3;
% s2 = 2;
% p = -5/6;
% s12 = p*s1*s2;
% S = [s1^2 s12; s12 s2^2];
% 
% [x1, x2] = meshgrid(mu(1)-10:0.1:mu(1)+10, mu(2)-10:0.1:mu(2)+10);
% y = (1/det(S)) * (S(2,2)*(x1 - mu(1)).^2 + S(1,1)*(x2 - mu(2)).^2 - 2*S(1,2)*(x1 - mu(1)).*(x2 - mu(2)));
% G = (1/((2*pi)*sqrt(det(S))))*exp(-0.5*y);
% 
% figure;
% subplot(121);
% meshc(x1, x2, y);
% %view([-55 9])
% xlabel('x_1');
% ylabel('x_2');
% title('$\left(\vec{x} - \vec{\mu}\right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} - \vec{\mu}\right)$', 'Interpreter', 'latex')
% grid on; 
% subplot(122);
% meshc(x1, x2, G);
% xlabel('x_1');
% ylabel('x_2');
% title('$p\left(\vec{x}\right) \, = \, \frac{1}{2\,\pi \, | \Sigma |^{1/2} } \, \cdot \, e^{-\frac{1}{2} \, \cdot \, \left(\vec{x} \, - \, \vec{\mu} \right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} \, - \, \vec{\mu}\right)}$', 'Interpreter', 'latex');
% grid on;
% 
% ellipses = ellipsedata(S, mu, numpoints, K);
% 
% figure;
% plot(ellipses(:, 1), ellipses(:, 2), 'r');
% hold on;
% plot(ellipses(:, 3), ellipses(:, 4), 'g');
% plot(ellipses(:, 5), ellipses(:, 6), 'b');
% grid on;
% xlabel('x_1');
% ylabel('x_2');
% title('$\left(\vec{x} - \vec{\mu}\right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} - \vec{\mu}\right) \, = \, K^2$', 'Interpreter', 'latex');
% legend('K = 1', 'K = 2', 'K = 3');
% axis([5 25 5 25]);
% 
% [U, L] = eig(S)
% alpha1 = atan2(U(2,1), U(1,1));
% p11 = [mu(1)-0.5*cos(alpha1), mu(2)-0.5*sin(alpha1)];
% p12 = [mu(1)+0.5*cos(alpha1), mu(2)+0.5*sin(alpha1)];
% 
% alpha2 = atan2(U(2,2), U(1,2));
% p21 = [mu(1)-0.5*cos(alpha2), mu(2)-0.5*sin(alpha2)];
% p22 = [mu(1)+0.5*cos(alpha2), mu(2)+0.5*sin(alpha2)];
% 
% arrow(p11,p12, 3);
% arrow(p21,p22, 3);
% 
% % ----- ----- ----- ----- -----
% 
% mu = [15;15];
% s1 = 3;
% s2 = 3;
% p = 5/9;
% s12 = p*s1*s2;
% S = [s1^2 s12; s12 s2^2];
% 
% [x1, x2] = meshgrid(mu(1)-10:0.1:mu(1)+10, mu(2)-10:0.1:mu(2)+10);
% y = (1/det(S)) * (S(2,2)*(x1 - mu(1)).^2 + S(1,1)*(x2 - mu(2)).^2 - 2*S(1,2)*(x1 - mu(1)).*(x2 - mu(2)));
% G = (1/((2*pi)*sqrt(det(S))))*exp(-0.5*y);
% 
% figure;
% subplot(121);
% meshc(x1, x2, y);
% view([-55 9])
% xlabel('x_1');
% ylabel('x_2');
% title('$\left(\vec{x} - \vec{\mu}\right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} - \vec{\mu}\right)$', 'Interpreter', 'latex');
% grid on; 
% subplot(122);
% meshc(x1, x2, G);
% xlabel('x_1');
% ylabel('x_2');
% title('$p\left(\vec{x}\right) \, = \, \frac{1}{2\,\pi \, | \Sigma |^{1/2} } \, \cdot \, e^{-\frac{1}{2} \, \cdot \, \left(\vec{x} \, - \, \vec{\mu} \right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} \, - \, \vec{\mu}\right)}$', 'Interpreter', 'latex');
% grid on;
% 
% ellipses = ellipsedata(S, mu, numpoints, K);
% 
% figure;
% plot(ellipses(:, 1), ellipses(:, 2), 'r');
% hold on;
% plot(ellipses(:, 3), ellipses(:, 4), 'g');
% plot(ellipses(:, 5), ellipses(:, 6), 'b');
% grid on;
% xlabel('x_1');
% ylabel('x_2');
% title('$\left(\vec{x} - \vec{\mu}\right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} - \vec{\mu}\right) \, = \, K^2$', 'Interpreter', 'latex');
% legend('K = 1', 'K = 2', 'K = 3');
% axis([5 25 5 25]);
% 
% [U, L] = eig(S)
% alpha1 = atan2(U(2,1), U(1,1));
% p11 = [mu(1)-0.5*cos(alpha1), mu(2)-0.5*sin(alpha1)];
% p12 = [mu(1)+0.5*cos(alpha1), mu(2)+0.5*sin(alpha1)];
% 
% alpha2 = atan2(U(2,2), U(1,2));
% p21 = [mu(1)-0.5*cos(alpha2), mu(2)-0.5*sin(alpha2)];
% p22 = [mu(1)+0.5*cos(alpha2), mu(2)+0.5*sin(alpha2)];
% 
% arrow(p11,p12, 3);
% arrow(p21,p22, 3);
% 
% % ----- ----- ----- ----- -----
% 
% mu = [15;15];
% s1 = 3;
% s2 = 3;
% p = -5/9;
% s12 = p*s1*s2;
% S = [s1^2 s12; s12 s2^2];
% 
% [x1, x2] = meshgrid(mu(1)-10:0.1:mu(1)+10, mu(2)-10:0.1:mu(2)+10);
% y = (1/det(S)) * (S(2,2)*(x1 - mu(1)).^2 + S(1,1)*(x2 - mu(2)).^2 - 2*S(1,2)*(x1 - mu(1)).*(x2 - mu(2)));
% G = (1/((2*pi)*sqrt(det(S))))*exp(-0.5*y);
% 
% figure;
% subplot(121);
% meshc(x1, x2, y);
% view([-55 9])
% xlabel('x_1');
% ylabel('x_2');
% title('$\left(\vec{x} - \vec{\mu}\right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} - \vec{\mu}\right)$', 'Interpreter', 'latex');
% grid on; 
% subplot(122);
% meshc(x1, x2, G);
% xlabel('x_1');
% ylabel('x_2');
% title('$p\left(\vec{x}\right) \, = \, \frac{1}{2\,\pi \, | \Sigma |^{1/2} } \, \cdot \, e^{-\frac{1}{2} \, \cdot \, \left(\vec{x} \, - \, \vec{\mu} \right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} \, - \, \vec{\mu}\right)}$', 'Interpreter', 'latex');
% grid on;
% 
% ellipses = ellipsedata(S, mu, numpoints, K);
% 
% figure;
% plot(ellipses(:, 1), ellipses(:, 2), 'r');
% hold on;
% plot(ellipses(:, 3), ellipses(:, 4), 'g');
% plot(ellipses(:, 5), ellipses(:, 6), 'b');
% grid on;
% xlabel('x_1');
% ylabel('x_2');
% title('$\left(\vec{x} - \vec{\mu}\right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} - \vec{\mu}\right) \, = \, K^2$', 'Interpreter', 'latex');
% legend('K = 1', 'K = 2', 'K = 3');
% axis([5 25 5 25]);
% 
% [U, L] = eig(S)
% alpha1 = atan2(U(2,1), U(1,1));
% p11 = [mu(1)-0.5*cos(alpha1), mu(2)-0.5*sin(alpha1)];
% p12 = [mu(1)+0.5*cos(alpha1), mu(2)+0.5*sin(alpha1)];
% 
% alpha2 = atan2(U(2,2), U(1,2));
% p21 = [mu(1)-0.5*cos(alpha2), mu(2)-0.5*sin(alpha2)];
% p22 = [mu(1)+0.5*cos(alpha2), mu(2)+0.5*sin(alpha2)];
% 
% arrow(p11,p12, 3);
% arrow(p21,p22, 3);
% 
% % ----- ----- ----- ----- -----
% 
% mu = [15; 15];
% s1 = 1;
% s2 = 3;
% p = 0;
% s12 = p*s1*s2;
% S = [s1^2 s12; s12 s2^2];
% 
% [x1, x2] = meshgrid(mu(1)-10:0.1:mu(1)+10, mu(2)-10:0.1:mu(2)+10);
% y = (1/det(S)) * (S(2,2)*(x1 - mu(1)).^2 + S(1,1)*(x2 - mu(2)).^2 - 2*S(1,2)*(x1 - mu(1)).*(x2 - mu(2)));
% G = (1/((2*pi)*sqrt(det(S))))*exp(-0.5*y);
% 
% figure;
% subplot(121);
% meshc(x1, x2, y);
% %view([-55 9])
% xlabel('x_1');
% ylabel('x_2');
% title('$\left(\vec{x} - \vec{\mu}\right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} - \vec{\mu}\right)$', 'Interpreter', 'latex')
% grid on; 
% subplot(122);
% meshc(x1, x2, G);
% xlabel('x_1');
% ylabel('x_2');
% title('$p\left(\vec{x}\right) \, = \, \frac{1}{2\,\pi \, | \Sigma |^{1/2} } \, \cdot \, e^{-\frac{1}{2} \, \cdot \, \left(\vec{x} \, - \, \vec{\mu} \right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} \, - \, \vec{\mu}\right)}$', 'Interpreter', 'latex');
% grid on;
% 
% ellipses = ellipsedata(S, mu, numpoints, K);
% 
% figure;
% plot(ellipses(:, 1), ellipses(:, 2), 'r');
% hold on;
% plot(ellipses(:, 3), ellipses(:, 4), 'g');
% plot(ellipses(:, 5), ellipses(:, 6), 'b');
% grid on;
% xlabel('x_1');
% ylabel('x_2');
% title('$\left(\vec{x} - \vec{\mu}\right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} - \vec{\mu}\right) \, = \, K^2$', 'Interpreter', 'latex');
% legend('K = 1', 'K = 2', 'K = 3');
% axis([5 25 5 25]);
% 
% [U, L] = eig(S)
% alpha1 = atan2(U(2,1), U(1,1));
% p11 = [mu(1)-0.5*cos(alpha1), mu(2)-0.5*sin(alpha1)];
% p12 = [mu(1)+0.5*cos(alpha1), mu(2)+0.5*sin(alpha1)];
% 
% alpha2 = atan2(U(2,2), U(1,2));
% p21 = [mu(1)-0.5*cos(alpha2), mu(2)-0.5*sin(alpha2)];
% p22 = [mu(1)+0.5*cos(alpha2), mu(2)+0.5*sin(alpha2)];
% 
% arrow(p11,p12, 3);
% arrow(p21,p22, 3);
% 
% % ----- ----- ----- ----- 
% 
% mu = [15; 15];
% s1 = 3;
% s2 = 1;
% p = 0;
% s12 = p*s1*s2;
% S = [s1^2 s12; s12 s2^2];
% 
% [x1, x2] = meshgrid(mu(1)-10:0.1:mu(1)+10, mu(2)-10:0.1:mu(2)+10);
% y = (1/det(S)) * (S(2,2)*(x1 - mu(1)).^2 + S(1,1)*(x2 - mu(2)).^2 - 2*S(1,2)*(x1 - mu(1)).*(x2 - mu(2)));
% G = (1/((2*pi)*sqrt(det(S))))*exp(-0.5*y);
% 
% figure;
% subplot(121);
% meshc(x1, x2, y);
% %view([-55 9])
% xlabel('x_1');
% ylabel('x_2');
% title('$\left(\vec{x} - \vec{\mu}\right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} - \vec{\mu}\right)$', 'Interpreter', 'latex')
% grid on; 
% subplot(122);
% meshc(x1, x2, G);
% xlabel('x_1');
% ylabel('x_2');
% title('$p\left(\vec{x}\right) \, = \, \frac{1}{2\,\pi \, | \Sigma |^{1/2} } \, \cdot \, e^{-\frac{1}{2} \, \cdot \, \left(\vec{x} \, - \, \vec{\mu} \right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} \, - \, \vec{\mu}\right)}$', 'Interpreter', 'latex');
% grid on;
% 
% ellipses = ellipsedata(S, mu, numpoints, K);
% 
% figure;
% plot(ellipses(:, 1), ellipses(:, 2), 'r');
% hold on;
% plot(ellipses(:, 3), ellipses(:, 4), 'g');
% plot(ellipses(:, 5), ellipses(:, 6), 'b');
% grid on;
% xlabel('x_1');
% ylabel('x_2');
% title('$\left(\vec{x} - \vec{\mu}\right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} - \vec{\mu}\right) \, = \, K^2$', 'Interpreter', 'latex');
% legend('K = 1', 'K = 2', 'K = 3');
% axis([5 25 5 25]);
% 
% [U, L] = eig(S)
% alpha1 = atan2(U(2,1), U(1,1));
% p11 = [mu(1)-0.5*cos(alpha1), mu(2)-0.5*sin(alpha1)];
% p12 = [mu(1)+0.5*cos(alpha1), mu(2)+0.5*sin(alpha1)];
% 
% alpha2 = atan2(U(2,2), U(1,2));
% p21 = [mu(1)-0.5*cos(alpha2), mu(2)-0.5*sin(alpha2)];
% p22 = [mu(1)+0.5*cos(alpha2), mu(2)+0.5*sin(alpha2)];
% 
% arrow(p11,p12, 3);
% arrow(p21,p22, 3);
% 
% % ----- ----- ----- ----- 
% 
% mu = [15; 15];
% s1 = 3;
% s2 = 3;
% p = 0;
% s12 = p*s1*s2;
% S = [s1^2 s12; s12 s2^2];
% 
% [x1, x2] = meshgrid(mu(1)-10:0.1:mu(1)+10, mu(2)-10:0.1:mu(2)+10);
% y = (1/det(S)) * (S(2,2)*(x1 - mu(1)).^2 + S(1,1)*(x2 - mu(2)).^2 - 2*S(1,2)*(x1 - mu(1)).*(x2 - mu(2)));
% G = (1/((2*pi)*sqrt(det(S))))*exp(-0.5*y);
% 
% figure;
% subplot(121);
% meshc(x1, x2, y);
% %view([-55 9])
% xlabel('x_1');
% ylabel('x_2');
% title('$\left(\vec{x} - \vec{\mu}\right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} - \vec{\mu}\right)$', 'Interpreter', 'latex')
% grid on; 
% subplot(122);
% meshc(x1, x2, G);
% xlabel('x_1');
% ylabel('x_2');
% title('$p\left(\vec{x}\right) \, = \, \frac{1}{2\,\pi \, | \Sigma |^{1/2} } \, \cdot \, e^{-\frac{1}{2} \, \cdot \, \left(\vec{x} \, - \, \vec{\mu} \right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} \, - \, \vec{\mu}\right)}$', 'Interpreter', 'latex');
% grid on;
% 
% ellipses = ellipsedata(S, mu, numpoints, K);
% 
% figure;
% plot(ellipses(:, 1), ellipses(:, 2), 'r');
% hold on;
% plot(ellipses(:, 3), ellipses(:, 4), 'g');
% plot(ellipses(:, 5), ellipses(:, 6), 'b');
% grid on;
% xlabel('x_1');
% ylabel('x_2');
% title('$\left(\vec{x} - \vec{\mu}\right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} - \vec{\mu}\right) \, = \, K^2$', 'Interpreter', 'latex');
% legend('K = 1', 'K = 2', 'K = 3');
% axis([5 25 5 25]);
% 
% [U, L] = eig(S)
% alpha1 = atan2(U(2,1), U(1,1));
% p11 = [mu(1)-0.5*cos(alpha1), mu(2)-0.5*sin(alpha1)];
% p12 = [mu(1)+0.5*cos(alpha1), mu(2)+0.5*sin(alpha1)];
% 
% alpha2 = atan2(U(2,2), U(1,2));
% p21 = [mu(1)-0.5*cos(alpha2), mu(2)-0.5*sin(alpha2)];
% p22 = [mu(1)+0.5*cos(alpha2), mu(2)+0.5*sin(alpha2)];
% 
% arrow(p11,p12, 3);
% arrow(p21,p22, 3);

% ----- ----- ----- ----- 

mu = [0; 0];
s1 = 1;
s2 = 2;
p = 1;
s12 = p*s1*s2;
S = [s1^2 s12; s12 s2^2];

lower_limit = -1;
upper_limit = +1;

[x1, x2] = meshgrid(mu(1)+lower_limit:0.01:mu(1)+upper_limit, mu(2)+2*lower_limit:0.01:mu(2)+2*upper_limit);
y = (1/det(S)) * (S(2,2)*(x1 - mu(1)).^2 + S(1,1)*(x2 - mu(2)).^2 - 2*S(1,2)*(x1 - mu(1)).*(x2 - mu(2)));
G = (1/((2*pi)*sqrt(det(S))))*exp(-0.5*y);

figure;
subplot(121);
meshc(x1, x2, y);
%view([-55 9])
xlabel('x_1');
ylabel('x_2');
title('$\left(\vec{x} - \vec{\mu}\right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} - \vec{\mu}\right)$', 'Interpreter', 'latex')
grid on; 
subplot(122);
meshc(x1, x2, G);
xlabel('x_1');
ylabel('x_2');
title('$p\left(\vec{x}\right) \, = \, \frac{1}{2\,\pi \, | \Sigma |^{1/2} } \, \cdot \, e^{-\frac{1}{2} \, \cdot \, \left(\vec{x} \, - \, \vec{\mu} \right)^T \, \cdot \, \Sigma^{-1} \, \cdot \, \left(\vec{x} \, - \, \vec{\mu}\right)}$', 'Interpreter', 'latex');
grid on;

ellipses = ellipsedata(S, mu, numpoints, 1);

figure;
plot(ellipses(:, 1), ellipses(:, 2), 'r');
hold on;
grid on;
xlabel('x_1');
ylabel('x_2');
axis([-1.5 1.5 -3 3]);

[U, L] = eig(S)
alpha1 = atan2(U(2,1), U(1,1));
p11 = [mu(1)-0.5*cos(alpha1), mu(2)-0.5*sin(alpha1)];
p12 = [mu(1)+0.5*cos(alpha1), mu(2)+0.5*sin(alpha1)];

alpha2 = atan2(U(2,2), U(1,2));
p21 = [mu(1)-0.5*cos(alpha2), mu(2)-0.5*sin(alpha2)];
p22 = [mu(1)+0.5*cos(alpha2), mu(2)+0.5*sin(alpha2)];

arrow(p11,p12, 3);
arrow(p21,p22, 3);

