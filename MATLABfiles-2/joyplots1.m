%% plot
figure(1); clf; hold on; grid on;
plot(xest(1,:), xest(2,:));
plot(Xgt(1,:), Xgt(2, :));
scatter(Zmat(1,:), Zmat(2,:))
axis('equal')
title(sprintf('posRMSE = %.3f, velRMSE = %.3f, peakPosDev = %.3f, peakVelDev = %.3f',posRMSE, velRMSE, peakPosDeviation, peakVelDeviation))

figure(2); clf; hold on; grid on;
subplot(3,1,1);
hold on; grid on;
plot(sqrt(sum(xest(3:4,:).^2,1)))
plot(sqrt(sum(Xgt(3:4,:).^2,1)))
ylabel('speed')
subplot(3,1,2);
hold on; grid on;
plot(atan2(xest(4,:), xest(3,:)))
plot(atan2(Xgt(4,:), Xgt(3,:)))
ylabel('theta')
subplot(3,1,3)
hold on; grid on;
plot(diff(unwrap(atan2(xest(4,:), xest(3,:))))./Ts')
plot(diff(unwrap(atan2(Xgt(4,:), Xgt(3,:))))./Ts')
ylabel('omega')

figure(3); clf;
plot(probhat');
grid on;

figure(4); clf;
subplot(2,1,1); 
plot(poserr); grid on;
ylabel('position error')
subplot(2,1,2);
plot(velerr); grid on;
ylabel('velocity error')

figure(5); clf;
subplot(3,1,1);
plot(NEES); grid on; hold on;
ylabel('NEES');
ciNEES = chi2inv([0.05, 0.95], 4);
inCI = sum((NEES >= ciNEES(1)) .* (NEES <= ciNEES(2)))/K * 100;
plot([1,K], repmat(ciNEES',[1,2])','r--')
text(K*1.04, -5, sprintf('%.2f%% inside CI', inCI),'Rotation',90);

subplot(3,1,2);
plot(NEESpos); grid on; hold on;
ylabel('NEESpos');
ciNEES = chi2inv([0.05, 0.95], 2);
inCI = sum((NEESpos >= ciNEES(1)) .* (NEESpos <= ciNEES(2)))/K * 100;
plot([1,K], repmat(ciNEES',[1,2])','r--')
text(K*1.04, -5, sprintf('%.2f%% inside CI', inCI),'Rotation',90);

subplot(3,1,3);
plot(NEESvel); grid on; hold on;
ylabel('NEESvel');
ciNEES = chi2inv([0.05, 0.95], 2);
inCI = sum((NEESvel >= ciNEES(1)) .* (NEESvel <= ciNEES(2)))/K * 100;
plot([1,K], repmat(ciNEES',[1,2])','r--')
text(K*1.04, -5, sprintf('%.2f%% inside CI', inCI),'Rotation',90);