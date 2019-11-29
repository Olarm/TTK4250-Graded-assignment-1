% IMM-PDA
% sensor 
r = 1;%...
lambda = 0.04;%...
PD = 0.98;%...
gateSize = 18;%...

% dynamic models
qCV = 0.2;%...
qCT = [0.08, 0.000001];%...
x0 = [6, 25, 0, 0, 0]';%...
P0 = diag([25, 25, 3, 3, 0.0005].^2);%...

% markov chain (you are free to parametrize this in another way)
PI11 = 0.80;%...
PI22 = 0.65;%... 
p10 = 0.80;%... 

PI = [PI11, (1 - PI22); (1 - PI11), PI22]; assert(all(sum(PI, 1) == [1, 1]),'columns of PI must sum to 1')
sprobs0 = [p10; (1 - p10)]; assert(sum(sprobs0) == 1, 'initial mode probabilities must sum to 1');

% make model
models =  cell(2,1);
models{1} = EKF(discreteCVmodel(qCV, r));
models{2} = EKF(discreteCTmodel(qCT, r));
imm = IMM(models, PI);
tracker = IMMPDAF(imm, lambda, PD, gateSize);

% allocate
xbar = zeros(5, 2, K);
Pbar = zeros(5, 5, 2, K);
probbar = zeros(2, K);
xhat = zeros(5, 2, K);
xest = zeros(5, K);
Pest = zeros(5, 5, K);
Phat = zeros(5, 5, 2, K);
probhat = zeros(2, K);
NEES = zeros(K, 1);
NEESpos = zeros(K, 1);
NEESvel = zeros(K, 1);


% initialize
xbar(:, :, 1) = repmat(x0, [1, 2]);
Pbar(:, : ,:, 1) = repmat(P0,[1,1,2]);
probbar(:, 1) = sprobs0;

% filter
for k=1:100
    [probhat(:, k), xhat(:, :, k), Phat(:, :, :, k)] = tracker.update(cell2mat(Z(k,:)), probbar(:,k), xbar(:,:,k), Pbar(:,:,:,k));%... update
    [xest(:, k), Pest(:, :, k)] = reduceGaussMix(probbar(:, k), xbar(:, :, k), Pbar(:, :, :, k));%... total state mean and cov
    NEES(k) = ((xhat(1:4,k)-xbar(1:4,k))')/Pbar(1:4,1:4,k)*(xhat(1:4,k) - xbar(1:4,k));%... 
    NEESpos(k) = ((xhat(1:2,k)-xbar(1:2,k))')*inv(Pbar(1:2,1:2,k))*(xhat(1:2,k) - xbar(1:2,k));%...
    NEESvel(k) = ((xhat(3:4,k)-xbar(3:4,k))')*inv(Pbar(3:4,3:4,k))*(xhat(3:4,k) - xbar(3:4,k));%...
    if k < 100
        [probbar(:, k+1), xbar(:, :, k+1), Pbar(:, :, :, k+1)] = tracker.predict(probhat(:,k), xhat(:,:,k), Phat(:,:,:,k), Ts);%... predict
    end
end

% errors
poserr = sqrt(sum((xest(1:2,:) - Xgt(1:2,:)).^2, 1));
posRMSE = sqrt(mean(poserr.^2));

 % not true RMSE (which is over monte carlo simulations)
velerr = sqrt(sum((xest(3:4, :) - Xgt(3:4, :)).^2, 1));
velRMSE = sqrt(mean(velerr.^2)); % not true RMSE (which is over monte carlo simulations)
peakPosDeviation = max(poserr);
peakVelDeviation = max(velerr);

% consistency
CI2K = chi2inv([0.025, 0.975], K*2)/K;
ANEESpos = mean(NEESpos);
ANEESvel = mean(NEESvel);

sprintf('%f  <  %f,  %f  <  %f', CI2K(1), ANEESpos, ANEESvel, CI2K(2))

CI4K = chi2inv([0.025, 0.975], K*4)/K;
ANEES = mean(NEES);

sprintf('%f  <  %f  <  %f', CI4K(1), ANEES, CI4K(2)) 

% plot
figure(6); clf; hold on; grid on;
plot(xest(1,:), xest(2,:));
plot(Xgt(1,:), Xgt(2, :));
axis('equal')
title(sprintf('posRMSE = %.3f, velRMSE = %.3f, peakPosDev = %.3f, peakVelDev = %.3f',posRMSE, velRMSE, peakPosDeviation, peakVelDeviation))

figure(7); clf; hold on; grid on;
plot(xest(5,:))
plot(Xgt(5,:))

figure(8); clf;
plot(probhat');
grid on;

figure(9); clf;
subplot(2,1,1); 
plot(poserr); grid on;
ylabel('position error')
subplot(2,1,2);
plot(velerr); grid on;
ylabel('velocity error')

figure(10); clf;
subplot(3,1,1);
plot(NEES); grid on; hold on;
ylabel('NEES');
ciNEES = chi2inv([0.05, 0.95], 4);
inCI = sum((NEES >= ciNEES(1)) .* (NEES <= ciNEES(2)))/K * 100;
plot([1,K], repmat(ciNEES',[1,2])','r--')
text(104, -5, sprintf('%.2f%% inside CI', inCI),'Rotation',90);

subplot(3,1,2);
plot(NEESpos); grid on; hold on;
ylabel('NEESpos');
ciNEES = chi2inv([0.05, 0.95], 2);
inCI = sum((NEESpos >= ciNEES(1)) .* (NEESpos <= ciNEES(2)))/K * 100;
plot([1,K], repmat(ciNEES',[1,2])','r--')
text(104, -5, sprintf('%.2f%% inside CI', inCI),'Rotation',90);

subplot(3,1,3);
plot(NEESvel); grid on; hold on;
ylabel('NEESvel');
ciNEES = chi2inv([0.05, 0.95], 2);
inCI = sum((NEESvel >= ciNEES(1)) .* (NEESvel <= ciNEES(2)))/K * 100;
plot([1,K], repmat(ciNEES',[1,2])','r--')
text(104, -5, sprintf('%.2f%% inside CI', inCI),'Rotation',90);