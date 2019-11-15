classdef ESKF
    properties
        % accelerometer parameters
        qA
        qAb
        pAcc
        Sa
        
        % gyro parameters
        qG
        qGb
        pGyro
        Sg = eye(3);
        
        % continuous time covariance matrix
        Qerr
        
        % other constants
        g % gravity vector (positive down)
    end
    methods
        function obj = ESKF(qA, qG, qAb, qGb, pAcc, pGyro)
            % initializes the ESKF object
            %
            % qA (scalar): IMU acceleration noise
            % qG (scalar): IMU gyro noise
            % qAb (scalar): acceleration bias driving noise
            % qGb (scalar): gyro bias driving noise
            % pAcc (scalar): acceleration bias reciprocal time constant
            % pGyro (scalar): gyro bias reciprocal time constant
            
            obj.qA = qA;
            obj.qAb = qAb;
            
            obj.qG = qG;
            obj.qGb = qGb;
            
            obj.pAcc = pAcc;
            obj.pGyro = pGyro;
            
            obj.Qerr = blkdiag(obj.qA * eye(3), obj.qG * eye(3), obj.qAb * eye(3), obj.qGb * eye(3));
            
            % needed constants
            obj.g = [0; 0; 9.82]; % higher gravity around here.
            
            obj.Sa = eye(3); % accelerometer correction matrix
            obj.Sg = eye(3); % gyro corretion matrix
        end 
        
        function xnompred = predictNominal(obj, xnom, acc, omega, Ts)
            % predicts the nominal state
            %
            % xnom (16 x 1):nominal state (pos, vel ,quat, accbias, gyrobias)
            % acc (3 x 1): debiased and rectified acceleration measurement
            % omega (3 x 1): debiased and rectified rotation rate measurement
            % Ts (scalar): sampling time
            %
            % xnompred(16 x 1): predicted nominal state
            
            % extract states
            pos = xnom(1:3);
            vel = xnom(4:6);
            quat = xnom(7:10);
            accBias = xnom(11:13);
            gyroBias = xnom(14:16);
            
            % get rotation matrix from quaternion
            R = quat2rotmat(quat);
            
            % predictions
            posPred = pos + Ts * vel + Ts^2/2 * (R * acc + obj.g);%  obj.g skal vere med?
            velPred = vel + Ts * (R * acc + obj.g);%
            
            dq = Ts * omega;
            quatPred = quatProd(quat, euler2quat(dq));
            
            accBiasPred = -(1/obj.pAcc)*eye(3)*accBias;%  is eye needed?
            gyroBiasPred = -(1/obj.pGyro)*eye(3)*gyroBias;%
            
            % make sure quaternion is normalized
            quatPred = quatPred/norm(quatPred);% 
            
            % concatenate into the predicted nominal state
            xnompred = [posPred;
                        velPred;
                        quatPred;
                        accBiasPred;
                        gyroBiasPred];
        end
        
        function A = Aerr(obj, xnom, acc, omega)
            % creates the continuous time error state system matrix
            %
            % xnom (16 x 1): nominal state
            % acc (3 x 1): debiased and rectified acceleration measurement
            % omega (3 x 1): debiased and rectified gyro measurement
            %
            % A (15 x 15): continuous time error state system matrix
            
            % get the rotation matrix
            R = quat2rotmat(xnom(7:10));
            
            % allocate the matrix
            A = zeros(15, 15);
            
            I3 = eye(3);
            
            % instert the different terms
            A(1:3, 4:6) = I3;%...; % vel to pos
            A(4:6, 7:9) = -R*crossProdMat(acc); %...; % attitude to vel
            A(4:6, 10:12) = -R; %...; % acc bias to vel
            A(7:9, 7:9) = -crossProdMat(omega); %...; % attitude to attitude
            A(7:9, 13:15) = -I3; %...; % gyro bias to attitude
            A(10:12, 10:12) = -(1/obj.pAcc)*I3; %...; % acc bias to acc bias
            A(13:15, 13:15) = -(1/obj.pGyro)*I3; %...; % gyro bias to gyro bias
            
            % bias corrections
            A(4:6, 10:12) = A(4:6, 10:12) * obj.Sa;
            A(7:9, 13:15) = A(7:9, 13:15) * obj.Sg;
        end
        
        function G = Gerr(~, xnom)
            % creates the continous time error state noise input matrix
            % 
            % xnom (16 x 1): nominal state
            %
            % G (15 x 12): continous time error state noise input matrix
            
            % get rotation matrix
            Rot = quat2rotmat(xnom(7:10));
            
            I = eye(3);
            
            % create the input matrix
            G = vertcat(zeros(3, 12), blkdiag(-Rot, -I, I, I));% 
        end
        
        function [Ad, GQGd] = discreteErrMats(obj, xnom, acc, omega, Ts)
            % creates the discrete time error state system and covariance
            % matrices
            %
            % xnom (16 x 1): nominal state
            % acc (3 x 1): debiased and rectified acceleration measurement
            % omega (3 x 1): debiased and rectified rotation rate measurement
            % Ts (scalar): sampling time
            % 
            % Ad (15 x 15): discrete time error state system matrix
            % GQGd (15 x 15): discrete time noise covariance matrix
            
            % get continuous time matrices
            A = obj.Aerr(xnom, acc, omega);
            G = obj.Gerr(xnom);
            
            zero = zeros(size(A));
            
            % use Van Loan
            V = Ts * [-A G * obj.Qerr * G';
                       zero           A'  ];%; % the matrix exponent in Van Loan
            VanLoanMat = expm(V); % can potentially be slow
             
            % exctract relevat matrices.
            Ad = (VanLoanMat(16:30, 16:30))';
            GQGd = Ad*VanLoanMat(1:15, 16:30);    
        end
        
        function Ppred = predictCovariance(obj, xnom, P, acc, omega, Ts)
            % predicts the error state covariance
            %
            % xnom (16 x 1): nominal state
            % P (15 x 15): error state covariance
            % acc (3 x 1): debiased and rectified acceleration measurement
            % omega (3 x 1): debiased and rectified rotation rate measurement
            % Ts (scalar): sampling time
            %
            % Ppred (15 x 15): predicted error state covariance
            
            % get discrete time system matrices
            [Ad, GQGd] = obj.discreteErrMats(xnom, acc, omega, Ts);
            
            % KF covariance predict
            Ppred = Ad*P*Ad' + GQGd;
            
        end
        
        function [xnompred, Ppred] = predict(obj, xnom, P, zAcc, zGyro, Ts)
            % predicts the nominal state and error state covariance
            % 
            % xnom (16 x 1): nominal state
            % P (15 x 15): error state covariance
            % zAcc (3 x 1): the measured acceleration from IMU
            % zGyro (3 x 1): the measured rotation rate from IMU
            % Ts (scalar): sampling time
            %
            % xnompred (16 x 1): predicted nominal state
            % Ppred (15 x 15): predicted error state covariance
            
            % rectify the measurements
            zAcc = obj.Sa * zAcc;
            zGyro = obj.Sg * zGyro;
            
            % extract biases and rectify
            accBias = obj.Sa * xnom(11:13);
            gyroBias = obj.Sg * xnom(14:16);

            % debias measurements
            acc = zAcc - accBias; % expected value of accelleration in body given IMU measurements
            omega = zGyro - gyroBias; % expected value of rotation rate in body given IMU measurements
            
            % perform prediction using the above functions
            xnompred = obj.predictNominal(xnom, acc, omega, Ts);
            Ppred = obj.predictCovariance(xnom, P, acc, omega, Ts);
        end
        
        function [xinjected, Pinjected] = inject(~, xnom, deltaX, P)
            % performs the injection step of the ESKF (error state resett)
            %
            % xnom (16 x 1): nominal state
            % deltaX (15 x 1): estimated (updated from a measurement) error state
            %
            % xinjected (16 x 1): nominal state after injection
            % Pinjected (15 x 15): error state covariance after injection
            
            % Inject error state into nominal state (quaternions cannot be added)
            xinjected([1:6 11:16]) = xnom([1:6 11:16]) + deltaX([1:6 10:15]);
            xinjected(7:10) = quatProd(xnom(7:10), euler2quat(deltaX(7:9))); 
            
            % make sure quaterion is normalized
            xinjected(7:10) = xinjected(7:10)/norm(xinjected(7:10));
    
            % compensate for injection in the covariance
            S = crossProdMat(deltaX(7:9));
            Ginject = blkdiag(eye(6), (eye(3) - S), eye(6));
            Pinjected = Ginject*P*Ginject';
        end
        
        function [v, S] = innovationGNSS(~, xnom, P, zGNSSpos, RGNSS, leverarm)
            % Calculates the innovation and its covariance for a GNSS
            % position measurement.
            %
            % xnom (16 x 1): nominal state
            % P (15 x 15): error state covariance
            % zGNSSpos (3 x 1): GNSS position measurement
            % RGNSS (3 x 3): GNSS position noise covariance
            % leverarm (3 x 1) [optional]: the position of the GNSS antenna
            % 
            % v (3 x 1): innovation
            % S (3 x 3): innovation covariance
            
            H = horzcat(eye(3), zeros(3, 12)); 
            % innovation calculation
            v = zGNSSpos - H*[xnom(1:6); quat2eul(xnom(7:10)); xnom(11:16)]; % innovation
            
            % in case of a specified lever arm
            if nargin > 5
                R = quat2rotmat(xnom(7:10));
                H(:, 7:9) = -R * crossProdMat(leverarm);
                v = v - R * leverarm;
            end
            
            S = H*P*H' + RGNSS; % Innovation covariance
        end
        
        function [xinjected, Pinjected] = updateGNSS(obj, xnom, P, zGNSSpos, RGNSS, leverarm)
            % Updates the the state and covariance from a GNSS position measurement
            %
            % xnom (16 x 1): nominal state
            % P (15 x 15): error state covariance
            % zGNSSpos (3 x 1): GNSS position measurement
            % RGNSS (3 x 3): GNSS position noise covariance
            % leverarm (3 x 1) [optional]: the position of the GNSS antenna
            % 
            % xinjected (16 x 1): nominal state after injection of updated 
            %                       error state
            % Preset (15 x 15): error state covariance after error state
            %                   update and injection
            
            if nargin < 6
                leverarm = zeros(3, 1);
            end
            
            I = eye(size(P));
            
            [innov, S] = obj.innovationGNSS(xnom, P, zGNSSpos, RGNSS, leverarm);
            % measurement matrix
            H = horzcat(eye(3), zeros(3, 12));  
            
            % in case of a specified lever arm
            if nargin > 5
                R = quat2rotmat(xnom(7:10));
                H(:, 7:9) = - R * crossProdMat(leverarm);
            end
            
            % KF error state update
            W = P * H' / S; % Kalman gain
            deltaX = W * innov; 
            Pupd = (eye(15) - W * H) * P; 
            
            % error state injection
            [xinjected, Pinjected] = obj.inject(xnom, deltaX, Pupd); 
        end
        
        
        function NIS = NISGNSS(obj, xnom, P, zGNSSpos, RGNSSpos, leverarm)
            % Calculates the NIS for a GNSS position measurement
            %
            % xnom (16 x 1): nominal state
            % P (15 x 15): error state covariance
            % zGNSSpos (3 x 1): GNSS position measurement
            % RGNSS (3 x 3): GNSS position noise covariance
            % leverarm (3 x 1) [optional]: the position of the GNSS antenna
            % 
            % NIS (scalar): (z - pos(x))' * S^(-1) (z - pos(x))
            if nargin < 6
                leverarm = zeros(3,1);
            end
            [innov, S] = obj.innovationGNSS(xnom, P, zGNSSpos, RGNSSpos, leverarm);
            NIS = innov' / S * innov;
        end
        
        function deltaX = deltaX(~, xnom, xtrue)
            % calculates the error state between xnom and xtrue
            % 
            % xnom (16 x 1): nominal state
            % xtrue (16 x 1): true state
            %
            % deltaX (15 x 1): the errors such that deltaX = xtue "-" xnom
            %                   ("-" due to the attitude not being a true
            %                   addition)
           
           % pos and vel
           deltaPos = xtrue(1:3) - xnom(1:3);
           deltaVel = xtrue(4:6) - xnom(4:6);
           
            % attitude (just some suggested steps, you are free to change)
           qConj = conj(xnom(7:10)); % conjugated nominal quaternion
           deltaQuat = quatProd(qConj, xtrue(7:10)); % the error quaternion
           deltaTheta = deltaQuat(2:4); % the error state
           
           deltaBias = xtrue(11:16) - xnom(11:16);
           
           deltaX = [deltaPos; deltaVel; deltaTheta; deltaBias];
        end
        
        function [NEES, NEESpos, NEESvel, NEESatt, NEESaccbias, NEESgyrobias] ...
                = NEES(obj, xnom, P, xtrue)
            % Calculates the total NEES and the NEES for the sub states
            %
            % xnom (16 x 1): nominal (estimated) state
            % P (15 x 15): error state covariance
            % xtrue (16, 1): the true state
            %
            % NEESxxx (scalar): NEES of the xxx sub error state
            
            deltaX = obj.deltaX(xnom, xtrue);
            
            NEES = deltaX' / P * deltaX;
            NEESpos = deltaX(1:3)'*(P(1:3, 1:3)\deltaX(1:3));
            NEESvel = deltaX(4:6)'*(P(4:6, 4:6)\deltaX(4:6));
            NEESatt = deltaX(7:9)'*(P(7:9, 7:9)\deltaX(7:9));
            NEESaccbias = deltaX(10:12)'*(P(10:12, 10:12)\deltaX(10:12));
            NEESgyrobias = deltaX(13:15)'*(P(13:15, 13:15)\deltaX(13:15));
        end
    end
end