#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  //std_a_ = 0.75;

  // Process noise standard deviation yaw acceleration in rad/s^2
  //std_yawdd_ = 0.01;
  
  //requirement: .9, .10, .40, .20
  //std_a_ = 0.1; // .13, .12, .61, .24 
  //std_yawdd_ = 0.5; 
  //std_a_ = 0.4; // .10, .09, .61, .23
  //std_yawdd_ = 0.4; 
  //std_a_ = 0.4; // .12, .10, .63, .26
  //std_yawdd_ = 0.2; 
  //std_a_ = 0.8; // .10, .09, .60, .24
  //std_yawdd_ = 0.8; 
  //std_a_ = 1.6; // .10, .10, .62, .29
  //std_yawdd_ = 1.6; 
  //std_a_ = 1.0; // .10, .09, .61, .25
  //std_yawdd_ = 1.0; 
  //std_a_ = 1.1; // .10, .09, .60, .24
  //std_yawdd_ = 0.5; 
  //std_a_ = 1.2; // .10, .09, .60, .25
  //std_yawdd_ = 0.9; 
  //std_a_ = 2.2; // .11, .10, .63, .31
  //std_yawdd_ = 1.9; 
  //std_a_ = 3.2; // .11, .10, .65, .36
  //std_yawdd_ = 2.9; 
  //std_a_ = 0.75; // .29, .20, .82, .48
  //std_yawdd_ = 0.05; 
  //std_a_ = 0.75; // .10, .09, .65, .27
  //std_yawdd_ = 1.5;  // crashed towards end 
  //std_a_ = 1.1; // .10, .09, .60, .24
  //std_yawdd_ = 0.7;
  //std_a_ = 5; // .12, .10, .48, .26
  //std_yawdd_ = 0.75; 
  //std_a_ = 5; // .12, .10, .52, .34
  //std_yawdd_ = 2.5; 
  //std_a_ = 3.0; // .15, .09, .56, .23
  //std_yawdd_ = 1.0; 
  //std_a_ = 7.0; // .12, .10, .78, .30
  //std_yawdd_ = 0.8; 
  //std_a_ = 1.5; // .18, .09, .86, .21
  //std_yawdd_ = 1.0; 
  //std_a_ = 5; // .21, .09, .67, .22
  //std_yawdd_ = 2.5; 
  //std_a_ = 5; // .12, .10, .52, .34
  //std_yawdd_ = 2.5; 
  //std_a_ = 1.2; // .07, .09, .37, .33
  //std_yawdd_ = 0.9; 
  //std_a_ = 5.0; // .07, .09, .37, .33
  //std_yawdd_ = 0.7; 
  std_a_ = 3.0; // .07, .09, .37, .33
  std_yawdd_ = 1.2; 
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  is_initialized_ = false;
  
  n_x_ = 5;
  
  n_aug_= n_x_ + 2;
  
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);   /// ??? should this be n_aug_,n_aug_ ???
  
  lambda_ = 3 - n_aug_;   /// ??? should this be n_aug_ instead of n_x_ ???
  
  weights_ = VectorXd(2*n_aug_+1);
  weights_.fill(1/(2*(lambda_+n_aug_)));
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  epsilon = VectorXd(500);
  eps_size = 0;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  float px, py;
  if (!is_initialized_) {
	  //if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
	  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		  float rho = meas_package.raw_measurements_(0);
		  float phi = meas_package.raw_measurements_(1);
		  float rhd = meas_package.raw_measurements_(2);
		  px = rho*cos(phi);
		  py = rho*sin(phi);
		  //is_initialized_ = true;
		  //cout << "done radar initialization processMeasurement(113)\n";
	  }
	  //else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
	  else {
		  px = meas_package.raw_measurements_(0);
		  py = meas_package.raw_measurements_(1);
		  //is_initialized_ = true;
		  //cout << "done laser initialization processMeasurement(151)\n";
	  }
	  x_ << px, py, 0, 0, 0;
	  P_ << 1, 0, 0, 0, 0,
			0, 1, 0, 0, 0,
			0, 0, 0, 0, 0,
			0, 0, 0, 0, 0,
			0, 0, 0, 0, 0;
			
	  is_initialized_ = true;
	  cout << "done initialization processMeasurement(161)\n";
	  time_us_ = meas_package.timestamp_ ;
      cout << "x_      =\n" << x_ << endl;
      cout << "P_      =\n" << P_ << endl;
	  cout << "lambda_ =\n" << lambda_ << endl;
      //cout << "weights_=\n" << weights_ << endl;
	  return;
  }

  // Predict
  float delta_time = (meas_package.timestamp_ - time_us_) / 1000000.0; //dt - expressed in seconds
  cout << "mesa time|previous time|delta time=" << meas_package.timestamp_ <<"|"<< time_us_ <<"|"<< delta_time << endl;
  Prediction(delta_time);

  // Update
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
	  UpdateRadar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_){
	  //UpdateLidar(meas_package);
	  UpdateLidarUKF(meas_package);
  }
  
  // update previous time stamp
  time_us_ = meas_package.timestamp_ ;
  
  return;
}


/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  // Calculate X sigma points to start
  MatrixXd Xsig(n_x_, 2*n_x_+1);
  cout << "inside prediction(206)\n";
  //cout << "delta_t=" << delta_t << endl;
  Xsig.col(0) = x_;
  //cout << "Xsig.col(0)=\n" << Xsig.col(0) << endl;
  MatrixXd A = P_.llt().matrixL();
  cout << "A=\n" << A << endl;
  for (int i=0; i<n_x_; i++){
	  Xsig.col(i+1)      = x_ + sqrt(lambda_+n_x_)*A.col(i);
	  /* a mistake of normalizing angle at sigma point generations. notice that the first angle gets
	     normalized started at 1 instead of 0. So ends up some angles get normalized while some doesn't.
		 Leaving this here as commented code for example of bad normalization practice.
	  // normalized angle
	  while(Xsig(3,i+1) >  M_PI) { Xsig(3,i+1) -= 2*M_PI; }
	  while(Xsig(3,i+1) < -M_PI) { Xsig(3,i+1) += 2*M_PI; }*/
	  
	  Xsig.col(i+n_x_+1) = x_ - sqrt(lambda_+n_x_)*A.col(i);
	  /*// normalized angle
	  while(Xsig(3,i+n_x_+1) >  M_PI) { Xsig(3,i+n_x_+1) -= 2*M_PI; }
	  while(Xsig(3,i+n_x_+1) < -M_PI) { Xsig(3,i+n_x_+1) += 2*M_PI; }*/
  }
  //cout << "Xsig=\n" << Xsig << endl;

  // Prepare x augmented by adding noise to x vector, then calculate X augmented sigma points
  VectorXd x_aug(n_aug_);   // 7
  x_aug.head(n_x_) = x_;    // fill the first 5 element with x
  x_aug(5) = 0;
  x_aug(6) = 0;
  MatrixXd P_aug(n_aug_, n_aug_);  // P_aug size is 7x7
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) << P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;
  MatrixXd Xsig_aug(n_aug_, 2*n_aug_+1);  //Xsig_aug size is 7x15
  MatrixXd A_aug = P_aug.llt().matrixL(); // P_aug transpose
  Xsig_aug.col(0) = x_aug;
  for (int i=0; i<n_aug_; i++){
	  Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_+n_aug_)*A_aug.col(i);
	  /* a mistake of normalizing angle at sigma point generations. notice that the first angle gets
	     normalized started at 1 instead of 0. So ends up some angles get normalized while some doesn't.
		 Leaving this here as commented code for example of bad normalization practice.
	  // normalized angle
	  while(Xsig_aug(3,i+1) >  M_PI) { Xsig_aug(3,i+1) -= 2*M_PI; }
	  while(Xsig_aug(3,i+1) < -M_PI) { Xsig_aug(3,i+1) += 2*M_PI; }*/
	  
	  Xsig_aug.col(i+n_aug_+1) = x_aug - sqrt(lambda_+n_aug_)*A_aug.col(i);
	  /*// normalized angle
	  while(Xsig_aug(3,i+n_aug_+1) >  M_PI) { Xsig_aug(3,i+n_aug_+1) -= 2*M_PI; }
	  while(Xsig_aug(3,i+n_aug_+1) < -M_PI) { Xsig_aug(3,i+n_aug_+1) += 2*M_PI; }*/
  }
  cout << "Xsig_aug prediction(255)\n";
  cout << "Xsig_aug=\n" << Xsig_aug << endl;
  
  // use X_aug augmented sigma points and h(x,noise) function to transform Xsig_pred predicted sigma points
  for (int i=0; i<2*n_aug_+1; i++){
	  VectorXd xk = Xsig_aug.col(i);
	  float xkpx  = Xsig_aug(0, i);
	  float xkpy  = Xsig_aug(1, i);
	  float xkvk  = Xsig_aug(2, i);
	  float xkyw  = Xsig_aug(3, i);
	  float xkyd  = Xsig_aug(4, i);
	  float xkna  = Xsig_aug(5, i);
	  float xkny  = Xsig_aug(6, i);
	  float yydt  = xkyw + xkyd*delta_t;
	  float ttsq  = delta_t*delta_t;
	  float yddt  = xkyd*delta_t;
	  float e1    = 0.5*ttsq*cos(xkyw)*xkna;
	  float e2    = 0.5*ttsq*sin(xkyw)*xkna; 
	  float e3    = delta_t*xkna;
	  float e4    = 0.5*ttsq*xkny;
	  float e5    = delta_t*xkny;

      //cout << "Xsig_aug prediction(277)\n";
      //cout << "xk=\n" << xk << endl;
 
	  if(fabs(xkyd) > 0.001){
		  Xsig_pred_(0,i) = xkpx + xkvk/xkyd*( sin(yydt)-sin(xkyw)) + e1,
		  Xsig_pred_(1,i) = xkpy + xkvk/xkyd*(-cos(yydt)+cos(xkyw)) + e2;
		  Xsig_pred_(3,i) = xkyw + yddt + e4;
	  }
	  else {
		  Xsig_pred_(0,i) = xkpx + xkvk*cos(xkyw)*delta_t + e1,
		  Xsig_pred_(1,i) = xkpy + xkvk*sin(xkyw)*delta_t + e2;
	  	  Xsig_pred_(3,i) = xkyw + 0 + e4;
	  }
	  Xsig_pred_(2,i) = xkvk + 0 + e3;
	  Xsig_pred_(4,i) = xkyd + 0 + e5;
	  
	  /* a mistake of normalizing angle at sigma point prediction. 
		 Leaving this here as commented code for example of bad normalization practice.
	  // normalized angle
	  while(Xsig_pred_(3,i) >  M_PI) { Xsig_pred_(3,i) -= 2*M_PI; }
	  while(Xsig_pred_(3,i) < -M_PI) { Xsig_pred_(3,i) += 2*M_PI; }*/
  } // end of for loop
  
  cout << "Xsig_pred prediction(300)\n";
  cout << "Xsig_pred=\n" << Xsig_pred_ << endl;
  
  // use Xsig_pred predicted sigma points, calculate predicted mean xk+1 (x_ update) and covariance Pk+1|k (P_ update)
  x_.fill(0);  // predicted mean state is the summation of weighted Xsig_pred matrix columns. so initialize it to 0 before summation
  for(int i=0; i<2*n_aug_+1; i++){
	  x_ += weights_(i) * Xsig_pred_.col(i);
  }

  //cout << "x_ prediction(309)\n";
  P_.fill(0);  // predicted process covariance matrix is the summation of weighted Xsig_pred matrix. so initialize it to 0 before summation
  for(int i=0; i<2*n_aug_+1; i++){
	  MatrixXd Xx  = Xsig_pred_.col(i) - x_;
	  
	  // normalize angle
	  while(Xx(3) >  M_PI) { Xx(3) -= 2*M_PI; }
	  while(Xx(3) < -M_PI) { Xx(3) += 2*M_PI; }
	  
	  P_ += weights_(i) * Xx * Xx.transpose();
  }
  cout << "inside prediction(320)\n";
  cout << "predicted x_=\n" << x_ << endl;
  cout << "predicted P_=\n" << P_ << endl;
 
  return;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  cout << "inside updateLidar(340)\n";
  
  // use Xsig_pred predicted sigma points in prediction() and function of h(x) to transform predicted measurement Sk+1|k
  // this is lidar measurement, where z=[px py]
  MatrixXd H = MatrixXd(2, 5);
  H << 1, 0, 0, 0, 0,
       0, 1, 0, 0, 0;
  MatrixXd R = MatrixXd(2, 2);
  R << std_laspx_*std_laspx_, 0,
       0, std_laspy_*std_laspy_;
  
  VectorXd y  = meas_package.raw_measurements_ - H*x_;
  MatrixXd Ht = H.transpose();
  MatrixXd S  = H*P_*Ht + R;
  MatrixXd Si = S.inverse();
  MatrixXd K  = P_*Ht*Si;
  
  // new state
  x_ = x_ + K*y;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K*H)*P_;

  //cout << "updateLidar(321) - y=\n" << y << endl;
  //cout << "updateLidar(321) - Ht=\n" << Ht << endl;
  //cout << "updateLidar(321) - S=\n" << S << endl;
  //cout << "updateLidar(321) - Si=\n" << Si << endl;
  //cout << "updateLidar(321) - K=\n" << K << endl;
  //cout << "updateLidar(321) - new L x=\n" << x_ << endl;
  //cout << "updateLidar(321) - I=\n" << I << endl;
  cout << "inside UpdateLidar(370)\n";
  cout << "updated x_=\n" << x_ << endl;
  cout << "updated P_=\n" << P_ << endl;
  
  // NIS epsilon calculation
  esp  = y.transpose()*Si*y;
  epsilon(eps_size++) = esp;
  cout << "updateLidar(377)\n";
  cout << "newly esp=" << esp << endl;
}


/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  cout << "inside updateRadar(395)\n";

  // use Xsig_pred predicted sigma points in prediction() and function of h(x) to transform predicted measurement Sk+1|k
  MatrixXd Zsig = MatrixXd(3, 2*n_aug_+1);
  Zsig.fill(0.0);
  for(int i=0; i<2*n_aug_+1; i++){
	  VectorXd xk = Xsig_pred_.col(i);
	  float xkpx  = xk(0);
	  float xkpy  = xk(1);
	  float xkvk  = xk(2);
	  float xkyw  = xk(3);
	  float zrho  = sqrt(xkpx*xkpx + xkpy*xkpy);
	  float zphi  = atan2(xkpy, xkpx);
	  //float zrhd  = (fabs(zrho > 0.0001)) ? (xkpx*cos(xkyw)*xkvk + xkpy*sin(xkyw)*xkvk)/zrho : 0.1;
	  float zrhd  = (xkpx*cos(xkyw)*xkvk + xkpy*sin(xkyw)*xkvk)/zrho;
	  Zsig(0,i) = zrho;
	  Zsig(1,i) = zphi;
	  Zsig(2,i) = zrhd;
  }

  // use Zsig predicted measurement sigma points to calculate z_pred predicted measurement mean  
  VectorXd z_pred = VectorXd(3);
  z_pred.fill(0.0);
  for(int i=0; i<2*n_aug_+1; i++){
	  z_pred += weights_(i) * Zsig.col(i);
  }

  // use Zsig predicted measurement sigma points to calculate the S measurement covariance matrix
  MatrixXd S = MatrixXd(3, 3);
  S.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++) {
	  VectorXd Zzd = Zsig.col(i) - z_pred;

	  // normalize angel in radian after subtraction
	  while(Zzd(1) >  M_PI){ Zzd(1) -= 2*M_PI; }
	  while(Zzd(1) < -M_PI){ Zzd(1) += 2*M_PI; }
	  
	  S += weights_(i)*Zzd*Zzd.transpose();
  }
  // add R noise matrix to S measurement covariance
  MatrixXd R = MatrixXd(3,3);
  R << std_radr_*std_radr_, 0, 0,
       0, std_radphi_*std_radphi_, 0,
	   0, 0, std_radrd_*std_radrd_;
  S += R; 
  
  cout << "updateRadar(441)\n";
  cout << "z_pred=\n" << z_pred << endl;
  cout << "S=\n" << S << endl;
  
  // calculating Pk+1|k+1 updated covariance matrix, new P_
  // use S predicted measurement covariance to calculate Tk+1|k for cross-correlation matrix
  MatrixXd T = MatrixXd(n_x_, 3);
  T.fill(0.0);
  for(int i=1; i<2*n_aug_+1; i++){
	  // Xk+1|x,i - xk+1|k 
	  VectorXd Xxd = Xsig_pred_.col(i) - x_;

	  // normalize angel in radian after subtraction
	  while(Xxd(3) >  M_PI){ Xxd(3) -= 2*M_PI; /*cout << "i="<< i << "\t(-) Xxd(3)=" << Xxd(3) << endl;*/ }
	  while(Xxd(3) < -M_PI){ Xxd(3) += 2*M_PI; /*cout << "i="<< i << "\t(+) Xxd(3)=" << Xxd(3) << endl;*/ }
	  
	  //cout << "updateRadar(359)" << endl;
	  //cout << "i="<< i << "\tXxd(3)=" << Xxd(3) << endl;

	  // Zk+1|k,i - zk+1|k 
	  VectorXd Zzd = Zsig.col(i) - z_pred;

	  // normalize angel in radian after subtraction
	  while(Zzd(1) >  M_PI){ Zzd(1) -= 2*M_PI; /*cout << "i="<< i << "\t(+) Zzd(3)=" << Zzd(1) << endl;*/ }
	  while(Zzd(1) < -M_PI){ Zzd(1) += 2*M_PI; /*cout << "i="<< i << "\t(+) Zzd(3)=" << Zzd(1) << endl;*/ }
	  
	  //cout << "Zzd=\n" << Zzd << endl;
	  
	  //MatrixXd Zzt = Zzd.transpose();

	  T += weights_(i) * Xxd * Zzd.transpose();
  }

  cout << "updateRadar(474)\n";
  cout << "T=\n" << T << endl;
  
  // use T cross-correlation matrix to calculate Kag Kalman gain matrix
  MatrixXd Kag = MatrixXd(n_x_, 3);
  //Kag.fill(0.0);
  Kag = T * S.inverse();
  
  cout << "updateRadar(482)\n";
  cout << "Kag=\n" << Kag << endl;

  // use Kag Kalman gain matrix to calculate xk+1|k+1 update state
  VectorXd zd = (meas_package.raw_measurements_ - z_pred);
  
  // normalize angel after subtraction
  while(zd(1) >  M_PI) { zd(1)-=(2*M_PI); }
  while(zd(1) < -M_PI) { zd(1)+=(2*M_PI); }
  
  x_ += Kag*zd;
  
  // calculate Pk+1|k+1 updated covariance matrix
  P_ -= Kag*S*Kag.transpose(); 

  cout << "updateRadar(497)\n";
  cout << "newly updated x_=\n" << x_ << endl;
  cout << "newly updated P_=\n" << P_ << endl;
  
  // NIS epsilon calculation
  esp = zd.transpose()*S.inverse()*zd;
  epsilon(eps_size++) = esp;
  cout << "updateRadar(503)\n";
  cout << "newly esp=" << esp << endl;
} // UpdateRadar




/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidarUKF(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  cout << "inside updateLidarUKF(479)\n";

  // use Xsig_pred predicted sigma points in prediction() and function of h(x) to transform predicted measurement Sk+1|k
  MatrixXd Zsig = MatrixXd(2, 2*n_aug_+1);
  Zsig.fill(0.0);
  for(int i=0; i<2*n_aug_+1; i++){
	  VectorXd xk = Xsig_pred_.col(i);
	  float xkpx  = xk(0);
	  float xkpy  = xk(1);
	  Zsig(0,i) = xkpx;
	  Zsig(1,i) = xkpy;
  }

  // use Zsig predicted measurement sigma points to calculate z_pred predicted measurement mean  
  VectorXd z_pred = VectorXd(2);
  z_pred.fill(0.0);
  for(int i=0; i<2*n_aug_+1; i++){
	  z_pred += weights_(i) * Zsig.col(i);
  }

  // use Zsig predicted measurement sigma points to calculate the S measurement covariance matrix
  MatrixXd S = MatrixXd(2, 2);
  S.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++) {
	  VectorXd Zzd = Zsig.col(i) - z_pred;
	  S += weights_(i)*Zzd*Zzd.transpose();
  }
  // add R noise matrix to S measurement covariance
  MatrixXd R = MatrixXd(2,2);
  R << std_laspx_*std_laspx_, 0,
       0, std_laspy_*std_laspy_;
  S += R; 
  
  cout << "updateLidarUKF(512)\n";
  cout << "z_pred=\n" << z_pred << endl;
  cout << "S=\n" << S << endl;
  
  // calculating Pk+1|k+1 updated covariance matrix, new P_
  // use S predicted measurement covariance to calculate Tk+1|k for cross-correlation matrix
  MatrixXd T = MatrixXd(n_x_, 2);
  T.fill(0.0);
  for(int i=1; i<2*n_aug_+1; i++){
	  // Xk+1|x,i - xk+1|k 
	  VectorXd Xxd = Xsig_pred_.col(i) - x_;
	  // Zk+1|k,i - zk+1|k 
	  VectorXd Zzd = Zsig.col(i) - z_pred;
	  T += weights_(i) * Xxd * Zzd.transpose();
  }

  cout << "updateLidarUKF(528)\n";
  cout << "T=\n" << T << endl;
  
  // use T cross-correlation matrix to calculate Kag Kalman gain matrix
  MatrixXd Kag = MatrixXd(n_x_, 2);
  //Kag.fill(0.0);
  Kag = T * S.inverse();
  
  cout << "updateLidarUKF(536)\n";
  cout << "Kag=\n" << Kag << endl;

  // use Kag Kalman gain matrix to calculate xk+1|k+1 update state
  VectorXd zd = (meas_package.raw_measurements_ - z_pred);
  
  x_ += Kag*zd;
  
  // calculate Pk+1|k+1 updated covariance matrix
  P_ -= Kag*S*Kag.transpose(); 

  cout << "updateLidarUKF(547)\n";
  cout << "newly updated x_=\n" << x_ << endl;
  cout << "newly updated P_=\n" << P_ << endl;
  
  // NIS epsilon calculation
  esp = zd.transpose()*S.inverse()*zd;
  epsilon(eps_size++) = esp;
  cout << "updateLidarUKF(595)\n";
  cout << "newly esp=" << esp << endl;
}
