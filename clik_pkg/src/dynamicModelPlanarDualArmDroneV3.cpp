#include "dynamicModelPlanarDualArmDroneV3.hpp"
#include <iostream>

#define mD 20.269
#define mS 1.917 //3*0.639
#define mA 0.799 //0.233+0.246+0.214+0.106
#define LC 1          //1
#define off 0.14      //0.14
#define iDxx 0.845
#define iSxx 9.06e-3 //3*3.02e-3
#define iAxx 0.017188145790769
#define iDyy 0.845
#define iSyy 2.592e-3 //3*8.64e-4
#define iAyy 0.017011890024962
#define lSz 0         //0
#define lAz 0.25      //0.25 //maybe should be 0.524 
#define fvD 0
#define fvA 0
#define fvS 0
#define g0 9.8        //9.8

// #define mD 4.65       //20.269
// #define mS 0.1        //3*0.639
// #define mA 1          //0.233+0.246+0.214+0.106
// #define LC 1          //1
// #define off 0.14      //0.14
// #define iDxx 0.3045   //0.845
// #define iSxx 0.001    //3*3.02e-3
// #define iAxx 0.0625   //0.017188145790769
// #define iDyy 0.2156   //0.845
// #define iSyy 0.001    //3*8.64e-4
// #define iAyy 0.1      //0.017011890024962
// #define lSz 0         //0
// #define lAz 0.25      //0.25 //maybe should be 0.524 
// #define fvD 0.1       //0
// #define fvA 0.05      //0
// #define fvS 0.05      //0
// #define g0 9.8        //9.8

void B_row1(Eigen::MatrixXd& B, Eigen::VectorXd qVec) {
  
  double qD = qVec(0);
  double qC = qVec(1);
  double qA = qVec(2);

  double t2 = -qC;
  double t3 = qA+t2;
  double t4 = cos(t3);
  double t5 = lAz*mA*t4;
  double t6 = -t5;
  B(0,0) = mA*2.0+mD+mS;
  B(0,1) = t5*-2.0-lSz*mS*cos(qC);
  B(0,2) = t6;
  B(0,3) = t6;

}



void B_row2(Eigen::MatrixXd& B, Eigen::VectorXd qVec) {
  
  double qD = qVec(0);
  double qC = qVec(1);
  double qA = qVec(2);

  double t2 = lAz*lAz;
  double t3 = mA*t2;
  double t4 = iAyy+t3;
  B(1,0) = -lSz*mS*cos(qC)-lAz*mA*cos(qA-qC)*2.0;
  B(1,1) = iSyy+t4*2.0+(lSz*lSz)*mS;
  B(1,2) = t4;
  B(1,3) = t4;



}

void B_row3(Eigen::MatrixXd& B, Eigen::VectorXd qVec) {

  double qD = qVec(0);
  double qC = qVec(1);
  double qA = qVec(2);

  double t2 = lAz*lAz;
  double t3 = mA*t2;
  double t4 = iAyy+t3;
  B(2,0) = -lAz*mA*cos(qA-qC);
  B(2,1) = t4;
  B(2,2) = t4;

}

void B_row4(Eigen::MatrixXd& B, Eigen::VectorXd qVec) {

  double qD = qVec(0);
  double qC = qVec(1);
  double qA = qVec(2);

  double t2 = lAz*lAz;
  double t3 = mA*t2;
  double t4 = iAyy+t3;
  B(3,0) = -lAz*mA*cos(qA-qC);
  B(3,1) = t4;
  B(3,3) = t4;


}

void n_row1(Eigen::VectorXd& n, Eigen::VectorXd qVec, Eigen::VectorXd qDotVec) {
  
  double qD = qVec(0);
  // std::cout<<"qD: "<<qD<<std::endl;
  double qC = qVec(1);
  // std::cout<<"qC: "<<qC<<std::endl;
  double qA = qVec(2);
  // std::cout<<"qA: "<<qA<<std::endl;
  double qDd = qDotVec(0);
  // std::cout<<"qDd: "<<qDd<<std::endl;
  double qCd = qDotVec(1);
  // std::cout<<"qCd: "<<qCd<<std::endl;
  double qAd = qDotVec(2);
  // std::cout<<"qAd: "<<qAd<<std::endl;

  double t2 = qCd*qCd;
  // std::cout<<"t2: "<<t2<<std::endl;
  double t3 = -qC;
  // std::cout<<"t3: "<<t3<<std::endl;
  double t4 = qA+t3;
  // std::cout<<"t4: "<<t4<<std::endl;
  double t5 = sin(t4);
  // std::cout<<"t5: "<<t5<<std::endl;

  n(0) = fvD*qDd+lAz*mA*t2*t5*2.0+lAz*mA*(qAd*qAd)*t5*2.0-lSz*mS*t2*sin(qC)-lAz*mA*qAd*qCd*t5*4.0;
  // std::cout<<"fvD*qDd: "<<fvD*qDd<<std::endl; 
  // std::cout<<"lAz*mA*t2*t5*2.0: "<<lAz*mA*t2*t5*2.0<<std::endl;
  // std::cout<<"lAz*mA*(qAd*qAd)*t5*2.0: "<<lAz*mA*(qAd*qAd)*t5*2.0<<std::endl;
  // std::cout<<"-lSz*mS*t2*sin(qC): "<<-lSz*mS*t2*sin(qC)<<std::endl;
  // std::cout<<"-lAz*mA*qAd*qCd*t5*4.0: "<<-fvD*qDd<<std::endl;


  // n(0) = -fvS*qCd-g0*lSz*mS*sin(qC)+g0*lAz*mA*sin(qA-qC)*2.0;     // SENZA rotazione vettore di gravita' (probabilmente sbagliato)

}

void n_row2(Eigen::VectorXd& n, Eigen::VectorXd qVec, Eigen::VectorXd qDotVec) {

  double qD = qVec(0);
  double qC = qVec(1);
  double qA = qVec(2);
  double qDd = qDotVec(0);
  double qCd = qDotVec(1);
  double qAd = qDotVec(2);

  n(1) = -fvS*qCd-g0*lSz*mS*sin(qC)+g0*lAz*mA*sin(qA-qC)*2.0;
  // n(1) = fvA*qAd+g0*lAz*mA*sin(qA-qC)-lAz*mA*off*(qCd*qCd)*cos(qA);     // SENZA rotazione vettore di gravita' (probabilmente sbagliato)

}


void n_row3(Eigen::VectorXd& n, Eigen::VectorXd qVec, Eigen::VectorXd qDotVec) {
  
  double qD = qVec(0);
  double qC = qVec(1);
  double qA = qVec(2);
  double qDd = qDotVec(0);
  double qCd = qDotVec(1);
  double qAd = qDotVec(2);

  n(2) = fvA*qAd+g0*lAz*mA*sin(qA-qC);

  // n(2) = fvA*qAd+g0*lAz*mA*sin(qA-qC)+lAz*mA*off*(qCd*qCd)*cos(qA);     // SENZA rotazione vettore di gravita' (probabilmente sbagliato)

}

void n_row4(Eigen::VectorXd& n, Eigen::VectorXd qVec, Eigen::VectorXd qDotVec) {
  
  double qD = qVec(0);
  double qC = qVec(1);
  double qA = qVec(2);
  double qDd = qDotVec(0);
  double qCd = qDotVec(1);
  double qAd = qDotVec(2);

  n(3) = fvA*qAd+g0*lAz*mA*sin(qA-qC);

}


