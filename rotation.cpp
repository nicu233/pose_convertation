#include <iostream>
#include <stdlib.h>
#include<math.h>
#define pi 3.1415926535
#define d2r(d) d*pi/180
#define r2d(r) r*180/pi


//
void zyxEulerToQuaternion(double x, double y, double z, double &qw , double &qx, double &qy, double &qz){
	double c1 = cos(x/2);
	double c2 = cos(y/2);
	double c3 = cos(z/2);
	
	double s1 = sin(x/2);
	double s2 = sin(y/2);
	double s3 = sin(z/2);
	
	qw = c1 * c2 * c3 + s1 * s2 * s3;
	qx = s1 * c2 * c3 - c1 * s2 * s3;
	qy = c1 * s2 * c3 + s1 * c2 * s3;
	qz = c1 * c2 * s3 - s1 * s2 * c3;
}

void QuaternionToAxisAngle(double &qw , double &qx, double &qy, double &qz, double &x1 ,double &y1, double &z1){
	double angle_radian = 2 * acos(qw);
	
	x1 = angle_radian * qx / sqrt( 1 - qw*qw );
	y1 = angle_radian * qy / sqrt( 1 - qw*qw );
	z1 = angle_radian * qz / sqrt( 1 - qw*qw );
	
}

void AxisAngleToQuaternion(double &x1 ,double &y1, double &z1,double &qw , double &qx, double &qy, double &qz){
	double radian;
	radian = sqrt(x1*x1 + y1*y1 + z1*z1);
	qw = cos(radian/2);
	qx = x1 / radian * sin(radian / 2);
	qy = y1 / radian * sin(radian / 2);
	qz = z1 / radian * sin(radian / 2);
	
}

void QuaternionTozyxEuler(double &qw , double &qx, double &qy, double &qz, double &psi ,double &theta, double &phi){
	
	psi = atan2(2 * (qx * qy + qz * qw),(qw * qw + qx * qx - qy * qy - qz * qz));
    //theta = asin(2 * (qy * qw - qx * qz));
	theta = pi/2 - acos(2 * (qy * qw - qx * qz));
    phi = atan2(2 * (qx * qw + qz * qy),(qw * qw - qx * qx - qy * qy + qz * qz));
}

int main (int argc, char** argv)
{
		
	if (argc < 3){
		std::cout << "too few argv" << std::endl;
	}
	
	std::cout << "\n \ninput ZYX Euler "  <<std::endl;
	std::cout << "Euler x:" << argv[1] <<std::endl;
	std::cout << "Euler y:" << argv[2] <<std::endl;
	std::cout << "Euler z:" << argv[3] <<std::endl;
	
	// double x = d2r(-179.904572);
	// double y = d2r(-0.780936);
	// double z = d2r(-164.295822);
	
	double x = d2r(strtod(argv[1],NULL));
	double y = d2r(strtod(argv[2],NULL));
	double z = d2r(strtod(argv[3],NULL));
	
	double qw,qx,qy,qz;
	//axis angle 
	double x1,y1,z1;
	
	std::cout << std::endl;
	std::cout << std::endl;
	std::cout << "////////////////////////////"<<std::endl;
	std::cout << "from ZYX_Euler to quaternion"<<std::endl;
	std::cout << "////////////////////////////"<<std::endl;
	
	zyxEulerToQuaternion(x,y,z,qw,qx,qy,qz);
	
	std::cout << "qw: "<< qw <<std::endl;
	std::cout << "qx: "<< qx <<std::endl;
	std::cout << "qy: "<< qy <<std::endl;
	std::cout << "qz: "<< qz <<std::endl;
	
	std::cout << std::endl;
	std::cout << std::endl;
	std::cout << "////////////////////////////"<<std::endl;
	std::cout << "from quaternion to axis_angle"<<std::endl;
	std::cout << "////////////////////////////"<<std::endl;
	
	QuaternionToAxisAngle(qw,qx,qy,qz,x1,y1,z1);
	
	std::cout << "ax: "<< x1 <<std::endl;
	std::cout << "ay: "<< y1 <<std::endl;
	std::cout << "az: "<< z1 <<std::endl;
	
	
	std::cout << std::endl;
	std::cout << std::endl;
	std::cout << "////////////////////////////"<<std::endl;
	std::cout << "from axis_angle to quaternion"<<std::endl;
	std::cout << "////////////////////////////"<<std::endl;
	
	double qw_1,qx_1,qy_1,qz_1;
	AxisAngleToQuaternion(x1,y1,z1,qw_1,qx_1,qy_1,qz_1);
	std::cout << "qw_1: "<< qw_1<<std::endl;
	std::cout << "qx_1: "<< qx_1<<std::endl;
	std::cout << "qy_1: "<< qy_1<<std::endl;
	std::cout << "qz_1: "<< qz_1<<std::endl;
	
	
	std::cout << std::endl;
	std::cout << std::endl;
	std::cout << "////////////////////////////"<<std::endl;
	std::cout << "from quaternion to ZYX_Euler"<<std::endl;
	std::cout << "////////////////////////////"<<std::endl;
	
	//psi = z, theta = y, phi = x
	double psi,theta,phi;
	QuaternionTozyxEuler(qw_1,qx_1,qy_1,qz_1,psi,theta,phi);
	std::cout << "x: "<< r2d(phi) <<std::endl;
	std::cout << "y: "<< r2d(theta) <<std::endl;
	std::cout << "z: "<< r2d(psi) <<std::endl;
	
	
	//
	
}