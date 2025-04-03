#include <Common/Math/Matrix.hpp>
#include <Common/Math/Rotation.hpp>
#include <Common/Time/Timer.hpp>
#include <iostream>
#include <thread>
#include <math.h>

#include "Common/Time/HardwareTimer.hpp"

HardwareTimer hwTimer;

void TestTiming(void){
	Timer t(&hwTimer);
	int sleepTime = 1;
	printf("Will sleep for %d seconds\n", sleepTime);
    std::this_thread::sleep_for (std::chrono::seconds(sleepTime));
	printf("\tMeasured duration %lfs, or %dus", t.GetSeconds<float>(), int(t.GetMicroSeconds()));
}

template<typename Real>
void TestRotations(void){
//	Vec3<Real> rotVec(1,0,0);
	Vec3<Real> rotVec(0,0,Real(M_PI/2));
	Rotation<Real> rot = Rotation<Real>::FromRotationVector(rotVec);
	Vec3<Real> v(1,0,0);

	Vec3<Real> out = rot*v;
	printf("Rot vec = %f, %f, %f\n", rotVec.x, rotVec.y, rotVec.z);
	printf("In vec  = %f, %f, %f\n", v.x, v.y, v.z);
	printf("Out     = %f, %f, %f\n", out.x, out.y, out.z);
	printf("length  %f, %f\n", v.GetNorm2(), out.GetNorm2());
	printf("=====================\n");

	Vec3<Real> rv, vout;
	Rotation<Real> r1;

	rv = Vec3<Real>(1,1,0);
	while(rv.GetNorm2() > M_PI)
	{
		printf("REducing\n");
    rv -= 2 * Real(M_PI) * rv / rv.GetNorm2();
	}
	vout  = Rotation<Real>::FromRotationVector(rv).ToRotationVector();
	printf("In vec  = %f, %f, %f\n", rv.x, rv.y, rv.z);
	printf("Out     = %f, %f, %f\n", vout.x, vout.y, vout.z);
	printf("=====================\n");
	return;
}

int main()
{
	TestRotations<float>();
	TestRotations<double>();

	TestTiming();
	return 0;
}
