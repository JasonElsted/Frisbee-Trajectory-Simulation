//=======================================================================
// Name        : FrisbeeFlightTrajectory.cpp
// Author      : Sam Swift
// Version     : 1.0
// Description : Frisbee Flight Simulation
//=======================================================================

#define _USE_MATH_DEFINES

#include <iostream>
#include <cmath>
#include <fstream>

using namespace std;

//World Constants
double g = -9.81;
double air_density = 1.225;


double diameter = 0.27305;
double area = M_PI * diameter * diameter / 2;
double Iz = 0.00235;

class Vector {
private:
	double x;
	double y;
	double z;
public:
	Vector()
	{
		x = 0;
		y = 0;
		z = 0;
	}
	Vector(double x0, double y0, double z0)
	{
		x = x0;
		y = y0;
		z = z0;
	}
	double dot(Vector v)
	{
		return (x * v.x) + (y * v.y) + (z * v.z);
	}
	Vector cross(Vector v)
	{
		Vector ret(y * v.z - z * v.y,
				   z * v.x - x * v.z,
				   x * v.y - y * v.x);
		return ret;
	}
	Vector scale(double n)
	{
		Vector ret(x * n, y * n, z * n);
		return ret;
	}
	Vector operator*(double n)
	{
		return scale(n);
	}
	Vector operator/(double n)
	{
		return scale(1/n);
	}
	double operator*(Vector v)
	{
		return dot(v);
	}
	double magnitude()
	{
		return sqrt(x *x + y * y + z * z);
	}
	Vector unit()
	{
		return *this / magnitude();
	}
	Vector rotateX(double theta)
	{
		Vector ret(x,
				   cos(theta) * y - sin(theta) * z,
				   sin(theta) * y + cos(theta) * z);
		return ret;
	}
	Vector rotateY(double theta)
	{
		Vector ret(cos(theta) * x + sin(theta) * z,
				   y,
				   cos(theta) * z - sin(theta) * x);
		return ret;
	}
	Vector rotateZ(double theta)
	{
		Vector ret(cos(theta) * x - sin(theta) * y,
				   sin(theta) * x + cos(theta) * y,
				   z);
		return ret;
	}

	double getX()
	{
		return x;
	}
	double getY()
	{
		return y;
	}
	double getZ()
	{
		return z;
	}
	void setX(double n)
	{
		x = n;
	}
	void setY(double n)
	{
		y = n;
	}
	void setZ(double n)
	{
		z = n;
	}
	Vector operator+(Vector v)
	{
		Vector ret(x + v.x, y + v.y, z + v.z);
		return ret;
	}
	Vector operator-(Vector v)
	{
		Vector ret(x - v.x, y - v.y, z - v.z);
		return ret;
	}
	void operator+=(Vector v)
	{
		x += v.x;
		y += v.y;
		z += v.z;
	}
	void operator=(Vector v)
	{
		x = v.x;
		y = v.y;
		z = v.z;
	}
	void print(ostream & out)
	{
		out << *this;
	}
	void print()
	{
		print(cout);
	}
	friend ostream &operator<<(ostream & out, const Vector &v)
	{
		out << v.x << "," << v.y << "," << v.z;
		return out;
	}
};

double angleOfAttack(Vector Normal, Vector Velocity)
{
	return acos(Normal * Velocity / Normal.magnitude() / Velocity.magnitude()) - M_PI / 2;
}

Vector weight(double m)
{
	double weight = m * g;
	Vector Fgravity(0, 0, weight);
	return Fgravity;
}

Vector lift(Vector normal, Vector v)
{
	double attackAngle = angleOfAttack(normal, v);
	double Clift0 = 0.13;
	double Clifta = 3.09;
	double Clift = Clift0 + Clifta * attackAngle;

	double lift = 0.5 * air_density * (v * v) * area * Clift;

	Vector Flift = v.cross(normal).cross(v).unit() * lift;

	return Flift;
}

Vector drag(Vector normal, Vector v)
{
	double attackAngle = angleOfAttack(normal, v);
	double Cdrag0 = 0.085;
	double Cdraga = 3.30;
	double a0 = -0.052;
	double Cdrag = Cdrag0 + Cdraga * pow(attackAngle - a0, 2);

	double drag = -0.5 * air_density * (v * v) * area * Cdrag;
	Vector Fdrag = v.unit() * drag;
	return Fdrag;
}

Vector pitchForce(Vector normal, Vector v)
{
	double attackAngle = angleOfAttack(normal, v);
	double Cmoment0 = -0.01;
	double Cmomenta = 0.057;
	double Cmoment = Cmoment0 + Cmomenta * attackAngle;

	double moment = 0.5 * air_density * (v * v) * area * Cmoment * diameter;

	Vector Fmoment = normal.cross(v).unit() * moment;

	return Fmoment;
}

int main() {

	//Unit Vectors
	Vector I(1, 0, 0);
	Vector J(0, 1, 0);
	Vector K(0, 0, 1);

	Vector i(0, 0, 0);
	Vector j(0, 0, 0);
	Vector k(0, 0, 0);

	//Initial Conditions
	double m = 0.175;
	Vector Position(0, 0, 1);
	Vector rotation(0, 0, -100);
	Vector Velocity(20, 0, 0);
	Vector Normal(-0.1, 0, 1);

	ofstream fout("../simulation_data/trajectory_m0.175_r-100_Vx20_Vz0_Nx-0.1_Ny0_Nz1.csv");
	fout << "time,x,y,z" << endl;

	//Intermediate values
	Vector Force(0, 0, 0);
	Vector Acceleration(0, 0, 0);
	Vector moment(0, 0, 0);
	Vector normal(0, 0, 0);
	Vector radius(0, 0, 0);
	Vector Radius(0, 0, 0);
	//double attackAngle = 0;	//Angle of Attack
	double rateOfPrecession = 0;
	//double My = 0;	//Moment about relative y axis In direction K x V

	double step = 0.001;	//Time Step in seconds
	int num_steps = 0;

	for(double time = 0; Position.getZ() > 0; time += step)
	{

		i = Velocity.unit();
		j = Normal.cross(Velocity).unit();
		k = i.cross(j);

		//fout << time << "," << Position << endl;

		fout << time << "," << Position + j * diameter / 2 << endl;
		fout << time << "," << Position - j * diameter / 2 << endl;

		//Step kinematically
		Force = weight(m) + lift(Normal, Velocity) + drag(Normal, Velocity);
		Acceleration = Force / m;
		Position += Velocity * step + Acceleration * 0.5 * step * step;
		Velocity += Acceleration * step;

		moment = pitchForce(Normal, Velocity);
		//Step angle
		rateOfPrecession = -moment.magnitude() / (Iz * rotation.magnitude());
		//Convert normal vector to little xyz
		normal.setX(Normal * i);
		normal.setY(Normal * j);
		normal.setZ(Normal * k);
		//Rotate normal vector in little xyz
		normal = normal.rotateZ(rateOfPrecession * step);
		//Convert normal vector to BIG XYZ
		Normal = i * normal.getX() + j * normal.getY() + k * normal.getZ();
		num_steps++;
	}

	return 0;
}
