#pragma once
#include "includes.h"
#include "Bone.h"

class Kinematics
{
public:
	float step, epsilon;
	Kinematics(float s, float e){};
	
	Eigen::Vector3d solveFKTest(std::vector<Bone> & bones, int index, float dTheta, float dPhi);
	Eigen::Vector3d solveFK(std::vector<Bone> & bones, int index, float dTheta, float dPhi);
	Eigen::VectorXd solveIK(Bone & root, Eigen::Vector3d goalPos, float startStep);
	Eigen::MatrixXd jacobian(Bone & root, float step);
	Eigen::MatrixXd psuedoInverse(Eigen::MatrixXd jacobian);
};