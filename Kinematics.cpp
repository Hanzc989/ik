#include "Kinematics.h"

Eigen::Vector3d Kinematics::solveFKTest(std::vector<Bone> & bones, int start, float dTheta, float dPhi) {
	for(int i=start; i<bones.size(); i++) {
		bones[i].nextTheta = bones[i].currTheta+dTheta;
		bones[i].nextPhi = bones[i].currPhi+dPhi;
		Eigen::Vector3d increment(bones[i].length*sin(bones[i].nextTheta)*cos(bones[i].nextPhi),
						   bones[i].length*sin(bones[i].nextTheta)*sin(bones[i].nextPhi),
						   bones[i].length*cos(bones[i].nextTheta));
		if (i==0) {
			bones[i].nextPos = increment;
		} else if (i==start) {
			bones[i].nextPos = bones[i-1].currPos + increment;
		} else {
			bones[i].nextPos = bones[i-1].nextPos + increment;
		}
	}
	return bones[bones.size()-1].nextPos;
}

Eigen::Vector3d Kinematics::solveFK(std::vector<Bone> & bones, int start, float dTheta, float dPhi) {
	for(int i=start; i<bones.size(); i++) {
		bones[i].currTheta = bones[i].currTheta+dTheta;
		bones[i].currPhi = bones[i].currPhi+dPhi;
		Eigen::Vector3d increment(bones[i].length*sin(bones[i].currTheta)*cos(bones[i].currPhi),
						   bones[i].length*sin(bones[i].currTheta)*sin(bones[i].currPhi),
						   bones[i].length*cos(bones[i].currTheta));
		if (i==0) {
			bones[i].currPos = increment;
		} else {
			bones[i].currPos = bones[i-1].currPos + increment;
		}
	}
	return bones[bones.size()-1].currPos;
}

Eigen::MatrixXd Kinematics::jacobian(Bone & root, float step) {

}