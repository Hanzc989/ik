#include "Kinematics.h"
Kinematics::Kinematics(float s, float e) {
	step = s;
	epsilon = e;
}

Eigen::Vector3d Kinematics::solveFKTest(std::vector<Bone> & bones, int start, float dTheta, float dPhi) {
	for (int i=start; i<bones.size(); i++) {
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

// Eigen::Vector3d Kinematics::solveFKTest(std::vector<Bone> & bones, Eigen::VectorXd angles) {
// 	for (int rot = 0; rot < angles.size()) {
// 		for (int i=0; i<bones.size(); i++) {
// 			bones[i].nextTheta = bones[i].currTheta+dTheta;
// 			bones[i].nextPhi = bones[i].currPhi+dPhi;
// 			Eigen::Vector3d increment(bones[i].length*sin(bones[i].nextTheta)*cos(bones[i].nextPhi),
// 							   bones[i].length*sin(bones[i].nextTheta)*sin(bones[i].nextPhi),
// 							   bones[i].length*cos(bones[i].nextTheta));
// 			if (i==0) {
// 				bones[i].nextPos = increment;
// 			} else if (i==start) {
// 				bones[i].nextPos = bones[i-1].currPos + increment;
// 			} else {
// 				bones[i].nextPos = bones[i-1].nextPos + increment;
// 			}
// 		}
// 	}
// 	return bones[bones.size()-1].nextPos;
// }

Eigen::Vector3d Kinematics::solveFK(std::vector<Bone> & bones, int start, float dTheta, float dPhi) {
	for (int i=start; i<bones.size(); i++) {
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

Eigen::MatrixXd Kinematics::jacobian(std::vector<Bone> & bones, float step) {
	Eigen::MatrixXd jacobian(3, 2*bones.size());
	Eigen::Vector3d newPos, currPos = bones[bones.size()-1].currPos;
	for (int i = 0; i<2*bones.size(); i++) {
		if (i%2==0) {
			newPos = solveFKTest(bones, i/2, step, 0);
		} else {
			newPos = solveFKTest(bones, i/2, 0, step);
		}
		jacobian.col(i) = (newPos-currPos)/(step);
	}
	return jacobian;
}

Eigen::MatrixXd Kinematics::pseudoInverse(Eigen::MatrixXd & jacobian) {
	//return (jacobian.transpose()*jacobian).inverse()*jacobian.transpose();
    std::cout << "j*j-t inverse" << std::endl;
    std::cout << (jacobian*jacobian.transpose()) << std::endl;
    return jacobian.transpose()*(jacobian*jacobian.transpose()).inverse();
}

void Kinematics::solveIK(std::vector<Bone> & bones, Eigen::Vector3d goalPos) {
	float currStep = step;
	Eigen::Vector3d oldPos = bones[bones.size()-1].currPos;

	while ((oldPos-goalPos).norm() > epsilon && currStep > 0.0001) {
		//std::cout << "wah" << std::endl;
		Eigen::MatrixXd j = jacobian(bones, currStep);
        std::cout << j << std::endl;
		Eigen::MatrixXd p = pseudoInverse(j);
        std::cout << p << std::endl;
        //std::cout << goalPos << std::endl;
        //std::cout << oldPos << std::endl;
		Eigen::VectorXd angles = p*currStep*(goalPos-oldPos);
        //angles = -1*angles;
		Eigen::Vector3d newPos;
        std::cout << angles << std::endl;
		for (int i=0; i<angles.size(); i+=2) {
			newPos = solveFK(bones, i/2, angles[i], angles[i+1]);
		}

        //std::cout << (newPos-goalPos).norm() << std::endl;
		if ((newPos-goalPos).norm() >= (oldPos-goalPos).norm()) {
			for (int i=angles.size()-1; i>0; i-=2) {
				solveFK(bones, i/2, -angles[i-1], -angles[i]);
			}
			currStep /= 2;
		} else {
            //std::cout << "yo" << std::endl;
			oldPos = newPos;
		}
        std::cout << "currStep: "<<currStep << std::endl;
        std::cout <<"dist: " << (oldPos-goalPos).norm() << std::endl;
	}
}
