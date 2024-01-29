#pragma once

#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>
#include <math.h>
#include <vector>
#include <algorithm>
#include <map>
#include <unordered_set>

#include "../../lib/IMU/IMU.h"
#include "../../lib/GlobalPositioningSystem/GlobalPositioningSystem.h"
#include "../../lib/myMath/myMath.h"
#include "../../lib/easyLiDAR/easyLiDAR.h"
#include "../../lib/Tank/Tank.h"

extern GyroZ gyro;
extern GlobalPositioningSystem gps;
extern Tank tank;

extern Robot* robot;
extern Lidar* centralLidar;
extern int timeStep;

typedef struct {
	float x;
	float z;
}XZcoordinate;

enum class TagRange { // 測定範囲のタグ
	LL, // left Large, right Lerge 
	LS, // left Large, right Small
	SL, // left Small, right Large
	SS, // left Small, right Small
};

enum class TagMinDistance { // 測定範囲の中で最も近い壁がどこにあるか
	Nine,
	Fifteen,
	TwentyOne,
	Nine_and_Fifteen,
};

class pcModelBox {
public:
	pcModelBox(const TagRange tagRen, const TagMinDistance tagMinDis, const vector<float> _model) {
		model = _model;
		minDistance = tagMinDis;
		startDegree = startDegreeMap[tagRen];
		endDegree = endDegreeMap[tagRen];
	}

	vector<float> model;
	TagMinDistance minDistance;
	int startDegree = 0; // 0~512
	int endDegree = 0; // 0~512
private:
	//map<TagMinDistance, float> minDistanceMap = {
	//	{TagMinDistance::Nine, (float)8},
	//	{TagMinDistance::Fifteen, (float)14},
	//	{TagMinDistance::TwentyOne, (float)20},
	//	{TagMinDistance::Nine_and_Fifteen, (float)9}
	//};
	map<TagRange, int> startDegreeMap = {
		{TagRange::LL, -36}, // -25(360) -35.555(512)
		{TagRange::LS, -36}, // -25(360)
		{TagRange::SL, -22}, // -15(360) -21.333(512)
		{TagRange::SS, -22}  // -15(360)
	};
	map<TagRange, int> endDegreeMap = {
		{TagRange::LL, 36}, // 25(360)
		{TagRange::LS, 22}, // 15(360)
		{TagRange::SL, 36}, // 25(360)
		{TagRange::SS, 22}	// 15(360)
	};
};

class pcDetails {
public:
	pcDetails(const LiDAR_degree lidarDeg, pcModelBox& modelbox) :model(modelbox.model) {
		Degree = centralDegreeMap[lidarDeg];
		startDegree = Degree + modelbox.startDegree;
		endDegree = Degree + modelbox.endDegree;
		if (startDegree < 0) {
			startDegree += 512;
			flag0and512 = true;
		}
		minDistance = modelbox.minDistance;
	}

	vector<float>& model;
	int Degree = 0; // 0~512
	int startDegree = 0; // 0~512
	int endDegree = 0; // 0~512
	bool flag0and512 = false; // 0度と512度を跨いだ範囲であるか
	TagMinDistance minDistance; // 最も近い壁の距離の最大値
private:
	map<LiDAR_degree, int> centralDegreeMap = {
		{LiDAR_degree::FRONT,0},
		{LiDAR_degree::RIGHT,128},
		{LiDAR_degree::BACK, 256},
		{LiDAR_degree::LEFT, 384}
	};
};

class PointCloudLiDAR {
public:
	PointCloudLiDAR() {}

	// LiDARの値を更新する
	void update(GPSPosition goalPos) {
		rangeImage = centralLidar->getRangeImage();
		converttoPointCloud();

		float gyro_angle = (float)gyro.getGyro();
		if (abs(gyro_angle - 90) < 5) {
			fixPointCloudAngle(90, gyro_angle);
		}
		else if (abs(gyro_angle - 180) < 5) {
			fixPointCloudAngle(180, gyro_angle);
		}
		else if (abs(gyro_angle - 270) < 5) {
			fixPointCloudAngle(270, gyro_angle);
		}
		else if ((gyro_angle <= 0 && gyro_angle < 5) || (gyro_angle > 355 && gyro_angle <= 360)) {
			fixPointCloudAngle(0, gyro_angle);
		}
		GPSPosition nowPos = gps.getPosition();
		fixPointCloudPosition(goalPos, nowPos, gyro_angle);
	}

	// 修正する角度を指定してLiDARの値を更新する
	void update(const float angle) {
		rangeImage = centralLidar->getRangeImage();
		converttoPointCloud();
		float gyro_angle = (float)gyro.getGyro();
		fixPointCloudAngle(angle,gyro_angle);
	}

	float getDistance(const float angle_R) {
		int angle512 = (int)round(angle_R * 512 / 360);
		return rangeImage[angle512 + 1024];
	}

	// LiDARの変換されたXZ平面上の点
	vector<XZcoordinate> pointCloud = vector<XZcoordinate>(512, { 0,0 });

	// 特別に作成したワールドの壁をモデリングする専用の関数
	void modelSamplimg();

	void move_update_display(GPSPosition goalPos, int j);
private:
	const float* rangeImage = 0;
	MyMath myMath;

	void convertRELATIVEtoABSLOUTE(float& angle) { // 相対角を絶対角に変換
		//double g_angle = 360 - gyro.getGyro();
		//angle = 360 - angle; // 向きを反転
		//angle -= g_angle;
		angle = (float)gyro.getGyro() - angle;
		if (angle < 0) angle += 360;
	}

	void convertRELATIVEtoABSLOUTE(float& angle, const float& gyro) { // 相対角を絶対角に変換 IMUの値を受け取るver
		angle = gyro - angle;
		if (angle < 0) angle += 360;
	}

	void convertABSLOUTEtoRELATIVE(float& angle) { // 絶対角を相対角に変換
		//double g_angle = 360 - gyro.getGyro();
		//angle += g_angle;
		angle += (float)gyro.getGyro();
		if (angle > 360) angle -= 360;
	}

	void convertABSLOUTEtoRELATIVE(float& angle, const float& gyro) { // 絶対角を相対角に変換 IMUの値を受け取るver
		angle += gyro;
		if (angle > 360) angle -= 360;
	}

	// ポイントクラウドに変換
	void converttoPointCloud() {
		float tmpDistance = 0;
		for (int i = 0; i < 512; i++) {
			if (isinf(rangeImage[i + 1024])) {
				tmpDistance = (float)1.01;
			}
			else tmpDistance = rangeImage[i + 1024];
			tmpDistance *= 100; // m -> cm
			pointCloud[i].x = tmpDistance * myMath.sin[i];
			pointCloud[i].z = tmpDistance * myMath.cos[i];
			//cout << pointCloud[i].x << "," << pointCloud[i].z << endl;
		}
	}

	// 角度補正
	void fixPointCloudAngle(const float angle_G_A, const float& gyro_angle) {
		//cout << "angle goal absolute" << angle_G_A << ", gyro angle" << gyro_angle << endl;
		//int da = (int)round((angle_G_A - gyro_angle) * 512 / 360); // 偏角を求める 0~360 -> 0~512 四捨五入
		float da = angle_G_A - gyro_angle;
		da *= -1; // 時計回りに変換
		//cout << "da:" << da << endl;
		//if (da < 0) da += 512;
		//if (da > 512) da -= 512;
		// x=xcosθ−ysinθ
		// y=ycosθ+xsinθ
		da = da * (float)3.1415926535 / 180;
		//cout << "da:" << da << endl;
		if (da != 0 && da != 512) {
			for (int i = 0; i < 512; i++) {
				pointCloud[i].x = pointCloud[i].x * (float)cos(da) - pointCloud[i].z * (float)sin(da);
				pointCloud[i].z = pointCloud[i].z * (float)cos(da) + pointCloud[i].x * (float)sin(da);
			}
		}
	}

	void fixPointCloudPosition(GPSPosition goalPos, GPSPosition nowPos, const float& gyro_angle) {
		float dx = (float)goalPos.x - (float)nowPos.x;
		float dz = (float)goalPos.z - (float)nowPos.z;
		//cout << "goalPos.x:" << goalPos.x << ", goalPos.z:" << goalPos.z << endl;
		//cout << "nowPos.x:" << nowPos.x << ", nowPos.z:" << nowPos.z << endl;
		//cout << "dx:" << dx << ", dz:" << dz << endl;
		if (abs(gyro_angle - 90) < 5) {
			//cout << "90" << endl;
			for (int i = 0; i < 512; i++) {
				pointCloud[i].x += dz;
				pointCloud[i].z += dx;
			}
		}
		else if (abs(gyro_angle - 180) < 5) {
			//cout << "180" << endl;
			for (int i = 0; i < 512; i++) {
				pointCloud[i].x += dx;
				pointCloud[i].z -= dz;
			}
		}
		else if (abs(gyro_angle - 270) < 5) {
			//cout << "270" << endl;
			for (int i = 0; i < 512; i++) {
				pointCloud[i].x -= dz;
				pointCloud[i].z -= dx;
			}
		}
		else if ((gyro_angle <= 0 && gyro_angle < 5) || (gyro_angle > 355 && gyro_angle <= 360)) {
			//cout << "0" << endl;
			for (int i = 0; i < 512; i++) {
				pointCloud[i].x -= dx;
				pointCloud[i].z += dz;
			}
		}
	}

	double pd_degrees_to_rad(const double degrees) {
		return degrees * 3.1415926535 / 180;
	}

	void displayAllfloatVector(vector<float>& vec);

	void displayAllXZcoordinateVector(vector<XZcoordinate>& vec, int j);

	// モデルの嵐
	//pcModelBox model_p2_1 =  pcModelBox(TagRange::SS, TagMinDistance::Fifteen, {  -17.0664F, -17.0046F, -16.9275F, -16.8455F, -16.7421F, -16.6326F, -16.5061F, -16.3656F, -16.2151F, -16.0375F, -15.8451F, -15.3978F, -15.1309F, -14.8324F, -14.491F, -14.0886F, -13.5891F, -12.8317F, -11.79F, -11.7913F, -11.7925F, -11.7938F, -11.7949F, -11.7961F, -11.7974F, -11.7987F, -17.3046F, -17.3065F, -17.3083F, -17.3102F, -17.312F, -17.3139F, -17.3158F, -17.3179F, -17.3198F, -17.3217F, -17.3235F, -17.325F, -17.3269F, -17.3288F, -17.3308F, -17.3327F, -17.3346F, -17.3366F, -17.3386F });
	pcModelBox model_p2_2 =  pcModelBox(TagRange::SS, TagMinDistance::Fifteen, { -17.2163F, -17.1483F, -17.0656F, -16.9718F, -16.8677F, -16.7448F, -16.6143F, -16.4635F, -16.2963F, -16.1167F, -15.9102F, -15.4313F, -15.1512F, -14.8337F, -14.4611F, -14.0038F, -13.395F, -11.9739F, -11.9751F, -11.9764F, -11.9777F, -11.979F, -11.9783F, -11.9795F, -11.9808F, -17.4225F, -17.3851F, -17.3449F, -17.291F, -17.2343F, -17.164F, -17.0913F, -17.005F, -16.8149F, -16.7108F, -16.593F, -16.4737F, -16.3399F, -16.2063F, -16.0581F, -15.9101F, -15.7469F, -15.5857F, -15.4077F, -15.2316F });
	pcModelBox model_p2_3 =  pcModelBox(TagRange::SS, TagMinDistance::Fifteen, { -17.2909F, -17.2283F, -17.1568F, -17.0738F, -16.9758F, -16.8717F, -16.7441F, -16.6088F, -16.4573F, -16.2843F, -16.0978F, -15.6546F, -15.3944F, -15.1038F, -14.7728F, -14.3842F, -13.9045F, -13.2574F, -11.9952F, -11.9965F, -11.9978F, -11.999F, -11.9998F, -11.5061F, -11.513F, -11.5199F, -11.5305F, -11.5487F, -11.567F, -11.5853F, -11.6131F, -11.6435F, -11.674F, -11.7524F, -11.7964F, -11.8424F, -11.9015F, -11.961F, -12.0225F, -12.0999F, -12.1784F, -12.2635F, -12.3639F, -12.466F, -12.5891F });
	pcModelBox model_p2_4 =  pcModelBox(TagRange::SS, TagMinDistance::Fifteen, { -17.263F, -17.2005F, -17.1249F, -17.0422F, -16.9402F, -16.8321F, -16.7045F, -16.5649F, -16.414F, -16.2359F, -16.0443F, -15.5968F, -15.33F, -15.0319F, -14.6915F, -14.2906F, -13.7952F, -13.061F, -11.9797F, -11.981F, -11.9823F, -11.9835F, -11.9839F, -11.9852F, -11.9865F, -11.9878F, -14.1832F, -13.9798F, -13.7828F, -13.6169F, -13.455F, -13.3118F, -13.1775F, -12.9361F, -12.8261F, -12.7226F, -12.6307F, -12.5397F, -12.4534F, -12.3783F, -12.3041F, -12.2308F, -12.1688F, -12.1085F, -12.0488F });
	pcModelBox model_p2_5 =  pcModelBox(TagRange::SS, TagMinDistance::Fifteen, { -17.2661F, -17.2036F, -17.1278F, -17.0451F, -16.9428F, -16.8344F, -16.7067F, -16.5664F, -16.4155F, -16.2372F, -16.0449F, -15.5967F, -15.3296F, -15.0308F, -14.6891F, -14.2867F, -13.7876F, -13.0428F, -11.9836F, -11.9849F, -11.9861F, -11.9874F, -11.9878F, -11.9891F, -11.9904F, -11.9917F, -13.4285F, -14.0518F, -14.5141F, -14.8928F, -15.2179F, -15.5012F, -15.7556F, -16.1982F, -16.3804F, -16.5493F, -16.7064F, -16.8379F, -16.9618F, -17.0707F, -17.1649F, -17.2525F, -17.3204F, -17.3848F, -17.4317F });
	pcModelBox model_p2_6 =  pcModelBox(TagRange::SS, TagMinDistance::Fifteen, { -17.2664F, -17.2041F, -17.1286F, -17.0457F, -16.9435F, -16.8354F, -16.7078F, -16.5678F, -16.4169F, -16.2389F, -16.0469F, -15.5993F, -15.3326F, -15.0339F, -14.6931F, -14.2907F, -13.7937F, -13.0545F, -11.9833F, -11.9846F, -11.9859F, -11.487F, -11.4874F, -11.4886F, -11.4898F, -11.4911F, -11.4923F, -11.4935F, -11.4947F, -11.496F, -11.4972F, -11.4984F, -11.4997F, -11.5017F, -11.503F, -11.5042F, -11.5054F, -11.5064F, -11.5076F, -11.5089F, -11.5101F, -11.5114F, -11.5126F, -11.5139F, -11.5151F });
	pcModelBox model_p2_7 =  pcModelBox(TagRange::SL, TagMinDistance::Nine_and_Fifteen, { -17.2675F, -17.205F, -17.1298F, -17.0471F, -16.9453F, -16.8373F, -16.7097F, -16.5702F, -16.4193F, -16.2418F, -16.0503F, -15.6031F, -15.3368F, -15.0392F, -14.6994F, -14.2993F, -13.8041F, -13.0745F, -11.9832F, -11.9845F, -11.9858F, -11.4863F, -11.4867F, -11.4823F, -11.4779F, -11.473F, -11.4576F, -11.4421F, -11.4268F, -11.4008F, -11.3746F, -11.3485F, -11.313F, -11.2396F, -11.1952F, -11.1486F, -11.1024F, -11.0496F, -10.9937F, -10.9383F, -10.8789F, -10.8142F, -10.7504F, -10.6853F, -10.6126F, -10.541F, -10.4702F, -10.3915F, -10.3128F, -10.2354F, -10.0687F, -9.98527F, -9.90269F, -9.81229F, -9.72352F, -9.63635F, -9.45234F, -9.36091F, -9.27125F });
	pcModelBox model_p2_8 =  pcModelBox(TagRange::SL, TagMinDistance::Nine, { -17.2784F, -17.2158F, -17.1441F, -17.0612F, -16.9633F, -16.8591F, -16.7314F, -16.5964F, -16.4449F, -16.2719F, -16.0854F, -15.6422F, -15.3825F, -15.0923F, -14.7618F, -14.3723F, -13.8933F, -13.245F, -11.9828F, -11.9841F, -11.9853F, -11.9866F, -11.9874F, -5.48652F, -5.48981F, -5.49311F, -5.4964F, -5.49971F, -5.50301F, -5.50633F, -5.50964F, -5.51836F, -5.52717F, -5.54452F, -5.55341F, -5.56232F, -5.57127F, -5.58413F, -5.59906F, -5.61407F, -5.62917F, -5.64435F, -5.65961F, -5.67495F, -5.69561F, -5.71792F, -5.7404F, -5.76307F, -5.78591F, -5.80892F, -5.86678F, -5.8986F, -5.93077F, -5.96328F, -5.99615F, -6.03755F, -6.12574F, -6.17104F, -6.21701F });
	pcModelBox model_p2_9 =  pcModelBox(TagRange::SL, TagMinDistance::Nine, { -17.2643F, -17.2018F, -17.1265F, -17.0438F, -16.9421F, -16.8344F, -16.7068F, -16.5674F, -16.4165F, -16.239F, -16.0476F, -15.6005F, -15.3346F, -15.0372F, -14.6972F, -14.297F, -13.8023F, -13.0752F, -10.7757F, -10.1747F, -9.78303F, -9.47033F, -9.47068F, -9.22775F, -9.00491F, -8.8214F, -8.6453F, -8.49469F, -8.35551F, -8.2207F, -8.10203F, -7.99225F, -7.88541F, -7.69265F, -7.60631F, -7.5219F, -7.43936F, -7.3643F, -7.29489F, -7.22678F, -7.15994F, -7.09432F, -7.034F, -6.97847F, -6.92381F, -6.87F, -6.81703F, -6.76485F, -6.71806F, -6.67376F, -6.58664F, -6.54406F, -6.50202F, -6.46168F, -6.42613F, -6.39098F, -6.32156F, -6.28755F, -6.25394F });
	pcModelBox model_p2_10 = pcModelBox(TagRange::SL, TagMinDistance::Nine, { -17.2788F, -17.2162F, -17.1448F, -17.0619F, -16.9641F, -16.8598F, -16.7325F, -16.5976F, -16.4459F, -16.2729F, -16.0869F, -15.6441F, -15.3843F, -15.094F, -14.7638F, -5.9909F, -5.99143F, -5.99195F, -5.99247F, -5.993F, -5.99352F, -5.99404F, -5.99441F, -5.99494F, -5.99546F, -5.99598F, -5.9965F, -5.99703F, -5.99755F, -5.99808F, -7.50437F, -7.93128F, -8.27027F, -8.82174F, -9.04853F, -9.26769F, -9.45773F, -9.64142F, -9.80868F, -9.96642F, -10.1122F, -10.2498F, -10.3757F, -10.4971F, -10.6047F, -10.7128F, -10.8032F, -10.8951F, -10.9744F, -11.0496F, -11.1798F, -11.2401F, -11.2894F, -11.3349F, -11.3795F, -11.4113F, -11.4701F, -11.4891F, -11.5081F });
	pcModelBox model_p2_11 = pcModelBox(TagRange::SL, TagMinDistance::Nine, { -17.2647F, -17.2023F, -17.1274F, -17.0445F, -16.943F, -16.8355F, -16.7078F, -16.5687F, -16.4177F, -16.2405F, -16.0494F, -15.6025F, -15.3366F, -15.0396F, -14.7004F, -14.3011F, -13.808F, -13.0855F, -11.9798F, -11.9811F, -11.9824F, -11.9837F, -11.9841F, -5.48185F, -5.48243F, -5.48302F, -5.48361F, -5.48419F, -5.48478F, -5.48537F, -5.48595F, -5.48654F, -5.48713F, -5.4881F, -5.48869F, -5.48928F, -5.48988F, -5.49033F, -5.49092F, -5.49152F, -5.49212F, -5.49271F, -5.49331F, -5.49392F, -5.49452F, -5.49512F, -5.49573F, -5.49634F, -5.49695F, -5.49756F, -5.49855F, -5.49917F, -5.49979F, -5.50041F, -5.50103F, -5.50166F, -5.50267F, -5.5033F, -5.50393F });
};