#include "ColorSensor2.h"

static XZcoordinate convertImageXZtoXZ_start(vector<ImageXZcoordinate>::iterator p) {
	return XZcoordinate{ -6.0f + (12.0f / 37.0f) * (float)(p->x - 24), 18.0f - (12.0f / 37.0f) * p->z };
}

static XZcoordinate convertImageXZtoXZ_end(ImageXZcoordinate& p) {
	return XZcoordinate{ -6.0f + (12.0f / 37.0f) * (p.x - 23), 18.0f - (12.0f / 37.0f) * (p.z + 1) };
}

void ColorSensor2::nextTile(obstacleState& state, vector<int>& numbers)
{
	// ���ʉ�6cm,�c8cm���󂢂Ă��邩�ǂ���
	for (auto& num : numbers) {
		XZcoordinate p = lidar2.readPoint(num);
		if (abs(p.x) < 3 && p.z < 8) { // 1�ł��ΏۊO������ΏI��
			return; 
		}
	}

	// left
	vector<uchar> leftFloorBrightness(9);
	for (int i = 23; i <= 25; i++) {
		for (int j = 25; j <= 27; j++) {
			Vec4b pixel = inputImage.at<cv::Vec4b>(i, j);
			leftFloorBrightness[(i - 23) * 3 + (j - 25)] = max(pixel[0], max(pixel[1], pixel[2]));
		}
	}
	if (calculateVariance(leftFloorBrightness) == 0) {
		if (leftFloorBrightness[0] == 30) {
			state.leftHoleState = true;
			cout << "left hole" << endl;
		}
	}

	// right
	vector<uchar> rightFloorBrightness(9);
	for (int i = 23; i <= 25; i++) {
		for (int j = 36; j <= 38; j++) {
			Vec4b pixel = inputImage.at<cv::Vec4b>(i, j);
			rightFloorBrightness[(i - 23) * 3 + (j - 36)] = max(pixel[0], max(pixel[1], pixel[2]));
		}
	}
	if (calculateVariance(rightFloorBrightness) == 0) {
		if (rightFloorBrightness[0] == 30) {
			state.rightHoleState = true;
			cout << "right hole" << endl;
		}
	}
}

obstacleState ColorSensor2::obstacle()
{
	colorCam->saveImage("1.png", 100);

	// �]���̒ǉ�
	copyMakeBorder(inputImage, transformedImage, 1, 0, 10, 10, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255, 0));

	//�ǂݍ��񂾉摜�̎l�p�`�̒��_
	cv::Point2f pts1[] = { cv::Point2f(8, 28), cv::Point2f(75,28), cv::Point2f(56, 0), cv::Point2f(27, 0) };
	//�o�͉摜�ɑΉ�����l�p�`�̒��_
	cv::Point2f pts2[] = { cv::Point2f(24, 36), cv::Point2f(60, 36), cv::Point2f(60, 0), cv::Point2f(24, 0) };
	//�ˉe�ό`�s�����
	cv::Mat pmat = cv::getPerspectiveTransform(pts1, pts2);
	//�ˉe�ϊ���R�����͕ϊ��s��
	cv::warpPerspective(transformedImage, transformedImage, pmat, transformedImage.size(), cv::INTER_NEAREST);

	cv::imwrite("output_image.png", transformedImage);

	/* ��Q���̔��� */
	obstacleState obstacleState;
	vector<int> numbers = lidar2.getNcmNumbers(LiDAR_degree::FRONT, 12); // 12cm�͈̔͂ł̓_�̔ԍ����擾
	vector<ImageXZcoordinate> LiDARPoints = lidar2.getPositionOfImage(numbers);
	vector<ImageXZcoordinate> ObstaclePoints;
	int last_updated_num = -10;
	for (int i = 0; i < LiDARPoints.size(); i++) {
		if (LiDARPoints[i].z < 5) continue; // ����Ώۂ��͈͊O�ɂ���ꍇ�̓X�L�b�v
		Vec4b topPixel = transformedImage.at<Vec4b>(2, LiDARPoints[i].x);
		if (convertRGBtoHSV(topPixel).saturation != 0) continue; // �F�t��
		if (topPixel == Vec4b{ 192,192,192,255 }) continue; // ���F
		Vec4b Pixel, lastPixel = topPixel;
		int diff_count = 0;
		for (int j = 3; j < LiDARPoints[i].z - 3; j++) { // �F�̕ω�����񐔂��J�E���g
			Vec4b Pixel = transformedImage.at<Vec4b>(j, LiDARPoints[i].x);
			if ((int)Pixel[4] != 255) break; // ������r��
			if (Pixel != lastPixel) {
				diff_count++;
				if (diff_count > 3) break;
			}
			lastPixel = Pixel;
		}
		if (diff_count > 3) continue; // �ω�����񐔂������ꍇ�̓`�F�b�N�|�C���g�^�C��
		ObstaclePoints.push_back(LiDARPoints[i]);
		cout << LiDARPoints[i].z << ", " << LiDARPoints[i].x - 3 << ", saturation: " << convertRGBtoHSV(topPixel).saturation << endl;
	}

	if (ObstaclePoints.size() == 0) { // ��Q���̌�₪�Ȃ��ꍇ�͖��E
		nextTile(obstacleState,numbers);
		return obstacleState;
	}

	// �ʒu���Ƃɕ�����
	vector<vector<ImageXZcoordinate>> ObstaclePositions;
	ObstaclePositions.push_back(vector<ImageXZcoordinate>{ObstaclePoints[0]});
	for (int i = 1; i < ObstaclePoints.size(); i++) {
		if ((ObstaclePoints[i].x - ObstaclePoints[i - 1].x) <= 4) {
			ObstaclePositions[ObstaclePositions.size() - 1].push_back(ObstaclePoints[i]);
		}
		else {
			ObstaclePositions.push_back(vector<ImageXZcoordinate>{ObstaclePoints[i]});
			cout << "new" << endl;
		}
	}

	// ���ۂ̐��E�̍��W�ɕϊ�
	for (auto& Positions : ObstaclePositions) {
		if (Positions.size() > 2) { // 2�_�ȉ��̏ꍇ�͖���
			obstacleState.obstaclePositionsStart.push_back(convertImageXZtoXZ_start(Positions.begin()));
			cout << "start Image: " << Positions.begin()->x << ", " << Positions.begin()->z << endl;
			obstacleState.obstaclePositionsEnd.push_back(convertImageXZtoXZ_end(Positions.back()));
			cout << "end Image: " << Positions.back().x << ", " << Positions.back().z << endl;
		}
	}

	// �ł��߂��_��T��
	for (int i = 0; i < obstacleState.obstaclePositionsStart.size(); i++) {
		XZcoordinate last_closest = { 1000,1000 };
		for (auto pointNum : numbers) {
			XZcoordinate p = lidar2.readPoint(pointNum);
			if (p.x >= obstacleState.obstaclePositionsStart[i].x && p.x <= obstacleState.obstaclePositionsEnd[i].x &&
				p.z < last_closest.z) {
				last_closest = p;
			}
		}
		obstacleState.closestPoint.push_back(last_closest);
	}

	for (auto& start : obstacleState.obstaclePositionsStart) {
		cout << "start: " << start.x << ", " << start.z << endl;
	}
	for (auto& end : obstacleState.obstaclePositionsEnd) {
		cout << "end: " << end.x << ", " << end.z << endl;
	}
	for (auto& closest : obstacleState.closestPoint) {
		cout << "closest: " << closest.x << ", " << closest.z << endl;
	}

	nextTile(obstacleState,numbers);
	return obstacleState;
}
