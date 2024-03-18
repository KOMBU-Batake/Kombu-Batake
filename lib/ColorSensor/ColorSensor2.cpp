#include "ColorSensor2.h"

static XZcoordinate convertImageXZtoXZ_start(vector<ImageXZcoordinate>::iterator p) {
	return XZcoordinate{ -6.0f + (12.0f / 37.0f) * (float)(p->x - 24), 18.0f - (12.0f / 37.0f) * p->z };
}

static XZcoordinate convertImageXZtoXZ_end(vector<ImageXZcoordinate>::iterator p) {
	return XZcoordinate{ -6.0f + (12.0f / 37.0f) * (p->x - 23), 18.0f - (12.0f / 37.0f) * (p->z + 1) };
}

obstacleState ColorSensor2::obstacle()
{
	colorCam->saveImage("1.png", 100);

	// 余白の追加
	copyMakeBorder(inputImage, transformedImage, 1, 0, 10, 10, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255, 0));

	//読み込んだ画像の四角形の頂点
	cv::Point2f pts1[] = { cv::Point2f(8, 28), cv::Point2f(75,28), cv::Point2f(56, 0), cv::Point2f(27, 0) };
	//出力画像に対応する四角形の頂点
	cv::Point2f pts2[] = { cv::Point2f(24, 36), cv::Point2f(60, 36), cv::Point2f(60, 0), cv::Point2f(24, 0) };
	//射影変形行列を代入
	cv::Mat pmat = cv::getPerspectiveTransform(pts1, pts2);
	//射影変換第３引数は変換行列
	cv::warpPerspective(transformedImage, transformedImage, pmat, transformedImage.size(), cv::INTER_NEAREST);

	vector<ImageXZcoordinate> LiDARPoints = lidar2.getPositionOfImage();
	vector<ImageXZcoordinate> ObstaclePoints;
	int last_updated_num = -10;
	for (int i = 0; i < LiDARPoints.size(); i++) {
		if (LiDARPoints[i].z < 5) continue; // 測定対象が範囲外にある場合はスキップ
		Vec4b topPixel = transformedImage.at<Vec4b>(2, LiDARPoints[i].x);
		if (convertRGBtoHSV(topPixel).saturation != 0) continue; // 色付き
		if (topPixel == Vec4b{ 192,192,192,255 }) continue; // 白色
		Vec4b Pixel, lastPixel = topPixel;
		int diff_count = 0;
		for (int j = 3; j < LiDARPoints[i].z - 3; j++) { // 色の変化する回数をカウント
			Vec4b Pixel = transformedImage.at<Vec4b>(j, LiDARPoints[i].x);
			if ((int)Pixel[4] != 255) break; // 透明を排除
			if (Pixel != lastPixel) {
				diff_count++;
				if (diff_count > 3) break;
			}
			lastPixel = Pixel;
		}
		if (diff_count > 3) continue; // 変化する回数が多い場合はチェックポイントタイル
		ObstaclePoints.push_back(LiDARPoints[i]);
		cout << LiDARPoints[i].z << ", " << LiDARPoints[i].x - 3 << ", saturation: " << convertRGBtoHSV(topPixel).saturation << endl;
	}

	// 位置ごとに分ける
	vector<vector<ImageXZcoordinate>> ObstaclePositions;
	ObstaclePositions.push_back(vector<ImageXZcoordinate>{ObstaclePoints[0]});
	for (int i = 1; i < ObstaclePoints.size(); i++) {
		if ((ObstaclePoints[i].x - ObstaclePoints[i - 1].x) <= 4) {
			ObstaclePositions[ObstaclePositions.size() - 1].push_back(ObstaclePoints[i]);
		}
		else {
			ObstaclePositions.push_back(vector<ImageXZcoordinate>{ObstaclePoints[i]});
		}
	}

	// 実際の世界の座標に変換
	obstacleState obstacleState;
	for (auto& Positions : ObstaclePositions) {
		if (Positions.size() > 2) {
			obstacleState.obstaclePositionsStart.push_back(convertImageXZtoXZ_start(Positions.begin()));
			obstacleState.obstaclePositionsEnd.push_back(convertImageXZtoXZ_end(Positions.end()));
		}
	}

	cv::imwrite("output_image.png", transformedImage);

	for (auto& start : obstacleState.obstaclePositionsStart) {
		cout << "start: " << start.x << ", " << start.z << endl;
	}
	for (auto& end : obstacleState.obstaclePositionsEnd) {
		cout << "end: " << end.x << ", " << end.z << endl;
	}

	return obstacleState;
}
