#include "ColorSensor2.h"

XZcoordinate ColorSensor2::obstacle()
{
	colorCam->saveImage("1.png", 100);

	// 余白の追加
	copyMakeBorder(inputImage, inputImage, 1, 0, 10, 10, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255, 0));

	// 出力画像を保存
	cv::imwrite("output_image.png", inputImage); // 例: "output_image.png" を適切なファイル名に置き換えてください

	//読み込んだ画像の四角形の頂点
	cv::Point2f pts1[] = { cv::Point2f(8, 28), cv::Point2f(75,28), cv::Point2f(56, 0), cv::Point2f(27, 0) };
	//出力画像に対応する四角形の頂点
	cv::Point2f pts2[] = { cv::Point2f(24, 36), cv::Point2f(60, 36), cv::Point2f(60, 0), cv::Point2f(24, 0) };
	//射影変形行列を代入
	cv::Mat pmat = cv::getPerspectiveTransform(pts1, pts2);
	//射影変換第３引数は変換行列
	cv::warpPerspective(inputImage, inputImage, pmat, inputImage.size(), cv::INTER_NEAREST);

	vector<ImageXZcoordinate> LiDARPitns = lidar2.getPositionOfImage();

	cv::imwrite("output_image1.png", inputImage); // 例: "output_image.png" を適切なファイル名に置き換えてください
}
