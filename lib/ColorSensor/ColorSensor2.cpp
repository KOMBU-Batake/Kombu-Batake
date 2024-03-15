#include "ColorSensor2.h"

XZcoordinate ColorSensor2::obstacle()
{
	colorCam->saveImage("1.png", 100);

	// �]���̒ǉ�
	copyMakeBorder(inputImage, inputImage, 1, 0, 10, 10, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255, 0));

	// �o�͉摜��ۑ�
	cv::imwrite("output_image.png", inputImage); // ��: "output_image.png" ��K�؂ȃt�@�C�����ɒu�������Ă�������

	//�ǂݍ��񂾉摜�̎l�p�`�̒��_
	cv::Point2f pts1[] = { cv::Point2f(8, 28), cv::Point2f(75,28), cv::Point2f(56, 0), cv::Point2f(27, 0) };
	//�o�͉摜�ɑΉ�����l�p�`�̒��_
	cv::Point2f pts2[] = { cv::Point2f(24, 36), cv::Point2f(60, 36), cv::Point2f(60, 0), cv::Point2f(24, 0) };
	//�ˉe�ό`�s�����
	cv::Mat pmat = cv::getPerspectiveTransform(pts1, pts2);
	//�ˉe�ϊ���R�����͕ϊ��s��
	cv::warpPerspective(inputImage, inputImage, pmat, inputImage.size(), cv::INTER_NEAREST);

	vector<ImageXZcoordinate> LiDARPitns = lidar2.getPositionOfImage();

	cv::imwrite("output_image1.png", inputImage); // ��: "output_image.png" ��K�؂ȃt�@�C�����ɒu�������Ă�������
}
