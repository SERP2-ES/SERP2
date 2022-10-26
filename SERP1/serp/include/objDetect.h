#pragma once
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>

const double pi = 3.14159265358979323846;

using namespace std;
using namespace cv;

//RNG rng(12345);


//contantes para dete��o de objetos
const int minThreshold = 240;
const int threshMultiplier = 3;
const int kernelSize = 3;
const int minRadiusFilter = 60;

//valores para a segmentação da imagem em regiões de interesse
//frente
const int N_beginningX = 90;
const int N_width = 420;
const int N_endingX = N_beginningX + N_width;
const int N_beginningY = 400;
const int N_height = 400;
//frente direita
const int NE_beginningX = 95;
const int NE_width = 470;
const int NE_endingX = NE_beginningX + NE_width;
const int NE_beginningY = 1;
const int NE_height = 400;

//frente esquerda
const int NW_beginningX = 90;
const int NW_width = 470;
const int NW_endingX = NW_beginningX + NW_width;
const int NW_beginningY = 800;
const int NW_height = 400;

//atrás
const int S_beginningX = 1200;
const int S_width = 300;
const int S_endingX = S_beginningX + S_width;
const int S_beginningY = 350;
const int S_height = 450;




//inicializa��es vari�veis de sensores
uint8_t N = 0, NE = 0, NW = 0, S = 0;
// N=Frente
// NE=Frente Direita
// NW=Frente Esquerda
// S=Tr�s
// SE=Tr�s Esquerda
// SW=Tr�s Direita

//fun��o que calcula o n�vel de per�go de objeto dependendo da sua dist�ncia ao robo. 0 min, 3 max
uint8_t levelDanger(float n)
{
    //cout << "distancia" << n << endl;
    
    if ((n <= 420) && (n > 200))
        return 1;

    else if ((n <= 200) && (n > 100))
        return 2;

    else if ((n <= 100) && (n >= 0))
        return 3;
    else
        return 0;

}



//filtra os objetos detetados, e identifica qual o mais pr�ximo, onde calcula o seu perigo
void sortDetectedObjects(String pos, Mat img, vector<Point2f> centers, vector<float> radius, vector<Rect> rectangles)
{
    int height = img.rows;
    int width = img.cols;

    float distQuant = 0;
    float minDist = 1000;

    if (pos.compare("N") == 0) {

        for (size_t i = 0; i < centers.size(); i++)
        {

            if ((float)radius[i] > minRadiusFilter) {

                distQuant = abs(rectangles[i].br().x - N_width);

                if (distQuant < minDist) {
                    minDist = distQuant;
                }

            }
        }

        N = levelDanger(minDist);

    }
    else if (pos.compare("S") == 0) {

        for (size_t i = 0; i < centers.size(); i++)
        {
            if ((float)radius[i] > minRadiusFilter) {

                distQuant = abs(rectangles[i].tl().x - 0);

                cout << distQuant << "atras distancia" << endl;

                if (distQuant < minDist) {
                    minDist = distQuant;
                }

            }
        }
        cout << distQuant << "atras distancia" << endl;
        S = levelDanger(minDist);

    }
    else if (pos.compare("NE") == 0) {

        for (size_t i = 0; i < centers.size(); i++)
        {
            if ((float)radius[i] > minRadiusFilter) {

                distQuant = sqrt(pow(abs(rectangles[i].br().x - NE_width), 2) + pow(abs(rectangles[i].tl().y - 400), 2));

                if (distQuant < minDist) {
                    minDist = distQuant;
                }

            }
        }

        NE = levelDanger(minDist);

    }
    else if (pos.compare("NW") == 0) {

        for (size_t i = 0; i < centers.size(); i++)
        {
            if ((float)radius[i] > minRadiusFilter) {

                distQuant = sqrt(pow(abs(rectangles[i].br().x - NW_width), 2) + pow(abs(rectangles[i].br().y - 1), 2));

                if (distQuant < minDist) {
                    minDist = distQuant;
                }

            }
        }

        NW = levelDanger(minDist);

    }

}

//trabalho incacabado de elemina��o de circulos dentro de circulos
bool inRadious(Point2f centerOrigin, Point2f center2, int r) {

    float distBetweenCenters = sqrt(pow(abs((double)centerOrigin.x - center2.x), 2) + pow(abs((double)centerOrigin.y - center2.y), 2));

    if (distBetweenCenters <= r)
        return true;
    else
        return false;
}


//Encontrar objetos na imagem, e guardar os centros e os raios em vatores
void detectObj(Mat img, int lowThreshold, int threshMult, int kernel_size, String pos)
{
    Mat equalized, detected_edges;

    float sumX = 0, sumY = 0, sumR = 0;
    int num = 0;

    Mat img_grey;
    //preprocessing on image
    //pyrMeanShiftFiltering(img, img, 10, 20);
    //medianBlur(img, img, 27);

    int erosion_size = 3;

    Mat element = getStructuringElement(MORPH_ELLIPSE,
        Size(2 * erosion_size + 1, 2 * erosion_size + 1),
        Point(erosion_size, erosion_size));

    morphologyEx(img, img,
        MORPH_GRADIENT, element,
        Point(-1, -1), 1);


    cvtColor(img, img_grey, COLOR_BGR2GRAY);
    blur(img_grey, img_grey, Size(5, 5));

    equalizeHist(img_grey, equalized);


    Canny(equalized, detected_edges, lowThreshold, lowThreshold * threshMult, kernel_size);

    vector<vector<Point> > contours;
    findContours(detected_edges, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
    vector<vector<Point> > contours_poly(contours.size());
    vector<Rect> boundRect(contours.size());
    vector<Point2f> centers(contours.size());
    vector<float> radius(contours.size());
    for (size_t j = 0; j < contours.size(); j++)
    {
        approxPolyDP(contours[j], contours_poly[j], 3, true);
        boundRect[j] = boundingRect(contours_poly[j]);
        minEnclosingCircle(contours_poly[j], centers[j], radius[j]);
    }
    //Mat drawing = Mat::zeros(detected_edges.size(), CV_8UC3);

    //eliminar deteção de objetos sobrepostas
    int i = 0;
    for (size_t i = 0; i < contours.size(); i++) {

        /*sumX = centers[i].x;
        sumY = centers[i].y;
        sumR = radius[i];
        num = 1;*/

        for (size_t j = 0; j < contours.size(); j++)
        {
            if ((i != j)) {

                if ((radius[j] != 0) && (inRadious(centers[i], centers[j], radius[i]))) {
                    /*sumX += centers[j].x;
                    sumY += centers[j].y;
                    sumR += radius[j];
                    radius[j] = 0;
                    num += 1;*/
                    if (radius[j] > radius[i])
                        radius[i] = 0;
                    else if (radius[j] <= radius[i])
                        radius[j] = 0;
                    
                }
            }
        }

    }


    //debug only. desenha contornos e circulos nos objetos detetados
     /*for (size_t j = 0; j < contours.size(); j++)
     {
         Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));

         if ((int)radius[j] > minRadiusFilter) {
             rectangle(img, boundRect[j].tl(), boundRect[j].br(), color, 2);
             circle(img, centers[j], (int)radius[j], color, 2);
         }
     }*/

    sortDetectedObjects(pos, img, centers, radius, boundRect);

    /*cv::imshow("Image ", img);
    cv::waitKey(0);*/
}


//fun��o que pega em cada frame do video, aplica pre processamento, segmenta a imagem e deteta os objetos em cada segmento da imagem
vector<uint8_t> frameProcessing(Mat frame) {

    N = 0;
    NW = 0;
    NE = 0;
    S = 0;

    //divisão dos frames em regiões de interece
    //ROI(pixelHorizontalInicio, PixelVerticalInicio, QuantidadePixeisHorizontal, QuantidadePixeisVertical)
    Rect ROI_NE(NE_beginningX, NE_beginningY, NE_width, NE_height);
    Mat image_NE = frame(ROI_NE);

    Rect ROI_N(N_beginningX, N_beginningY, N_width, N_height);
    Mat image_N = frame(ROI_N);

    Rect ROI_NW(NW_beginningX, NW_beginningY, NW_width, NW_height);
    Mat image_NW = frame(ROI_NW);

    Rect ROI_S(S_beginningX, S_beginningY, S_width, S_height);
    Mat image_S = frame(ROI_S);

    //deteta os objetos com os thresholds, e com o identificador de que parte �.
    detectObj(image_NE, minThreshold, threshMultiplier, kernelSize, "NE");
    detectObj(image_N, minThreshold, threshMultiplier, kernelSize, "N");
    detectObj(image_NW, minThreshold, threshMultiplier, kernelSize, "NW");
    detectObj(image_S, minThreshold, threshMultiplier, kernelSize, "S");

    //imprimir o valor dos sensores. 
    /*cout << "FD: " << NE << endl;
    cout << "FE: " << NW << endl;
    cout << "F: " << N << endl;
    cout << "T: " << S << endl;
    cout << "__________________________________________________________" << endl;

    return frame;*/

    //vetor para retornar valores dos sensores
    std::vector<uint8_t> retorna_valores{NE , N , S, NW};
    return retorna_valores;

}