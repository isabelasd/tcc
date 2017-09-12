#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPixmap>
#include <QFileDialog>
#include <QMessageBox>

Mat binary_image, image_gray;
Mat dst, detected_edges;

Mat cropped,cropped_new,cropped_blur, dilated, erosion_dst;

int edgeThresh = 1;
int lowThreshold = 11;
int const max_lowThreshold = 100;
int razao = 3;
int kernel_size = 3;

int dilation_elem = 2;
int dilation_size = 3;
int erosion_elem = 2;
int erosion_size = 2;
int const max_elem = 2;
int const max_kernel_size = 21;

bool warm_hand = false;




Mat frame , frame_threshold;
Mat image2, newImage;

void on_high_r_thresh_trackbar(int, void *);
void on_high_g_thresh_trackbar(int, void *);
void on_high_b_thresh_trackbar(int, void *);

void on_low_r_thresh_trackbar(int, void *);
void on_low_g_thresh_trackbar(int, void *);
void on_low_b_thresh_trackbar(int, void *);

int low_r=0, low_g=0, low_b=0;
int high_r=1, high_g=1, high_b=1;


void on_low_r_thresh_trackbar(int, void *)
{
    low_r = min(high_r-1, low_r);
    setTrackbarPos("Low R","Object Detection", low_r);

    inRange(newImage,Scalar(low_b,low_g,low_r), Scalar(high_b,high_g,high_r),frame_threshold);

    imshow("Object Detection",frame_threshold);
}
void on_high_r_thresh_trackbar(int, void *)
{
    high_r = max(high_r, low_r+1);
    setTrackbarPos("High R", "Object Detection", high_r);

    inRange(newImage,Scalar(low_b,low_g,low_r), Scalar(high_b,high_g,high_r),frame_threshold);
    imshow("Object Detection",frame_threshold);
}
void on_low_g_thresh_trackbar(int, void *)
{
    low_g = min(high_g-1, low_g);
    setTrackbarPos("Low G","Object Detection", low_g);

    inRange(newImage,Scalar(low_b,low_g,low_r), Scalar(high_b,high_g,high_r),frame_threshold);
    imshow("Object Detection",frame_threshold);
}
void on_high_g_thresh_trackbar(int, void *)
{
    high_g = max(high_g, low_g+1);
    setTrackbarPos("High G", "Object Detection", high_g);

    inRange(newImage,Scalar(low_b,low_g,low_r), Scalar(high_b,high_g,high_r),frame_threshold);
    imshow("Object Detection",frame_threshold);
}
void on_low_b_thresh_trackbar(int, void *)
{
    low_b= min(high_b-1, low_b);
    setTrackbarPos("Low B","Object Detection", low_b);

    inRange(newImage,Scalar(low_b,low_g,low_r), Scalar(high_b,high_g,high_r),frame_threshold);
    imshow("Object Detection",frame_threshold);
}
void on_high_b_thresh_trackbar(int, void *)
{
    high_b = max(high_b, low_b+1);
    setTrackbarPos("High B", "Object Detection", high_b);

    inRange(newImage,Scalar(low_b,low_g,low_r), Scalar(high_b,high_g,high_r),frame_threshold);
    imshow("Object Detection",frame_threshold);
}

void ascending_sort(int array[], int arraySize)
{
    int temp;
    for(int i = 0; i < arraySize; i++)
    {
          for(int j = 1; j < arraySize-1; j++)
          {
                   if(array[j] > array[i])
                   {
                        //swap them
                       temp = array[i];
                       array[i] = array[j];
                       array[j] = temp;
                    }
           }
    }

}




void shift_r(int arr[], int n)
{
  int i, temp;
  temp = arr[0];
  for (i = 0; i < n-1; i++)
     arr[i] = arr[i+1];
  arr[i] = temp;
}

double distance (int x1 , int x2, int y1 , int y2)
{
    return sqrt( (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) ) ;

}


void on_trackbar( int, void* )
{
    blur( image_gray, binary_image, Size(3,3) );
    Canny( binary_image, binary_image, lowThreshold, lowThreshold*razao, kernel_size );
      //createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );



    imshow("window", binary_image) ;
}

bool compareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 )
{
    double i = fabs( contourArea(cv::Mat(contour1)) );
    double j = fabs( contourArea(cv::Mat(contour2)) );
    return ( i < j );
}

int findBiggestContour(vector<vector<Point> > contours)
{
    int indexOfBiggestContour = -1;
    int sizeOfBiggestContour = 0;

    for (int i = 0; i < contours.size(); i++){
        if(contours[i].size() > sizeOfBiggestContour){
            sizeOfBiggestContour = contours[i].size();
            indexOfBiggestContour = i;
        }
    }
    return indexOfBiggestContour;
}

int findSmallestValue(int array[], int arraySize)
{
    int minimumValue = array[0];
    int minimumIndex = 0 ;

        for ( int i = 1 ; i < arraySize ; i++ )
        {
            if ( array[i] < minimumValue )
            {
               minimumValue = array[i];
               minimumIndex = i+1;
            }
        }
    return minimumIndex ;
}

int getSmallestX(int number1 , int number2)
{
    qDebug() << "\n\n number 1 : " << number1 << " number 2 : " << number2 ;
    if (number1 < number2 )
    {
        return 1 ;
    }
    else
    {
        return 0 ;
    }
}


void Erosion (int, void*)
{

    int erosion_type;
      if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
      else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
      else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

      Mat element = getStructuringElement( erosion_type,
                           Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                           Point( erosion_size, erosion_size ) );

      // Apply the erosion operation
      erode( dilated, erosion_dst, element );
      imshow( "canny_image", erosion_dst );

      // contours
      vector<vector<Point> > contours;
      vector<Vec4i> hierarchy;
      RNG rng(12345);
      findContours( erosion_dst, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

      // begin of teest



      vector<vector<Point>> hull( contours.size() );
      vector<vector<Point>> hullsI( contours.size() );
      vector<vector<Point>> contours_poly( contours.size() );
      vector<vector<Vec4i>> defects( contours.size()) ;
      Point2f rect_points[4];
      vector<RotatedRect> minRect( contours.size() );
      vector<Rect> boundRect( contours.size() );

      Mat frame2 = Mat::zeros( erosion_dst.size(), CV_8UC3 );
      int largest_contour_index;
      largest_contour_index = findBiggestContour(contours);

      for( int i = 0; i < contours.size(); i++ )
           {
               convexHull( Mat(contours[i]), hull[i], false );
               convexHull( Mat(contours[i]), hullsI[i], true );
               //convexityDefects(Mat(contours[i]),hullsI[i], defects[i]);

                  if(largest_contour_index == i)
                     {
                       // minRect[i] = minAreaRect( Mat(contours[i]) );

                       //draw contour of biggest object
                        drawContours( frame2, contours,largest_contour_index, CV_RGB(255,255,255), 1, 8, vector<Vec4i>(),0, Point() );
                      //draw hull of biggesr object
                        drawContours( frame2, hull, largest_contour_index, CV_RGB(255,0,0), 1, 8, vector<Vec4i>(), 0, Point() );


                        approxPolyDP( Mat(hull[i]), contours_poly[i], 2, true );
                        boundRect[i] = boundingRect( Mat(contours_poly[i]) );
                        rectangle( frame2, boundRect[i].tl(), boundRect[i].br(), CV_RGB(0,255,0), 1, 8, 0 );




                     }
           }


      int PontoX = 0 ;
      int PontoY = 0 ;
      int markers_aux_X[hullsI.size()] = {};
      int markers_aux_Y[hullsI.size()] = {};
        //Point
      for (int a = 0 ; a < hullsI[largest_contour_index].size() ; a ++ )
      {
          PontoX = hullsI[largest_contour_index][a].x ;
          PontoY = hullsI[largest_contour_index][a].y ;
          markers_aux_X[a] = PontoX ;
          markers_aux_Y[a] = PontoY ;
          qDebug() << "ponto[" << a << "]: " << markers_aux_X[a] << "," << markers_aux_Y[a];

          circle( frame2, hullsI[largest_contour_index][a], 4, Scalar(0,0,255), 1, 8, 0 );

      }

      //sort(markers_aux_Y.begin(), markers_aux_Y.end() )
      //ascending_sort(markers_aux_Y, hullsI[largest_contour_index].size());
      int index_first = 0 ;
      index_first = findSmallestValue (markers_aux_Y , hullsI[largest_contour_index].size()) ;
      qDebug() << "minimum " << index_first ;
      qDebug() << "numero 1 : " << markers_aux_X[index_first-1] << " numero 2 " << markers_aux_X[index_first] ;
      int update_index_x = 0 ;
      update_index_x = getSmallestX (markers_aux_X[index_first-1] , markers_aux_X[index_first] ) ;

      // index of smallest x with y = 0 point
      index_first = index_first - update_index_x ;
      qDebug() << ", new minimum : " << index_first ;

      int shift = 0 ;
      //shift = hullsI[largest_contour_index].size() - index_first ;
      shift = index_first ;
      qDebug() << "\nshift : " << shift ;

      int x_aux_2[hullsI[largest_contour_index].size()] ;
      for (int i = 0 ; i < hullsI[largest_contour_index].size() ; i ++)
      {
          x_aux_2[i] = 0 ;
      }

      for (int i = 0 ; i < hullsI[largest_contour_index].size() ; i ++)
      {
          x_aux_2[i] = markers_aux_X[i] ;
      }

      for ( int i = 0 ; i < 5/*shift*/ ; i ++ )
      {
          int a, temp;
          temp = x_aux_2[0];
          for (a = 0; a <  hullsI[largest_contour_index].size()-1; a++)
          {
             x_aux_2[a] = x_aux_2[a+1];
          }
          x_aux_2[a] = temp;
          //shift_r (markers_aux_X, hullsI[largest_contour_index].size());

        //shift_r (markers_aux_Y, hullsI[largest_contour_index].size());
      }


      for (int a = 0 ; a < hullsI[largest_contour_index].size() ; a ++ )
      {
          qDebug() << "ponto[" << a << "]: " << x_aux_2[a] << "," << markers_aux_Y[a];
      }

     /* for (int a = 0 ; a < hullsI[largest_contour_index].size() ; a ++ )
      {
            qDebug() << "ponto[" << a << "]: " << markers_aux_X[a] << "," << markers_aux_Y[a];
      }

      */







        namedWindow( "trial", WINDOW_AUTOSIZE );
        imshow( "trial", frame2 );


        for (int a = 0 ; a < contours_poly[largest_contour_index].size() ; a ++ )
        {
            circle( frame2, contours_poly[largest_contour_index][a], 4, Scalar(0,255,255), 1, 8, 0 );
        }

        namedWindow( "trialPoly", WINDOW_AUTOSIZE );
        imshow( "trialPoly", frame2 );



        qDebug() <<"\n" << hullsI.size();
        qDebug() << "\n" << hullsI[largest_contour_index].size() ;
        qDebug() << "\npolyHull :" << contours_poly[largest_contour_index].size() ;

        //qDebug() <<"\n" << hull.size();

        /*
        for ( int i = 0 ; i < hull.size() ; i++)
        {
            for ( int j = 0 ; j < hull[i].size() ; j++)
            {
                qDebug << "\n hull "  <<  QString::number(hull[i][j]) ;
            }
        }


        for ( int i = 0 ; i < hullsI.size() ; i++)
        {
            for ( int j = 0 ; j < hullsI[i].size() ; j++)
            {
                qDebug << "\n hull si "  <<  QString::number(hullsI[i][j]) ;
            }
        }
        */












      // convex hull
/*
      // Find the convex hull object for each contour
         vector<vector<Point> >hull( contours.size() );
         for( int i = 0; i < contours.size(); i++ )
         {
             convexHull( Mat(contours[i]), hull[i], false );
         }

      Mat drawing = Mat::zeros( erosion_dst.size(), CV_8UC3 );
         for( int i = 0; i< contours.size(); i++ )
            {
              Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
              drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
              drawContours( drawing, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
              //circle( drawing, hull_points[i], 4, Scalar(0,0,255), -1, 8, 0 );
            }

         // Show in a window
         namedWindow( "Hull demo", WINDOW_AUTOSIZE );
         imshow( "Hull demo", drawing );




         */
       //  ---------------------
         // largest area trial
         /*
         int largest_area=0;
          int largest_contour_index=0;
          Rect bounding_rect;

         for( int i = 0; i< hull.size(); i++ ) // iterate through each contour.
               {
                double a=contourArea( hull[i],false);  //  Find the area of contour
                if(a>largest_area){
                largest_area=a;
                largest_contour_index=i;                //Store the index of largest contour
                bounding_rect=boundingRect(hull[i]); // Find the bounding rectangle for biggest contour
                }

               }
         Mat drawing_center = Mat::zeros( drawing.size(), CV_8UC3 );

          Scalar color( 0,0,255);
          drawContours( drawing_center, hull,largest_contour_index, color, CV_FILLED, 8, hierarchy ); // Draw the largest contour using previously stored index.
          circle( cropped, mc[largest_contour_index], 4, Scalar(255,0,0), -1, 8, 0 );
          rectangle(drawing, bounding_rect,  Scalar(0,255,0),1, 8,0);
          //imshow( "src", src );
          //imshow( "largest Contour", dst );

          // Show in a window
          namedWindow( "contour", WINDOW_AUTOSIZE );
          imshow( "contour", drawing_center );
          imshow("cropped", cropped);
*/

         // center mass trial
         //does not work

         /*
         // sort contours
         std::sort(hull.begin(), hull.end(), compareContourAreas);

         // grab contours
         std::vector<cv::Point> biggestContour = hull[hull.size()-1];

         Mat drawing_center = Mat::zeros( drawing.size(), CV_8UC3 );

         Scalar color( 0,0,255);
         polylines(drawing_center, biggestContour, true, color, 1, 8);

         // Get the moments
           vector<Moments> mu(hull.size() );
           for( int i = 0; i < hull.size(); i++ )
              { mu[i] = moments( hull[i], false ); }

           //  Get the mass centers:
           vector<Point2f> mc( hull.size() );
           for( int i = 0; i < hull.size(); i++ )
              { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }

           circle( drawing_center, mc[hull.size()-1], 4, Scalar(255,0,0), -1, 8, 0 );
           circle( cropped, mc[hull.size()-1], 4, Scalar(255,0,0), -1, 8, 0 );

          // Show in a window
          namedWindow( "contour", WINDOW_AUTOSIZE );
          imshow( "contour", drawing_center );
          imshow("cropped", cropped);
*/
         Mat center_test;
         int dilation_size = 1 ;
         Mat element2 = getStructuringElement( MORPH_ELLIPSE,
                                              Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                              Point( dilation_size, dilation_size ) );
         // Apply the dilation operation
         dilate( erosion_dst , center_test , element2 );
         namedWindow( "center", WINDOW_AUTOSIZE );
         imshow( "center", center_test );


         // get the mass center of a black and white image
         // another try for mass center
         Moments m = moments(center_test, false);
         Point p1(m.m10/m.m00, m.m01/m.m00);
         Point delta(0,40);
         Point p;
         p = p1 - delta ;

         circle(center_test, p, 5, Scalar(128,0,0), -1);
         circle(cropped, p, 5, Scalar(128,0,0), -1);
         imshow("center", center_test);
         imshow("cropped", cropped);




}

void Dilation( int, void* )
{
  int dilation_type;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  // Apply the dilation operation
  dilate( cropped_new, dilated, element );
  imshow( "canny_image", dilated );

  // contours
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  RNG rng(12345);
  findContours( dilated, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );


  // convex hull

  // Find the convex hull object for each contour
     vector<vector<Point> >hull( contours.size() );
     for( int i = 0; i < contours.size(); i++ )
     {
         convexHull( Mat(contours[i]), hull[i], false );
     }

     // Get the moments
       vector<Moments> mu(hull.size() );
       for( int i = 0; i < hull.size(); i++ )
          { mu[i] = moments( hull[i], false ); }

       //  Get the mass centers:
       vector<Point2f> mc( hull.size() );
       for( int i = 0; i < hull.size(); i++ )
          { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }





  Mat drawing = Mat::zeros( dilated.size(), CV_8UC3 );
     for( int i = 0; i< contours.size(); i++ )
        {
          Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
          drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
          drawContours( drawing, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
          //circle( drawing, mc[i], 4, Scalar(0,0,255), -1, 8, 0 );
        }
     //rectangle(drawing,mc[i], Point(coordinates[2],coordinates[3]), Scalar(0,0,0), 1);


     /// Show in a window
     namedWindow( "Hull demo", WINDOW_AUTOSIZE );
     imshow( "Hull demo", drawing );

}

void canny_thres( int, void* )
{
    // only updates de view when trackbar is again moved.
    // Canny detector
      if (warm_hand)
      {
          lowThreshold = 45 ;
      }
      else
      {
          lowThreshold = 11 ;
      }
      GaussianBlur( cropped, cropped_blur, Size(3,3) , 1, 1 );
      //blur( cropped, cropped_blur, Size(3,3) );
      Canny( cropped_blur, cropped_new, lowThreshold, lowThreshold*razao, kernel_size );

      imshow( "canny_image", cropped_new );

}

void removeBackground( Mat &image, int color_means[])
{

    int range = 30 ;

    qDebug() << " b: " << QString::number(color_means[0]) << " g: " << QString::number(color_means[1]) << " r: " << QString::number(color_means[2]) ;

    for(int i = 0 ; i < image.cols ; i++ )
    {
        for(int j = 0 ; j < image.rows ; j++)
        {
            Vec3b auxBGR = image.at<Vec3b>(j,i);
            //Point3_<uchar>* p = image.ptr<Point3_<uchar> >(y,x);
            uchar blue = auxBGR.val[0];
            uchar green = auxBGR.val[1];
            uchar red = auxBGR.val[2];

            int iblue = (int)blue;
            int igreen = (int)green;
            int ired = (int)red;

            auxBGR.val[0] = 0 ;
            auxBGR.val[1] = 0 ;
            auxBGR.val[2] = 0 ;

            if (iblue > color_means[0] - range && iblue < color_means[0] + range)
            {
                if (igreen > color_means[1] - 30 && igreen < color_means[1] + 30)
                {
                    if (ired > color_means[2] - 30 && ired < color_means[2] + 30)
                    {
                        auxBGR.val[0] = 255 ;
                        auxBGR.val[1] = 255 ;
                        auxBGR.val[2] = 255 ;
                    }
                }
            }

            image.at<Vec3b>(j,i)[0] = auxBGR.val[0];
            image.at<Vec3b>(j,i)[1] = auxBGR.val[1];
            image.at<Vec3b>(j,i)[2] = auxBGR.val[2];
         }
      }







}





// -- end of image processing



// FIND NICE WAY TO DECLARE LOOKUP TABLE !!! PLEZ :)
int R_channel[ SCALE_EL ]   = {   0,   2,   6,  18,  44,  90, 153, 216, 253, 244, 195, 131 } ;
int G_channel[ SCALE_EL ]   = {  14,  36,  78, 138, 203, 248, 250, 209, 144,  82,  39,  16 } ;
int B_channel[ SCALE_EL ]   = { 127, 189, 241, 254, 221, 159,  95,  47,  19,   6,   2,   0 } ;
int temperature[ SCALE_EL ] = {  16,  18,  20,  22,  24,  26,  28,  30,  32,  34,  36,  38 } ;

/* ------------------------------------------------------
 * MY FUNCTIONS
 * ----------------------------------------------------- */
// get average of a rectangle using a grayscale image
// image : grayscale image
// top left coordinate : (topLeftX, topLeftY)
// bottomRight coordinate : (bottomRightX, bottomRightY)

int MainWindow::getIntensityGray (const Mat &image, int topLeftX, int topLeftY, int bottomRightX, int bottomRightY)
{
    int intensity = 0;
    int aux = 0;
    int el_count = 0 ;

    for (int i = topLeftX + 1; i < bottomRightX; i++ )
    {
        for (int j = topLeftY + 1 ; j < bottomRightY; j++ )
        {
            aux = image.at<uchar>(Point(i,j));
            intensity = intensity + aux ;
           // aux = 0;
            //qDebug() << "\n intensity_" << i << "_" << j << ": " << QString::number(aux) << " intensity_var : " << intensity ;
            //intensity = 0 ;
            el_count++;
        }
    }
    intensity = intensity/el_count;
    return intensity;


}


// get average of a rectangle using a BGR image
// image : BGR image
// top left coordinate : (topLeftX, topLeftY)
// bottomRight coordinate : (bottomRightX, bottomRightY)

void MainWindow::getIntensityBGR (const Mat &image, int topLeftX, int topLeftY, int bottomRightX, int bottomRightY, int intensity_aux[])
{
    int el_count = 0 ;
    int intensityB = 0 ;
    int intensityG = 0 ;
    int intensityR = 0 ;


    for (int i = topLeftX + 1; i < bottomRightX; i++ )
    {
        for (int j = topLeftY + 1 ; j < bottomRightY; j++ )
        {
            Vec3b auxBGR = image.at<Vec3b>(j,i);
            uchar blue = auxBGR.val[0];
            uchar green = auxBGR.val[1];
            uchar red = auxBGR.val[2];

            int iblue = (int)blue;
            int igreen = (int)green;
            int ired = (int)red;

            intensityB = intensityB + iblue ;
            intensityG = intensityG + igreen ;
            intensityR = intensityR + ired ;

            iblue = 0;
            igreen = 0;
            ired = 0 ;
            //intensity = 0 ;
            el_count++;
        }
    }

    intensityB = intensityB / el_count ;
    intensityG = intensityG / el_count ;
    intensityR = intensityR / el_count ;

    intensity_aux[0] = intensityB ;
    intensity_aux[1] = intensityG ;
    intensity_aux[2] = intensityR ;

}

// function to get increment of a linear interpolation interval
// inicial point : (x0,y0)
// final point : (x1,y1)
// temperature_precision : interpolate with 0.1 celcius degrees precision. this was choosen for us.

double MainWindow::get_y_interp_increment( int x0, int x1, int y0, int y1, double temperature_precision )
{
    double increment = 0 ;
    increment = temperature_precision * ( y1 - y0 ) / (x1 - x0) ;
    return increment ;
}


// function to fill vector within a interval of the interpolation.
// v1 : initial value inside the interval
// v[] : vector to be filled
// initial_pos : initial position to be included. as it is 0.1 precision, this is 0, 20, 40 ... until 220.
// increment : increment on each interval, calculated using get_y_interp_increment

void MainWindow::fill_vec( int v1, double v[], int initial_pos, double increment )
{
    int size = SCALE_INTERVAL * SCALE_PREC ;

    v[ initial_pos ] = v1 ;
    for( int i = initial_pos + 1; i < ( initial_pos + size ); i++ ){
        v[ i ] = v[ i - 1 ] + increment ;
        //qDebug() << "\nel=" << QString::number( v[ i ] ) ;
    }
}

// function to fill all the values of a interpolated vector.
// temperature_vec : original temperature vector, without interpolation. this is used as x-axis (12 values)
// colorVec : original vector to be interpolated. this is used as y-axis. it can be color or the temperature itself (12 values)
// channel_dest : destination vector of the interpolated values. this is a bigger vector (220 values)

void MainWindow::fill_Color_Channel( int temperature_vec[], int colorVec[], double channel_dest[] )
{
    int pos = 0 ;
    double increment = 0 ;

    // It is used SCALE_EL - 1 to account for the number of intervals within a color channel.
    for( int i = 0; i < SCALE_EL - 1; i++ ) {
        increment = get_y_interp_increment( temperature_vec[ i ], temperature_vec[ i + 1 ], colorVec[ i ], colorVec[ i + 1], 0.1 ) ;

        fill_vec( colorVec[ i ], channel_dest, pos, increment ) ;

        pos = pos + ( SCALE_INTERVAL * SCALE_PREC ) ;
    }
    channel_dest[ pos ] = colorVec[ SCALE_EL - 1 ] ;
}

int find_closest_match( int BGR[], double R_interp[], double G_interp[], double B_interp[] )
{
    double aux_vec_B[ ( ( SCALE_EL - 1 ) * SCALE_PREC * SCALE_INTERVAL ) + 1 ] = {} ;
    double aux_vec_G[ ( ( SCALE_EL - 1 ) * SCALE_PREC * SCALE_INTERVAL ) + 1 ] = {} ;
    double aux_vec_R[ ( ( SCALE_EL - 1 ) * SCALE_PREC * SCALE_INTERVAL ) + 1 ] = {} ;
    double aux_vec_sum[ ( ( SCALE_EL - 1 ) * SCALE_PREC * SCALE_INTERVAL ) + 1 ] = {} ;

    for ( int i = 0 ; i < ( ( SCALE_EL - 1 ) * SCALE_PREC * SCALE_INTERVAL ) + 1 ; i++)
    {
        aux_vec_B[i] = abs( BGR[0] - B_interp[i] ) ;
        aux_vec_G[i] = abs( BGR[1] - G_interp[i] ) ;
        aux_vec_R[i] = abs( BGR[2] - R_interp[i] ) ;
        aux_vec_sum[i] = aux_vec_B[i] + aux_vec_G[i] + aux_vec_R[i] ;
        //qDebug() << "\nDistB(" << QString::number(i) << ") = " << QString::number( aux_vec_R[i] ) << "= " << QString::number( BGR[2] )
        //        << " - " << QString::number(R_interp[i]);
    }

    // Find minimum value within an array
    int index = 0;

    for( int i = 1; i < ( ( SCALE_EL - 1 ) * SCALE_PREC * SCALE_INTERVAL ) + 1; i++)
    {
        if ( aux_vec_sum[i] < aux_vec_sum[index] )
        index = i ;
    }

    return index ;
}

bool MainWindow::fileExists(QString path) {
    QFileInfo check_file(path);
    // check if file exists and if yes: Is it really a file and no directory?
    return check_file.exists() && check_file.isFile();
}

// return list of elements of a xml file

 QStringList MainWindow::ListElement(QDomElement root, QString tagname, QString attribute)
 {
     QDomNodeList items = root.elementsByTagName(tagname);
     QStringList entries ;

     for (int i = 0 ; i < items.count(); i++)
     {
         QDomNode itemnode = items.at(i) ;

         //convert to element
         if(itemnode.isElement())
         {
             QDomElement itemele = itemnode.toElement();
             entries << itemele.attribute(attribute) ;
         }
     }
     return entries ;
 }


 // update xml file temperature.xml
 void MainWindow::UpdateXML_temperature()
 {
     // xml saving
     QDomDocument document;
     QDomElement root = document.createElement("Fotos");
     document.appendChild(root);

     for (int i =0 ; i < PICTURE_NUMBER ; i++)
     {
         QDomElement photo = document.createElement("Foto");
         photo.setAttribute("Name", "Foto " + QString::number(i) );
         photo.setAttribute("ID", QString::number(i) );
         root.appendChild(photo);

         for (int j = 0; j < MARKERS_NUMBER ; j++)
         {
             QDomElement marker = document.createElement("Marcador");
             marker.setAttribute("Name", "Marcador " + QString::number(j+1) );
             marker.setAttribute("Temperatura", QString::number(temp_values[i * 6 + j]) );
             marker.setAttribute("x", QString::number(x_markers[i * 6 + j]) );
             marker.setAttribute("y", QString::number(y_markers[i * 6 + j]) );
             marker.setAttribute("central_marker_size", QString::number(size_bigger[i * 6 + j]) );
             marker.setAttribute("fingers_marker_size",QString::number(size_smaller[i * 6 + j]) );
             photo.appendChild(marker);
         }
     }

     //for (int i = 0; i < MARKERS_NUMBER * PICTURE_NUMBER; i++ ){
     //    qDebug() << "\nx_markers(" << i << ") = " << x_markers[i] ;
     //}


     // write to file
     QFile file(dir + "/temperatura.xml");

     if (!file.open(QIODevice::WriteOnly))
         {
             qDebug() << "Failed to open file for writing";
             //return -1;
         }
         else
         {
             qDebug() << "Writing..";
             QTextStream stream(&file);
             stream << document.toString();
             file.close();
             qDebug() << "Finished";
         }
 }

 void MainWindow::ReadXML_temperature()
 {
     // xml saving
     QDomDocument document;

     // Load to file
     QFile file(dir + "/temperatura.xml");

     if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
         {
             qDebug() << "Failed to open file for writing";
             //return -1;
         }
         else
         {
             if(!document.setContent((&file)))
             {
                  qDebug() << "Failed to load document";
                     //return -1;
             }
             file.close();
          }

     // get the root element
     QDomElement root = document.firstChildElement();

     // save the atributes in a list
     QStringList Foto_S = ListElement(root,"Foto", "Name");
     QStringList Marcador_S = ListElement(root,"Marcador", "Name");
     QStringList y_S = ListElement(root,"Marcador", "y");
     QStringList fingers_marker_size_S = ListElement(root,"Marcador", "fingers_marker_size");
     QStringList Temperatura_S = ListElement(root,"Marcador", "Temperatura");
     QStringList x_S = ListElement(root,"Marcador", "x");
     QStringList central_marker_size_S = ListElement(root,"Marcador", "central_marker_size");

     // fill arrays with value
     for(int i = 0; i < Foto_S.size(); i++) {
         for(int j = 0; j < 6; j++) {
             x_markers[i*6 + j] = x_S[i*6 + j].toInt() ;
             y_markers[i*6 + j] = y_S[i*6 + j].toInt() ;
             size_bigger[i*6 + j] = central_marker_size_S[i*6 + j].toInt() ;
             size_smaller[i*6 + j] = fingers_marker_size_S[i*6 + j].toInt() ;
             temp_values[i*6 + j] = Temperatura_S[i*6 + j].toDouble() ;
         }
     }

     // check if the array is filled properly
     for(int i = 0; i < Foto_S.size(); i++) {
              qDebug() << "\n" << Foto_S[i] ;
              for(int j = 0; j < 6; j++) {
                  qDebug() << "\n" << Marcador_S[i*6 + j] << "\t" << y_markers[i*6 + j] << "\t" << size_smaller[i*6 + j] << "\t" << temp_values[i*6 + j] << "\t" << x_markers[i*6 + j] << "\t" << size_bigger[i*6 + j];
              }
          }


     qDebug() << "\nFinished reading xml" ;
 }



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    scene = new QGraphicsScene(this);
    ui->graphicsView->setScene(scene);
    scene->setSceneRect( 0, 0, 320, 240 );

    // initialize variables
    xc = 10;
    yc = 30;
    wc = 8 ;
    hc = 8 ;

    xf = 30;
    yf = 30;
    wf = 4 ;
    hf = 4 ;

    ui ->spinBox -> blockSignals(true);
    ui ->spinBox ->setMinimum(1);
    ui ->spinBox ->setMaximum(20);
    ui ->spinBox ->setValue(8);
    ui ->spinBox -> blockSignals(false);

    ui ->spinBox_2 -> blockSignals(true);
    ui ->spinBox_2 ->setMinimum(1);
    ui ->spinBox_2 ->setMaximum(20);
    ui ->spinBox_2 ->setValue(4);
    ui ->spinBox_2 -> blockSignals(false);

    image_number = 0 ;

    // these buttons do not work unless user already chose a valid directory
    ui -> anterior -> setEnabled(false);
    ui -> proximo -> setEnabled(false);
    ui -> save -> setEnabled(false);
    ui -> temperature -> setEnabled(false);
    ui->editarFicha->setEnabled(false);

    ui ->spinBox->setEnabled(false);
    ui -> spinBox_2 ->setEnabled(false);

    ui -> actionSalvar->setEnabled(false);

    ui->photo_index->setText("Nenhuma foto valida selecionada");


}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_diretorio_clicked()
{
    // clear the graphics view and scene
    scene->clear();
    ui->graphicsView->viewport()->update();


    // get the directory of the photos
    dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                    QDir::homePath(),
                                                    QFileDialog::ShowDirsOnly
                                                    | QFileDialog::DontResolveSymlinks);

    // if not an empty directory,
    if(dir != "")
    {
        QDir image_folder(dir);
        image_folder.setNameFilters( QStringList() << "*.png" ) ;
        image_list = image_folder.entryList() ;

        // if the directory doesnt have exactly 12 pictures, error
        if(image_list.size() != PICTURE_NUMBER){
            QMessageBox::warning(this, "Numero de Imagens Incorreto", "Selecionar diretorio com 12 imagens .png") ;
            ui->graphicsView->setVisible(false);

            ui->label_11->setText("NA");
            ui->label_12->setText("NA");
            ui->label_13->setText("NA");
            ui->label_14->setText("NA");
            ui->label_15->setText("NA");
            ui->label_16->setText("NA");

            ui->photo_index->setStyleSheet("color: red");
            ui->photo_index->setText("Diretorio Invalido! Por favor, selecionar \ndiretorio com 12 fotos em formato .png");

            ui->anterior->setEnabled(false);
            ui->proximo->setEnabled(false);
            ui->temperature->setEnabled(false);
            ui->editarFicha->setEnabled(false);
            ui->save->setEnabled(false);

            ui -> actionSalvar->setEnabled(false);

            ui->spinBox->setEnabled(false);
            ui->spinBox_2->setEnabled(false);
        }
       // if the directory have 12 pictures, continue

        else {
            ui->graphicsView->setVisible(true);

            image_number = 0 ;

            QString first_image = dir+"/"+image_list.at(image_number) ;

            //----------
            // checking if xml file exists
            // xml initialization. if doesnt exists, create a new one
            if (!fileExists(dir+"/temperatura.xml"))
            {
                // initialize variables
                int increment = 20 ;
                int inicial_pos = 10 ;

                for (int i = 0 ; i < (MARKERS_NUMBER * PICTURE_NUMBER) ; i ++)
                {
                         if (i % 6 == 0) x_markers[i] = inicial_pos ;
                    else if (i % 6 == 1) x_markers[i] = inicial_pos + 1 * increment;
                    else if (i % 6 == 2) x_markers[i] = inicial_pos + 2 * increment;
                    else if (i % 6 == 3) x_markers[i] = inicial_pos + 3 * increment;
                    else if (i % 6 == 4) x_markers[i] = inicial_pos + 4 * increment;
                    else if (i % 6 == 5) x_markers[i] = inicial_pos + 5 * increment;

                    y_markers[i] = 30 ;
                    temp_values[i] = 0 ;
                    size_bigger[i] = wc ;
                    size_smaller[i] = wf ;
                }

                /*for (int i = 0; i < MARKERS_NUMBER * PICTURE_NUMBER; i++ ){
                    qDebug() << "\nx_markers(" << i << ") = " << x_markers[i] ;
                }*/

                // create xml file with these values
                UpdateXML_temperature();

            }
           // otherwise read the xml file

            ReadXML_temperature();

            // scene begin
            // add image
            scene->addPixmap(first_image);

            ui->photo_index->setStyleSheet("color: black");
            ui->photo_index->setText("Foto " + QString::number(image_number));

            // add markers
            // markers are displayed in increasing number sequence, from left to right

            // mark 1 - central marker
            mark1 = new squaremarker();
            mark1->setX(x_markers[image_number*6]) ;
            mark1->setY(y_markers[image_number*6]) ;
            mark1->w_m = size_bigger[image_number*6] ;
            mark1->h_m = size_bigger[image_number*6] ;
            scene->addItem(mark1);


            // fingers markers, starting from thumb to little finger
            mark2 = new squaremarker();
            mark2->setX(x_markers[image_number*6+1]) ;
            mark2->setY(y_markers[image_number*6+1]) ;
            mark2->w_m = size_smaller[image_number*6+1] ;
            mark2->h_m = size_smaller[image_number*6+1] ;
            scene->addItem(mark2);

            mark3 = new squaremarker();
            mark3->setX(x_markers[image_number*6+2]) ;
            mark3->setY(y_markers[image_number*6+2]) ;
            mark3->w_m = size_smaller[image_number*6+2] ;
            mark3->h_m = size_smaller[image_number*6+2] ;
            scene->addItem(mark3);

            mark4 = new squaremarker();
            mark4->setX(x_markers[image_number*6+3]) ;
            mark4->setY(y_markers[image_number*6+3]) ;
            mark4->w_m = size_smaller[image_number*6+3] ;
            mark4->h_m = size_smaller[image_number*6+3] ;
            scene->addItem(mark4);

            mark5 = new squaremarker();
            mark5->setX(x_markers[image_number*6+4]) ;
            mark5->setY(y_markers[image_number*6+4]) ;
            mark5->w_m = size_smaller[image_number*6+4] ;
            mark5->h_m = size_smaller[image_number*6+4] ;
            scene->addItem(mark5);

            mark6 = new squaremarker();
            mark6->setX(x_markers[image_number*6+5]) ;
            mark6->setY(y_markers[image_number*6+5]) ;
            mark6->w_m = size_smaller[image_number*6+5] ;
            mark6->h_m = size_smaller[image_number*6+5] ;
            scene->addItem(mark6);

            mark1->setToolTip("marcador central");
            mark2->setToolTip("marcador dedo polegar");
            mark3->setToolTip("marcador dedo indicador");
            mark4->setToolTip("marcador dedo medio");
            mark5->setToolTip("marcador dedo anelar");
            mark6->setToolTip("marcador dedo minimo");


            path_att = dir + "/" + image_list.at(image_number);
            path_att_cv  = path_att.toUtf8().constData() ;

            // update GUI values and enable buttons

            ui -> anterior -> setEnabled(true);
            ui -> proximo -> setEnabled(true);
            ui -> save -> setEnabled(true);
            ui -> temperature -> setEnabled(true);
            ui -> editarFicha -> setEnabled(true);

            ui -> actionSalvar -> setEnabled(true);

            ui -> spinBox -> setEnabled(true);
            ui -> spinBox -> setValue(size_bigger[6 * image_number]) ;
            ui -> spinBox_2 ->setEnabled(true);
            ui -> spinBox_2 -> setValue(size_smaller[6 * image_number]);

            ui->label_11->setText(QString::number(temp_values[image_number*6]) + " °C" );
            ui->label_12->setText(QString::number(temp_values[image_number*6 + 1]) + " °C" );
            ui->label_13->setText(QString::number(temp_values[image_number*6 + 2]) + " °C" );
            ui->label_14->setText(QString::number(temp_values[image_number*6 + 3]) + " °C" );
            ui->label_15->setText(QString::number(temp_values[image_number*6 + 4]) + " °C" );
            ui->label_16->setText(QString::number(temp_values[image_number*6 + 5]) + " °C" );
        }
    }

    else {
        QMessageBox::warning(this, "Diretorio Vazio", "Por favor, selecione um diretorio valido com 12 imagens .png") ;
        ui->graphicsView->setVisible(false);

        ui->label_11->setText("NA");
        ui->label_12->setText("NA");
        ui->label_13->setText("NA");
        ui->label_14->setText("NA");
        ui->label_15->setText("NA");
        ui->label_16->setText("NA");

        ui->photo_index->setStyleSheet("color: red");
        ui->photo_index->setText("Diretorio Vazio! Por favor, selecione um \ndiretorio com 12 fotos em formato .png");

        ui->anterior->setEnabled(false);
        ui->proximo->setEnabled(false);
        ui->temperature->setEnabled(false);
        ui->editarFicha->setEnabled(false);
        ui->save->setEnabled(false);

        ui -> actionSalvar->setEnabled(false);

        ui->spinBox->setEnabled(false);
        ui->spinBox_2->setEnabled(false);


    }
}


void MainWindow::on_temperature_clicked()
{
    coordinates[0]  = mark1 -> sceneBoundingRect().topLeft().x();
    coordinates[1]  = mark1 -> sceneBoundingRect().topLeft().y();
    coordinates[2]  = mark1 -> sceneBoundingRect().bottomRight().x();
    coordinates[3]  = mark1 -> sceneBoundingRect().bottomRight().y();

    coordinates[4]  = mark2 -> sceneBoundingRect().topLeft().x();
    coordinates[5]  = mark2 -> sceneBoundingRect().topLeft().y();
    coordinates[6]  = mark2 -> sceneBoundingRect().bottomRight().x();
    coordinates[7]  = mark2 -> sceneBoundingRect().bottomRight().y();

    coordinates[8]  = mark3 -> sceneBoundingRect().topLeft().x();
    coordinates[9]  = mark3 -> sceneBoundingRect().topLeft().y();
    coordinates[10]  = mark3 -> sceneBoundingRect().bottomRight().x();
    coordinates[11]  = mark3 -> sceneBoundingRect().bottomRight().y();

    coordinates[12]  = mark4 -> sceneBoundingRect().topLeft().x();
    coordinates[13]  = mark4 -> sceneBoundingRect().topLeft().y();
    coordinates[14]  = mark4 -> sceneBoundingRect().bottomRight().x();
    coordinates[15]  = mark4 -> sceneBoundingRect().bottomRight().y();

    coordinates[16]  = mark5 -> sceneBoundingRect().topLeft().x();
    coordinates[17]  = mark5 -> sceneBoundingRect().topLeft().y();
    coordinates[18]  = mark5 -> sceneBoundingRect().bottomRight().x();
    coordinates[19]  = mark5 -> sceneBoundingRect().bottomRight().y();

    coordinates[20]  = mark6 -> sceneBoundingRect().topLeft().x();
    coordinates[21]  = mark6 -> sceneBoundingRect().topLeft().y();
    coordinates[22]  = mark6 -> sceneBoundingRect().bottomRight().x();
    coordinates[23]  = mark6 -> sceneBoundingRect().bottomRight().y();


    // read image (using the path)
    image = imread( path_att_cv );

    // draw markers    
    rectangle(image,Point(coordinates[0],coordinates[1]), Point(coordinates[2],coordinates[3]), Scalar(0,0,0), 1);
    rectangle(image,Point(coordinates[4],coordinates[5]), Point(coordinates[6],coordinates[7]), Scalar(0,0,0), 1);
    rectangle(image,Point(coordinates[8],coordinates[9]), Point(coordinates[10],coordinates[11]), Scalar(0,0,0), 1);
    rectangle(image,Point(coordinates[12],coordinates[13]), Point(coordinates[14],coordinates[15]), Scalar(0,0,0), 1);
    rectangle(image,Point(coordinates[16],coordinates[17]), Point(coordinates[18],coordinates[19]), Scalar(0,0,0), 1);
    rectangle(image,Point(coordinates[20],coordinates[21]), Point(coordinates[22],coordinates[23]), Scalar(0,0,0), 1);

     // -----------------------
    // Gray scale
    Mat image_gray(image.size(),CV_8UC1);

    // IT MUST BE BGR TO GRAY, OTHERWISE THE GRAYSCALE ITS NOT AS IT SHOULD BE
    // comparing to MATLAB scale, this is the most similar one
    cvtColor(image,image_gray,CV_BGR2GRAY);


   //-----------------------------------------------------
   // section to get average : BGR colorspace

   int intensityMark1[3] = {};
   int intensityMark2[3] = {};
   int intensityMark3[3] = {};
   int intensityMark4[3] = {};
   int intensityMark5[3] = {};
   int intensityMark6[3] = {};

   intensity[0] = 0 ; // B
   intensity[1] = 0 ; // G
   intensity[2] = 0 ; // R

   getIntensityBGR(image, coordinates[0],coordinates[1],coordinates[2],coordinates[3], intensity);
   intensityMark1[0] = intensity[0];
   intensityMark1[1] = intensity[1];
   intensityMark1[2] = intensity[2];

   getIntensityBGR(image, coordinates[4],coordinates[5],coordinates[6],coordinates[7], intensity);
   intensityMark2[0] = intensity[0];
   intensityMark2[1] = intensity[1];
   intensityMark2[2] = intensity[2];

   getIntensityBGR(image, coordinates[8],coordinates[9],coordinates[10],coordinates[11], intensity);
   intensityMark3[0] = intensity[0];
   intensityMark3[1] = intensity[1];
   intensityMark3[2] = intensity[2];

   getIntensityBGR(image, coordinates[12],coordinates[13],coordinates[14],coordinates[15], intensity);
   intensityMark4[0] = intensity[0];
   intensityMark4[1] = intensity[1];
   intensityMark4[2] = intensity[2];

   getIntensityBGR(image, coordinates[16],coordinates[17],coordinates[18],coordinates[19], intensity);
   intensityMark5[0] = intensity[0];
   intensityMark5[1] = intensity[1];
   intensityMark5[2] = intensity[2];

   getIntensityBGR(image, coordinates[20],coordinates[21],coordinates[22],coordinates[23], intensity);
   intensityMark6[0] = intensity[0];
   intensityMark6[1] = intensity[1];
   intensityMark6[2] = intensity[2];


   // getting temperature out of BGR

   // interpolate each channel

   fill_Color_Channel( temperature, R_channel,     R_channel_interp ) ;
   fill_Color_Channel( temperature, G_channel,     G_channel_interp ) ;
   fill_Color_Channel( temperature, B_channel,     B_channel_interp ) ;
   fill_Color_Channel( temperature, temperature, temperature_interp ) ;

   // ----------------------

   // search for match

    int index_min1 = find_closest_match(intensityMark1, R_channel_interp, G_channel_interp, B_channel_interp ) ;
    int index_min2 = find_closest_match(intensityMark2, R_channel_interp, G_channel_interp, B_channel_interp ) ;
    int index_min3 = find_closest_match(intensityMark3, R_channel_interp, G_channel_interp, B_channel_interp ) ;
    int index_min4 = find_closest_match(intensityMark4, R_channel_interp, G_channel_interp, B_channel_interp ) ;
    int index_min5 = find_closest_match(intensityMark5, R_channel_interp, G_channel_interp, B_channel_interp ) ;
    int index_min6 = find_closest_match(intensityMark6, R_channel_interp, G_channel_interp, B_channel_interp ) ;

    ui->label_11->setText(QString::number(temperature_interp[index_min1] ) + " °C" ) ;
    ui->label_12->setText(QString::number(temperature_interp[index_min2] ) + " °C" ) ;
    ui->label_13->setText(QString::number(temperature_interp[index_min3] ) + " °C" ) ;
    ui->label_14->setText(QString::number(temperature_interp[index_min4] ) + " °C" ) ;
    ui->label_15->setText(QString::number(temperature_interp[index_min5] ) + " °C" ) ;
    ui->label_16->setText(QString::number(temperature_interp[index_min6] ) + " °C" ) ;

    // save temperatures os an array.
    // afterwards, this array will be used to export the data into a xml file.
    temp_values[image_number*6]   = temperature_interp[index_min1];
    temp_values[image_number*6+1] = temperature_interp[index_min2];
    temp_values[image_number*6+2] = temperature_interp[index_min3];
    temp_values[image_number*6+3] = temperature_interp[index_min4];
    temp_values[image_number*6+4] = temperature_interp[index_min5];
    temp_values[image_number*6+5] = temperature_interp[index_min6];

    // set mark x and y values to get markers position and its size
    x_markers[image_number*6]   = mark1 -> sceneBoundingRect().topLeft().x();
    x_markers[image_number*6+1] = mark2 -> sceneBoundingRect().topLeft().x();
    x_markers[image_number*6+2] = mark3 -> sceneBoundingRect().topLeft().x();
    x_markers[image_number*6+3] = mark4 -> sceneBoundingRect().topLeft().x();
    x_markers[image_number*6+4] = mark5 -> sceneBoundingRect().topLeft().x();
    x_markers[image_number*6+5] = mark6 -> sceneBoundingRect().topLeft().x();

    y_markers[image_number*6]   = mark1 -> sceneBoundingRect().topLeft().y();
    y_markers[image_number*6+1] = mark2 -> sceneBoundingRect().topLeft().y();
    y_markers[image_number*6+2] = mark3 -> sceneBoundingRect().topLeft().y();
    y_markers[image_number*6+3] = mark4 -> sceneBoundingRect().topLeft().y();
    y_markers[image_number*6+4] = mark5 -> sceneBoundingRect().topLeft().y();
    y_markers[image_number*6+5] = mark6 -> sceneBoundingRect().topLeft().y();

    size_bigger[image_number*6]   = wc;
    size_bigger[image_number*6+1] = wc;
    size_bigger[image_number*6+2] = wc;
    size_bigger[image_number*6+3] = wc;
    size_bigger[image_number*6+4] = wc;
    size_bigger[image_number*6+5] = wc;

    size_smaller[image_number*6]   = wf;
    size_smaller[image_number*6+1] = wf;
    size_smaller[image_number*6+2] = wf;
    size_smaller[image_number*6+3] = wf;
    size_smaller[image_number*6+4] = wf;
    size_smaller[image_number*6+5] = wf;


    // update xml file with new values
    UpdateXML_temperature();

   //-----------------------------------------------------
}

//spinbox used for bigger marker. updates its size
void MainWindow::on_spinBox_valueChanged(int arg1)
{

    // update markers position array
    x_markers[image_number*6]   = mark1 -> sceneBoundingRect().topLeft().x();
    x_markers[image_number*6+1] = mark2 -> sceneBoundingRect().topLeft().x();
    x_markers[image_number*6+2] = mark3 -> sceneBoundingRect().topLeft().x();
    x_markers[image_number*6+3] = mark4 -> sceneBoundingRect().topLeft().x();
    x_markers[image_number*6+4] = mark5 -> sceneBoundingRect().topLeft().x();
    x_markers[image_number*6+5] = mark6 -> sceneBoundingRect().topLeft().x();

    y_markers[image_number*6]   = mark1 -> sceneBoundingRect().topLeft().y();
    y_markers[image_number*6+1] = mark2 -> sceneBoundingRect().topLeft().y();
    y_markers[image_number*6+2] = mark3 -> sceneBoundingRect().topLeft().y();
    y_markers[image_number*6+3] = mark4 -> sceneBoundingRect().topLeft().y();
    y_markers[image_number*6+4] = mark5 -> sceneBoundingRect().topLeft().y();
    y_markers[image_number*6+5] = mark6 -> sceneBoundingRect().topLeft().y();

    wc = arg1 ;
    hc = arg1 ;

    // redraw marker . this is important, otherwise doesnt work.

    mark1->setX(x_markers[image_number*6]) ;
    mark1->setY(y_markers[image_number*6]) ;
    mark1->w_m = wc ;
    mark1->h_m = hc ;

   scene->removeItem(mark1);
   scene->addItem(mark1);

   mark1->setToolTip("marcador central");

}


//spinbox used for smaller markers. updates the size of the markers
void MainWindow::on_spinBox_2_valueChanged(int arg1)
{
    // update markers position array
    x_markers[image_number*6]   = mark1 -> sceneBoundingRect().topLeft().x();
    x_markers[image_number*6+1] = mark2 -> sceneBoundingRect().topLeft().x();
    x_markers[image_number*6+2] = mark3 -> sceneBoundingRect().topLeft().x();
    x_markers[image_number*6+3] = mark4 -> sceneBoundingRect().topLeft().x();
    x_markers[image_number*6+4] = mark5 -> sceneBoundingRect().topLeft().x();
    x_markers[image_number*6+5] = mark6 -> sceneBoundingRect().topLeft().x();

    y_markers[image_number*6]   = mark1 -> sceneBoundingRect().topLeft().y();
    y_markers[image_number*6+1] = mark2 -> sceneBoundingRect().topLeft().y();
    y_markers[image_number*6+2] = mark3 -> sceneBoundingRect().topLeft().y();
    y_markers[image_number*6+3] = mark4 -> sceneBoundingRect().topLeft().y();
    y_markers[image_number*6+4] = mark5 -> sceneBoundingRect().topLeft().y();
    y_markers[image_number*6+5] = mark6 -> sceneBoundingRect().topLeft().y();

    wf = arg1 ;
    hf = arg1 ;

    // redraw marker . this is important, otherwise doesnt work.

    mark2->setX(x_markers[image_number*6+1]) ;
    mark2->setY(y_markers[image_number*6+1]) ;
    mark2->w_m = wf ;
    mark2->h_m = hf ;
    scene->removeItem(mark2);
    scene->addItem(mark2);

    mark3->setX(x_markers[image_number*6+2]) ;
    mark3->setY(y_markers[image_number*6+2]) ;
    mark3->w_m = wf ;
    mark3->h_m = hf ;
    scene->removeItem(mark3);
    scene->addItem(mark3);

    mark4->setX(x_markers[image_number*6+3]) ;
    mark4->setY(y_markers[image_number*6+3]) ;
    mark4->w_m = wf ;
    mark4->h_m = hf ;
    scene->removeItem(mark4);
    scene->addItem(mark4);

    mark5->setX(x_markers[image_number*6+4]) ;
    mark5->setY(y_markers[image_number*6+4]) ;
    mark5->w_m = wf ;
    mark5->h_m = hf ;
    scene->removeItem(mark5);
    scene->addItem(mark5);

    mark6->setX(x_markers[image_number*6+5]) ;
    mark6->setY(y_markers[image_number*6+5]) ;
    mark6->w_m = wf ;
    mark6->h_m = hf ;
    scene->removeItem(mark6);
    scene->addItem(mark6);

    mark2->setToolTip("marcador dedo polegar");
    mark3->setToolTip("marcador dedo indicador");
    mark4->setToolTip("marcador dedo medio");
    mark5->setToolTip("marcador dedo anelar");
    mark6->setToolTip("marcador dedo minimo");
}

void MainWindow::on_proximo_clicked()
{
    // set mark x and y values to get markers position and its size
    x_markers[image_number*6]   = mark1 -> sceneBoundingRect().topLeft().x();
    x_markers[image_number*6+1] = mark2 -> sceneBoundingRect().topLeft().x();
    x_markers[image_number*6+2] = mark3 -> sceneBoundingRect().topLeft().x();
    x_markers[image_number*6+3] = mark4 -> sceneBoundingRect().topLeft().x();
    x_markers[image_number*6+4] = mark5 -> sceneBoundingRect().topLeft().x();
    x_markers[image_number*6+5] = mark6 -> sceneBoundingRect().topLeft().x();

    y_markers[image_number*6]   = mark1 -> sceneBoundingRect().topLeft().y();
    y_markers[image_number*6+1] = mark2 -> sceneBoundingRect().topLeft().y();
    y_markers[image_number*6+2] = mark3 -> sceneBoundingRect().topLeft().y();
    y_markers[image_number*6+3] = mark4 -> sceneBoundingRect().topLeft().y();
    y_markers[image_number*6+4] = mark5 -> sceneBoundingRect().topLeft().y();
    y_markers[image_number*6+5] = mark6 -> sceneBoundingRect().topLeft().y();

    size_bigger[image_number*6]   = wc;
    size_bigger[image_number*6+1] = wc;
    size_bigger[image_number*6+2] = wc;
    size_bigger[image_number*6+3] = wc;
    size_bigger[image_number*6+4] = wc;
    size_bigger[image_number*6+5] = wc;

    size_smaller[image_number*6]   = wf;
    size_smaller[image_number*6+1] = wf;
    size_smaller[image_number*6+2] = wf;
    size_smaller[image_number*6+3] = wf;
    size_smaller[image_number*6+4] = wf;
    size_smaller[image_number*6+5] = wf;

    on_temperature_clicked();

    // create a new scene
    scene->removeItem(mark1);
    scene->removeItem(mark2);
    scene->removeItem(mark3);
    scene->removeItem(mark4);
    scene->removeItem(mark5);
    scene->removeItem(mark6);

    // gets the original path
    QDir image_folder(dir);
    image_folder.setNameFilters( QStringList() << "*.png" ) ;

    // update position of the list
    image_number++ ;

    if( image_number > image_list.size() - 1 )
    {
        image_number = 0 ;
    }
    // updates path
    path_att = dir + "/" + image_list.at(image_number);
    path_att_cv  = path_att.toUtf8().constData() ;

    // recreate pixmap
    scene -> addPixmap( path_att );

    ui->photo_index->setText("Foto " + QString::number(image_number));

    // redraw markers

    mark1 = new squaremarker();
    mark1->setX(x_markers[image_number*6]) ;
    mark1->setY(y_markers[image_number*6]) ;
    mark1->w_m = size_bigger[image_number*6] ;
    mark1->h_m = size_bigger[image_number*6] ;
    scene->addItem(mark1);

    mark2 = new squaremarker();
    mark2->setX(x_markers[image_number*6+1]) ;
    mark2->setY(y_markers[image_number*6+1]) ;
    mark2->w_m = size_smaller[image_number*6 + 1] ;
    mark2->h_m = size_smaller[image_number*6 + 1] ;
    scene->addItem(mark2);

    mark3 = new squaremarker();
    mark3->setX(x_markers[image_number*6+2]) ;
    mark3->setY(y_markers[image_number*6+2]) ;
    mark3->w_m = size_smaller[image_number*6 + 2] ;
    mark3->h_m = size_smaller[image_number*6 + 2] ;
    scene->addItem(mark3);

    mark4 = new squaremarker();
    mark4->setX(x_markers[image_number*6+3]) ;
    mark4->setY(y_markers[image_number*6+3]) ;
    mark4->w_m = size_smaller[image_number*6 + 3] ;
    mark4->h_m = size_smaller[image_number*6 + 3] ;
    scene->addItem(mark4);

    mark5 = new squaremarker();
    mark5->setX(x_markers[image_number*6+4]) ;
    mark5->setY(y_markers[image_number*6+4]) ;
    mark5->w_m = size_smaller[image_number*6 + 4] ;
    mark5->h_m = size_smaller[image_number*6 + 4] ;
    scene->addItem(mark5);

    mark6 = new squaremarker();
    mark6->setX(x_markers[image_number*6+5]) ;
    mark6->setY(y_markers[image_number*6+5]) ;
    mark6->w_m = size_smaller[image_number*6 + 5] ;
    mark6->h_m = size_smaller[image_number*6 + 5] ;
    scene->addItem(mark6);

    mark1->setToolTip("marcador central");
    mark2->setToolTip("marcador dedo polegar");
    mark3->setToolTip("marcador dedo indicador");
    mark4->setToolTip("marcador dedo medio");
    mark5->setToolTip("marcador dedo anelar");
    mark6->setToolTip("marcador dedo minimo");

    // updates GUI values
    ui->spinBox->setValue(size_bigger[image_number*6]);
    ui->spinBox_2->setValue(size_smaller[image_number*6]);



    ui->label_11->setText(QString::number(temp_values[image_number*6]) + " °C" );
    ui->label_12->setText(QString::number(temp_values[image_number*6 + 1]) + " °C" );
    ui->label_13->setText(QString::number(temp_values[image_number*6 + 2]) + " °C" );
    ui->label_14->setText(QString::number(temp_values[image_number*6 + 3]) + " °C" );
    ui->label_15->setText(QString::number(temp_values[image_number*6 + 4]) + " °C" );
    ui->label_16->setText(QString::number(temp_values[image_number*6 + 5]) + " °C" );
}

void MainWindow::on_anterior_clicked()
{
    // set mark x and y values to get markers position and its size
    x_markers[image_number*6]   = mark1 -> sceneBoundingRect().topLeft().x();
    x_markers[image_number*6+1] = mark2 -> sceneBoundingRect().topLeft().x();
    x_markers[image_number*6+2] = mark3 -> sceneBoundingRect().topLeft().x();
    x_markers[image_number*6+3] = mark4 -> sceneBoundingRect().topLeft().x();
    x_markers[image_number*6+4] = mark5 -> sceneBoundingRect().topLeft().x();
    x_markers[image_number*6+5] = mark6 -> sceneBoundingRect().topLeft().x();

    y_markers[image_number*6]   = mark1 -> sceneBoundingRect().topLeft().y();
    y_markers[image_number*6+1] = mark2 -> sceneBoundingRect().topLeft().y();
    y_markers[image_number*6+2] = mark3 -> sceneBoundingRect().topLeft().y();
    y_markers[image_number*6+3] = mark4 -> sceneBoundingRect().topLeft().y();
    y_markers[image_number*6+4] = mark5 -> sceneBoundingRect().topLeft().y();
    y_markers[image_number*6+5] = mark6 -> sceneBoundingRect().topLeft().y();

    size_bigger[image_number*6]   = wc;
    size_bigger[image_number*6+1] = wc;
    size_bigger[image_number*6+2] = wc;
    size_bigger[image_number*6+3] = wc;
    size_bigger[image_number*6+4] = wc;
    size_bigger[image_number*6+5] = wc;

    size_smaller[image_number*6]   = wf;
    size_smaller[image_number*6+1] = wf;
    size_smaller[image_number*6+2] = wf;
    size_smaller[image_number*6+3] = wf;
    size_smaller[image_number*6+4] = wf;
    size_smaller[image_number*6+5] = wf;

    // update xml
    on_temperature_clicked();

    // goes to next pic

    scene->removeItem(mark1);
    scene->removeItem(mark2);
    scene->removeItem(mark3);
    scene->removeItem(mark4);
    scene->removeItem(mark5);
    scene->removeItem(mark6);

    // gets the original path
    QDir image_folder(dir);
    image_folder.setNameFilters( QStringList() << "*.png" ) ;

    // decrement position of image list

    image_number-- ;

    if( image_number < 0 )
    {
        image_number = image_list.size() - 1 ;
    }

    // updates path
    path_att = dir + "/" + image_list.at(image_number);
    path_att_cv  = path_att.toUtf8().constData() ;

    // recreate pixmap
    scene -> addPixmap( path_att );

    ui->photo_index->setText("Foto " + QString::number(image_number));

    QPen blackpen(Qt::black);
    blackpen.setWidth(0);

    mark1 = new squaremarker();
    mark1->setX(x_markers[image_number*6]) ;
    mark1->setY(y_markers[image_number*6]) ;
    mark1->w_m = size_bigger[image_number*6] ;
    mark1->h_m = size_bigger[image_number*6] ;
    scene->addItem(mark1);

    mark2 = new squaremarker();
    mark2->setX(x_markers[image_number*6+1]) ;
    mark2->setY(y_markers[image_number*6+1]) ;
    mark2->w_m = size_smaller[image_number*6 + 1] ;
    mark2->h_m = size_smaller[image_number*6 + 1] ;
    scene->addItem(mark2);

    mark3 = new squaremarker();
    mark3->setX(x_markers[image_number*6+2]) ;
    mark3->setY(y_markers[image_number*6+2]) ;
    mark3->w_m = size_smaller[image_number*6 + 2] ;
    mark3->h_m = size_smaller[image_number*6 + 2] ;
    scene->addItem(mark3);

    mark4 = new squaremarker();
    mark4->setX(x_markers[image_number*6+3]) ;
    mark4->setY(y_markers[image_number*6+3]) ;
    mark4->w_m = size_smaller[image_number*6 + 3] ;
    mark4->h_m = size_smaller[image_number*6 + 3] ;
    scene->addItem(mark4);

    mark5 = new squaremarker();
    mark5->setX(x_markers[image_number*6+4]) ;
    mark5->setY(y_markers[image_number*6+4]) ;
    mark5->w_m = size_smaller[image_number*6 + 4] ;
    mark5->h_m = size_smaller[image_number*6 + 4] ;
    scene->addItem(mark5);

    mark6 = new squaremarker();
    mark6->setX(x_markers[image_number*6+5]) ;
    mark6->setY(y_markers[image_number*6+5]) ;
    mark6->w_m = size_smaller[image_number*6 + 5] ;
    mark6->h_m = size_smaller[image_number*6 + 5] ;
    scene->addItem(mark6);

    mark1->setToolTip("marcador central");
    mark2->setToolTip("marcador dedo polegar");
    mark3->setToolTip("marcador dedo indicador");
    mark4->setToolTip("marcador dedo medio");
    mark5->setToolTip("marcador dedo anelar");
    mark6->setToolTip("marcador dedo minimo");

    //updates GUI values
    ui->spinBox->setValue(size_bigger[image_number*6]);
    ui->spinBox_2->setValue(size_smaller[image_number*6]);

    ui->label_11->setText(QString::number(temp_values[image_number*6]) + " °C" );
    ui->label_12->setText(QString::number(temp_values[image_number*6 + 1]) + " °C" );
    ui->label_13->setText(QString::number(temp_values[image_number*6 + 2]) + " °C" );
    ui->label_14->setText(QString::number(temp_values[image_number*6 + 3]) + " °C" );
    ui->label_15->setText(QString::number(temp_values[image_number*6 + 4]) + " °C" );
    ui->label_16->setText(QString::number(temp_values[image_number*6 + 5]) + " °C" );
}


void MainWindow::on_save_clicked()
{
    QDir dirSave(dir + "/editadas");
    if (!dirSave.exists()) {
        dirSave.mkpath(".");
    }

    QString file_name_saved ;
    QString file_name_oiginal = "edited" ;

    QStringList image_list_save;


    for (int i = 0 ; i < image_list.size() ; i++)
    {
        // remove all previous markers
        scene->removeItem(mark1);
        scene->removeItem(mark2);
        scene->removeItem(mark3);
        scene->removeItem(mark4);
        scene->removeItem(mark5);
        scene->removeItem(mark6);

        // updates path
        path_att = dir + "/" + image_list.at(i);
        path_att_cv  = path_att.toUtf8().constData() ;

        // recreate pixmap
        scene -> addPixmap( path_att );

        // redraw markers

        mark1 = new squaremarker();
        mark1->setX(x_markers[i*6]) ;
        mark1->setY(y_markers[i*6]) ;
        mark1->w_m = size_bigger[i*6] ;
        mark1->h_m = size_bigger[i*6] ;
        scene->addItem(mark1);

        mark2 = new squaremarker();
        mark2->setX(x_markers[i*6+1]) ;
        mark2->setY(y_markers[i*6+1]) ;
        mark2->w_m = size_smaller[i*6 + 1] ;
        mark2->h_m = size_smaller[i*6 + 1] ;
        scene->addItem(mark2);

        mark3 = new squaremarker();
        mark3->setX(x_markers[i*6+2]) ;
        mark3->setY(y_markers[i*6+2]) ;
        mark3->w_m = size_smaller[i*6 + 2] ;
        mark3->h_m = size_smaller[i*6 + 2] ;
        scene->addItem(mark3);

        mark4 = new squaremarker();
        mark4->setX(x_markers[i*6+3]) ;
        mark4->setY(y_markers[i*6+3]) ;
        mark4->w_m = size_smaller[i*6 + 3] ;
        mark4->h_m = size_smaller[i*6 + 3] ;
        scene->addItem(mark4);

        mark5 = new squaremarker();
        mark5->setX(x_markers[i*6+4]) ;
        mark5->setY(y_markers[i*6+4]) ;
        mark5->w_m = size_smaller[i*6 + 4] ;
        mark5->h_m = size_smaller[i*6 + 4] ;
        scene->addItem(mark5);

        mark6 = new squaremarker();
        mark6->setX(x_markers[i*6+5]) ;
        mark6->setY(y_markers[i*6+5]) ;
        mark6->w_m = size_smaller[i*6 + 5] ;
        mark6->h_m = size_smaller[i*6 + 5] ;
        scene->addItem(mark6);



        // make a screen shot of the graphics view area
        QPixmap pixMap = this->ui->graphicsView->grab();

        //updates file name
        file_name_saved = file_name_oiginal + QString::number(i);
        // save in a new directory /editadas
        pixMap.save(dir + "/editadas/"  + file_name_saved + ".png");
       // qDebug() << "\nLista editada 1 (" << i << "):" << image_list_save.at(i) ;
    }

    dirSave.setNameFilters( QStringList() << "*.png" ) ;
    image_list_save = dirSave.entryList() ;
    /*for (int i = 0 ; i < image_list_save.size() ; i++)
    {
        qDebug() << "\nLista editada 2 (" << i << "):" << image_list_save.at(i) ;
    }*/


    //recreate the view like it was when save was clicked
    // remove all previous markers
    scene->removeItem(mark1);
    scene->removeItem(mark2);
    scene->removeItem(mark3);
    scene->removeItem(mark4);
    scene->removeItem(mark5);
    scene->removeItem(mark6);

    // updates path
    path_att = dir + "/" + image_list.at(image_number);
    path_att_cv  = path_att.toUtf8().constData() ;

    // recreate pixmap
    scene -> addPixmap( path_att );

    // redraw markers

    mark1 = new squaremarker();
    mark1->setX(x_markers[image_number*6]) ;
    mark1->setY(y_markers[image_number*6]) ;
    mark1->w_m = size_bigger[image_number*6] ;
    mark1->h_m = size_bigger[image_number*6] ;
    scene->addItem(mark1);

    mark2 = new squaremarker();
    mark2->setX(x_markers[image_number*6+1]) ;
    mark2->setY(y_markers[image_number*6+1]) ;
    mark2->w_m = size_smaller[image_number*6 + 1] ;
    mark2->h_m = size_smaller[image_number*6 + 1] ;
    scene->addItem(mark2);

    mark3 = new squaremarker();
    mark3->setX(x_markers[image_number*6+2]) ;
    mark3->setY(y_markers[image_number*6+2]) ;
    mark3->w_m = size_smaller[image_number*6 + 2] ;
    mark3->h_m = size_smaller[image_number*6 + 2] ;
    scene->addItem(mark3);

    mark4 = new squaremarker();
    mark4->setX(x_markers[image_number*6+3]) ;
    mark4->setY(y_markers[image_number*6+3]) ;
    mark4->w_m = size_smaller[image_number*6 + 3] ;
    mark4->h_m = size_smaller[image_number*6 + 3] ;
    scene->addItem(mark4);

    mark5 = new squaremarker();
    mark5->setX(x_markers[image_number*6+4]) ;
    mark5->setY(y_markers[image_number*6+4]) ;
    mark5->w_m = size_smaller[image_number*6 + 4] ;
    mark5->h_m = size_smaller[image_number*6 + 4] ;
    scene->addItem(mark5);

    mark6 = new squaremarker();
    mark6->setX(x_markers[image_number*6+5]) ;
    mark6->setY(y_markers[image_number*6+5]) ;
    mark6->w_m = size_smaller[image_number*6 + 5] ;
    mark6->h_m = size_smaller[image_number*6 + 5] ;
    scene->addItem(mark6);

    mark1->setToolTip("marcador central");
    mark2->setToolTip("marcador dedo polegar");
    mark3->setToolTip("marcador dedo indicador");
    mark4->setToolTip("marcador dedo medio");
    mark5->setToolTip("marcador dedo anelar");
    mark6->setToolTip("marcador dedo minimo");


}

void MainWindow::on_editarFicha_clicked()
{
    // create new Window
    patientFile myPatient;
    myPatient.set_dir_patient(dir);

    //qDebug() << myPatient.get_dir_patient() << "\n";
    myPatient.setModal(true);
    myPatient.exec();
}

// choose markers location method
void MainWindow::on_processingMethod_currentIndexChanged(const QString &arg1)
{
    if(arg1 == "manual") qDebug() << "\n Metodo: " << arg1 ;
    if(arg1 == "metodo1") qDebug() << "\n Metodo: " << arg1 ;
    if(arg1 == "metodo2") qDebug() << "\n Metodo: " << arg1 ;
}


// IMAGE PROCESSING HERE
void MainWindow::on_pushButton_clicked()
{

    // --------- begin of reading image from path ---------

    QDir image_folder(dir);
    image_folder.setNameFilters( QStringList() << "*.png" ) ;
    image_list = image_folder.entryList() ;

    QString path_2 = dir + "/" + image_list.at(image_number);

    string path_cv  = path_2.toUtf8().constData() ;

    image = imread(path_cv);
    image2 = imread( "C:/Users/Isabela/Documents/fotos_editadasRGB/IR_04359.png" );

    // --------- end of reading image from path ---------



    // ---------- begin of crop image ----------

    // Setup a rectangle to define your region of interest
    cv::Rect myROI(2, 30, 300, 200);

    // Crop the full image to that image contained by the rectangle myROI
    // Note that this doesn't copy the data
    cv::Mat croppedRef(image, myROI);


    // Copy the data into new matrix
    croppedRef.copyTo(cropped);
    namedWindow( "cropped", WINDOW_AUTOSIZE );
    imshow( "cropped", cropped );

    // ---------- end of crop image ----------
/*
    // get BGR mean value inside a rectangle

    rectangle(cropped,Point(3,3), Point(10,190), Scalar(0,0,0), 1);


    int intensitymean[3] = {};
    int intensity_test[3];
    intensity_test[0] = 0 ; // B
    intensity_test[1] = 0 ; // G
    intensity_test[2] = 0 ; // R

    getIntensityBGR(cropped, 3,3,10,190, intensity_test);
    intensitymean[0] = intensity_test[0];
    intensitymean[1] = intensity_test[1];
    intensitymean[2] = intensity_test[2];

    qDebug() << " b: " << QString::number(intensitymean[0]) << " g: " << QString::number(intensitymean[1]) << " r: " << QString::number(intensitymean[2]) ;


    removeBackground(cropped, intensitymean);

    Mat cropped_gray(cropped.size(),CV_8UC1);

    cvtColor( cropped, cropped_gray, CV_BGR2GRAY );

    threshold( cropped_gray, cropped, 127, 255,0 );



  // bitwise_not(cropped,cropped);
    imshow( "cropped", cropped );

    //threshold( src_gray, dst, threshold_value, max_BINARY_value,threshold_type );

    // contours
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    RNG rng(12345);
    findContours( cropped, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );


    // convex hull

    // Find the convex hull object for each contour
       vector<vector<Point> >hull( contours.size() );
       for( int i = 0; i < contours.size(); i++ )
       {
           convexHull( Mat(contours[i]), hull[i], false );
       }




    Mat drawing = Mat::zeros( cropped.size(), CV_8UC3 );
       for( int i = 0; i< contours.size(); i++ )
          {
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
            drawContours( drawing, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
          }

       /// Show in a window
       namedWindow( "Hull demo", WINDOW_AUTOSIZE );
       imshow( "Hull demo", drawing );

*/

// end of segmentation using mean threashold.




    // another try using canny

     namedWindow( "canny_image", WINDOW_AUTOSIZE );
     Mat cropped_binary(cropped.size(), CV_8UC1);

     // detect if image is warm or not
     int get_colors[3] ;
     get_colors[0] = 0 ; // B
     get_colors[1] = 0 ; // G
     get_colors[2] = 0 ; // R
     getIntensityBGR(cropped, 0 , 0 , cropped.size().width , cropped.size().height , get_colors);
     if (get_colors[2]  > 35 )
     {
         warm_hand = true;
         //lowThreshold = 45;
         qDebug() << "imagem quente , R: " << QString::number(get_colors[2]);
     }
     else
     {
         warm_hand = false ;
         qDebug() << "imagem fria , R : " << QString::number(get_colors[2]) ;
     }

    // detected_edges = cropped ;

     if (warm_hand)
     {
         lowThreshold = 45 ;
     }
     else
     {
         lowThreshold = 11 ;
     }

     // begin of image processing

     // blur and canny
     GaussianBlur( cropped, cropped_blur, Size(3,3) , 1, 1 );
     //blur( cropped, cropped_blur, Size(3,3) );
     Canny( cropped_blur, cropped_new, lowThreshold, lowThreshold*razao, kernel_size );
     imshow( "canny_image", cropped_new );


     Mat element_dilation = getStructuringElement( MORPH_ELLIPSE,
                          Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                          Point( dilation_size, dilation_size ) );

     // dilation
     dilate( cropped_new, dilated, element_dilation );
     imshow( "canny_image", dilated );

     Mat element_erosion = getStructuringElement( MORPH_ELLIPSE,
                          Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                          Point( erosion_size, erosion_size ) );

     // Apply the erosion operation
     erode( dilated, erosion_dst, element_erosion );
     imshow( "canny_image", erosion_dst );

     // contours
     vector<vector<Point> > contours;
     vector<Vec4i> hierarchy;
     RNG rng(12345);
     findContours( erosion_dst, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

     // begin of test to get contours

     vector<vector<Point>> hull( contours.size() );
     vector<vector<Point>> hullsI( contours.size() );
     vector<vector<Point>> contours_poly( contours.size() );
     vector<vector<Vec4i>> defects( contours.size()) ;
     Point2f rect_points[4];
     vector<RotatedRect> minRect( contours.size() );
     vector<Rect> boundRect( contours.size() );

     Mat frame2 = Mat::zeros( erosion_dst.size(), CV_8UC3 );
     int largest_contour_index;
     largest_contour_index = findBiggestContour(contours);

     for( int i = 0; i < contours.size(); i++ )
          {
              convexHull( Mat(contours[i]), hull[i], false );
              convexHull( Mat(contours[i]), hullsI[i], true );
              //convexityDefects(Mat(contours[i]),hullsI[i], defects[i]);

                 if(largest_contour_index == i)
                    {
                      // minRect[i] = minAreaRect( Mat(contours[i]) );

                      //draw contour of biggest object
                       drawContours( frame2, contours,largest_contour_index, CV_RGB(255,255,255), 1, 8, vector<Vec4i>(),0, Point() );
                     //draw hull of biggesr object
                       drawContours( frame2, hull, largest_contour_index, CV_RGB(255,255,0), 1, 8, vector<Vec4i>(), 0, Point() );


                       approxPolyDP( Mat(hull[i]), contours_poly[i], 2, true );
                       boundRect[i] = boundingRect( Mat(contours_poly[i]) );
                       rectangle( frame2, boundRect[i].tl(), boundRect[i].br(), CV_RGB(0,255,0), 1, 8, 0 );




                    }
          }


     int PontoX = 0 ;
     int PontoY = 0 ;
     int sizeArray = hullsI[largest_contour_index].size() ;
     int markers_aux_X[sizeArray] = {};
     int markers_aux_Y[sizeArray] = {};


     int sizePoly = contours_poly[largest_contour_index].size() ;
     int poly_X[sizePoly] = {};
     int poly_Y[sizePoly] = {};
       //Point
     for (int a = 0 ; a < sizeArray ; a ++ )
     {
         PontoX = hullsI[largest_contour_index][a].x ;
         PontoY = hullsI[largest_contour_index][a].y ;
         markers_aux_X[a] = PontoX ;
         markers_aux_Y[a] = PontoY ;

        // qDebug() << "ponto[" << a << "]: " << markers_aux_X[a] << "," << markers_aux_Y[a];

         circle( frame2, hullsI[largest_contour_index][a], 4, Scalar(0,0,255), 1, 8, 0 );

     }


     //sort(markers_aux_Y.begin(), markers_aux_Y.end() )
     //ascending_sort(markers_aux_Y, hullsI[largest_contour_index].size());
     int index_first = 0 ;
     index_first = findSmallestValue (markers_aux_Y , sizeArray) ;
     qDebug() << "minimum " << index_first ;
     int update_index_x = 0 ;
     update_index_x = getSmallestX (markers_aux_X[index_first-1] , markers_aux_X[index_first] ) ;

     // index of smallest x with y = 0 point
     index_first = index_first - update_index_x ;
     qDebug() << ", new minimum : " << index_first ;

     int shift = 0 ;
     //shift = hullsI[largest_contour_index].size() - index_first ;
     shift = index_first ;
     qDebug() << "\nshift : " << shift ;

     int x_aux_2[sizeArray] ;
     int y_aux_2[sizeArray] ;
     for (int i = 0 ; i < sizeArray ; i ++)
     {
         x_aux_2[i] = 0 ;
         y_aux_2[i] = 0 ;
     }

     for (int i = 0 ; i < sizeArray ; i ++)
     {
         x_aux_2[i] = markers_aux_X[i] ;
         y_aux_2[i] = markers_aux_Y[i];
     }

    qDebug() << "\n\n\n\n";
     for ( int i = 0 ; i < shift ; i ++ )
     {
         shift_r (x_aux_2, sizeArray);
         shift_r (y_aux_2, sizeArray);
     }



     // gets poly info
     //fill polly aux array
       for (int a = 0 ; a < sizePoly ; a ++ )
       {

           poly_X[a] = contours_poly[largest_contour_index][a].x ;
           poly_Y[a] = contours_poly[largest_contour_index][a].y ;

           qDebug() << "ponto polly[" << a << "]: " << poly_X[a] << "," << poly_Y[a];

       }


       int index_first_poly = 0 ;
       index_first_poly = findSmallestValue (poly_Y , sizePoly) ;
       qDebug() << "minimum " << index_first_poly ;
       int update_index_x_poly = 0 ;
       update_index_x_poly = getSmallestX (poly_X[index_first_poly-1] , poly_X[index_first_poly] ) ;

       // index of smallest x with y = 0 point
       index_first_poly = index_first_poly - update_index_x_poly ;
       qDebug() << ", new minimum : " << update_index_x_poly ;

       int shift_poly = 0 ;
       //shift = hullsI[largest_contour_index].size() - index_first ;
       shift_poly = index_first_poly ;
       qDebug() << "\nshift : " << shift_poly ;

       int x_aux_poly[sizePoly] ;
       int y_aux_poly[sizePoly] ;
       for (int i = 0 ; i < sizePoly ; i ++)
       {
           x_aux_poly[i] = 0 ;
           y_aux_poly[i] = 0 ;
       }

       for (int i = 0 ; i < sizePoly ; i ++)
       {
           x_aux_poly[i] = poly_X[i] ;
           y_aux_poly[i] = poly_Y[i];
       }


      qDebug() << "\n\n\n\n";
       for ( int i = 0 ; i < shift_poly+1 ; i ++ )
       {
           shift_r (x_aux_poly, sizePoly);
           shift_r (y_aux_poly, sizePoly);
       }

       for (int a = 0 ; a < sizePoly ; a ++ )
       {

           qDebug() << "ponto shift polly[" << a << "]: " << x_aux_poly[a] << "," << y_aux_poly[a];

       }

/*
     for (int a = 0 ; a < sizeArray ; a ++ )
     {
         qDebug() << "ponto[" << a << "]: " << x_aux_2[a] << "," << y_aux_2[a];
     }
     */


     // calculate distance between neighboors

     double distanceArray[sizeArray];
     for (int i = 0 ; i < sizeArray ; i++)
     {
        distanceArray[i] = distance( x_aux_2[i] , x_aux_2[i+1] , y_aux_2[i], y_aux_2[i+1] ) ;
        qDebug() << "distancia entre " << i << " e " << i+1 << ":" << distanceArray[i];
     }


     // check if it is right hand or left hand
     // checa se o segundo ou o penultimo tem o y maior. se for o segundo, mao direita. se for penultimo, mao esquerda
     int hand = 0 ;
     if ( poly_Y[1] > poly_Y[sizePoly-2] )
     {
         hand = 1 ; // right hand
         qDebug() << "\n\n\ny1 : " << poly_Y[1] << "y2 : " << poly_Y[sizePoly-2] << "mao esquerda\n\n" ;
     }
     else
     {
        hand = 0 ;  // left hand
        qDebug() << "\n\n\ny1 : " << poly_Y[1] << "y2 : " << poly_Y[sizePoly-2] << "mao direita\n\n" ;
     }



     // initialize new point array with zero
     int new_point[sizeArray] ;
     for (int i = 0 ; i < sizeArray ; i++)
     {
        new_point[i] = 0 ;
     }

     //find new points until 5th point

     // still missing :
     // if it is right hand (hand = 1 ) , loop upwards towars x_aux_2 e y_aux_2. otherwise, loop downwards
     int total_points = 0 ;
     for (int i = 0 ; i < sizeArray ; i++)
     {

        if ( distanceArray[i] > 21.9 )
        {
            if ( total_points < 5 )
            {
                total_points++ ;
                new_point[i+1] = 1 ;
                circle( frame2, Point(x_aux_2[i+1],y_aux_2[i+1]), 4, Scalar(255,0,0), 1, 8, 0 );
            }
        }
        else
        {
            new_point[i+1] = 0 ;
        }
       // qDebug() << "distancia entre " << i << " e " << i+1 << ":" << distanceArray[i] << "novo ponto : " << new_point[i] ;
     }






       namedWindow( "trial", WINDOW_AUTOSIZE );
       imshow( "trial", frame2 );


       for (int a = 0 ; a < contours_poly[largest_contour_index].size() ; a ++ )
       {
           circle( frame2, contours_poly[largest_contour_index][a], 4, Scalar(0,255,255), 1, 8, 0 );
       }

       namedWindow( "trialPoly", WINDOW_AUTOSIZE );
       imshow( "trialPoly", frame2 );



       qDebug() <<"\n" << hullsI.size();
       qDebug() << "\n" << hullsI[largest_contour_index].size() ;
       qDebug() << "\npolyHull :" << contours_poly[largest_contour_index].size() ;


       //test to put center mark in place
       Mat center_test;
       int dilation_size = 1 ;
       Mat element2 = getStructuringElement( MORPH_ELLIPSE,
                                            Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                            Point( dilation_size, dilation_size ) );
       // Apply the dilation operation
       dilate( erosion_dst , center_test , element2 );
       namedWindow( "center", WINDOW_AUTOSIZE );
       imshow( "center", center_test );


       // get the mass center of a black and white image
       // another try for mass center
       Moments m = moments(center_test, false);
       Point p1(m.m10/m.m00, m.m01/m.m00);
       Point delta(0,40);
       Point p;
       p = p1 - delta ;

       circle(center_test, p, 5, Scalar(128,0,0), -1);
       circle(cropped, p, 5, Scalar(128,0,0), -1);
       imshow("center", center_test);
       imshow("cropped", cropped);


    // createTrackbar( "cany_thres:", "canny_image", &lowThreshold, max_lowThreshold, canny_thres );
    // canny_thres(0,0);



     // resultado melhor com gaussian e thres de aprox 11 e razao 3

     /* begin trackbars


     createTrackbar( "Element dilation :\n 0: Rect \n 1: Cross \n 2: Ellipse", "canny_image",
                      &dilation_elem, max_elem,
                      Dilation );

    createTrackbar( "Kernel size dilation:\n 2n +1", "canny_image",
                      &dilation_size, max_kernel_size,
                      Dilation );
    Dilation( 0, 0 );

    /// Create Erosion Trackbar
     createTrackbar( "Element eroison:\n 0: Rect \n 1: Cross \n 2: Ellipse", "canny_image",
                 &erosion_elem, max_elem,
             Erosion );

     createTrackbar( "Kernel size erosion:\n 2n +1", "canny_image",
             &erosion_size, max_kernel_size,
             Erosion );

     Erosion (0,0);

*/ // enf of track bars


     //Canny( cropped, cropped_binary, 50, 200, 3 );

   //  imshow( "canny_image", cropped_binary );
/*
     // contours
     vector<vector<Point> > contours;
     vector<Vec4i> hierarchy;
     RNG rng(12345);
     findContours( cropped_binary, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

     // Find the convex hull object for each contour
        vector<vector<Point> >hull( contours.size() );
        for( int i = 0; i < contours.size(); i++ )
        {
            convexHull( Mat(contours[i]), hull[i], false );
        }

     Mat drawing = Mat::zeros( cropped_binary.size(), CV_8UC3 );
        for( int i = 0; i< contours.size(); i++ )
           {
             Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
             drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
             drawContours( drawing, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
           }

        /// Show in a window
        namedWindow( "contours", WINDOW_AUTOSIZE );
        imshow( "contours", drawing );

*/


    // end of another try using canny

     // -----------------------
    // Gray scale

    /*
     *
    Mat image_gray(image.size(),CV_8UC1);
    //Mat binary_image ;

    cvtColor( image, image_gray, CV_BGR2GRAY );

    Mat cdst ;


    namedWindow("window", CV_WINDOW_AUTOSIZE );

    //createTrackbar( "Min Threshold:", "window", &lowThreshold, max_lowThreshold, on_trackbar );

    //Canny( image_gray, binary_image, 50, 50*3, 3 );
      //createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );
    //blur( image_gray, binary_image, Size(9,9) );



    Canny( image, binary_image, 50, 200, 3 );
    cvtColor(binary_image, cdst, CV_GRAY2BGR);
    /*
     * createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "window",
                      &dilation_elem, max_elem,
                      Dilation );

    //createTrackbar( "Kernel size:\n 2n +1", "window",
                      &dilation_size, max_kernel_size,
                      Dilation );
    Dilation( 0, 0 );


    vector<Vec2f> lines;
    HoughLines(binary_image, lines, 1, CV_PI/180, 30, 0, 0 );

    for( size_t i = 0; i < lines.size(); i++ )
    {
      float rho = lines[i][0], theta = lines[i][1];
      Point pt1, pt2;
      double a = cos(theta), b = sin(theta);
      double x0 = a*rho, y0 = b*rho;
      pt1.x = cvRound(x0 + 1000*(-b));
      pt1.y = cvRound(y0 + 1000*(a));
      pt2.x = cvRound(x0 - 1000*(-b));
      pt2.y = cvRound(y0 - 1000*(a));
      line( cdst, pt1, pt2, Scalar(0,0,255), 1, CV_AA);
    }
    //imshow("source", image);
   // imshow("window", image);

    imshow("window", binary_image) ;
    *
    * *
    */


    //hconcat(image, image2, newImage);


    /*
     namedWindow("Object Detection", CV_WINDOW_AUTOSIZE );
     namedWindow("Original", CV_WINDOW_AUTOSIZE );
     frame = image;
     newImage = image;
        //-- Trackbars to set thresholds for RGB values
     createTrackbar("Low R","Object Detection", &low_r, 255, on_low_r_thresh_trackbar);
        createTrackbar("High R","Object Detection", &high_r, 255, on_high_r_thresh_trackbar);
        createTrackbar("Low G","Object Detection", &low_g, 255, on_low_g_thresh_trackbar);
        createTrackbar("High G","Object Detection", &high_g, 255, on_high_g_thresh_trackbar);
        createTrackbar("Low B","Object Detection", &low_b, 255, on_low_b_thresh_trackbar);
        createTrackbar("High B","Object Detection", &high_b, 255, on_high_b_thresh_trackbar);

*/
     // begin of hand segmentation

     // color threasholding. these values were obtained empirically using trackbars
    // inRange(image,Scalar(0,128,0), Scalar(255,239,174),frame_threshold);

        /*

     // invert image
     bitwise_not ( frame_threshold, frame_threshold );

     // contours
     vector<vector<Point> > contours;
     vector<Vec4i> hierarchy;
     RNG rng(12345);
     findContours( frame_threshold, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );


     // convex hull

     // Find the convex hull object for each contour
        vector<vector<Point> >hull( contours.size() );
        for( int i = 0; i < contours.size(); i++ )
        {
            convexHull( Mat(contours[i]), hull[i], false );
        }




     Mat drawing = Mat::zeros( frame_threshold.size(), CV_8UC3 );
        for( int i = 0; i< contours.size(); i++ )
           {
             Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
             drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
             drawContours( drawing, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
           }

        /// Show in a window
        namedWindow( "Hull demo", WINDOW_AUTOSIZE );
        imshow( "Hull demo", drawing );


        */

     //imshow("Object Detection",frame_threshold);


     //imshow("Original",image);



}



void MainWindow::on_actionEscolher_novo_diretorio_triggered()
{
    on_diretorio_clicked();
}

void MainWindow::on_actionSalvar_triggered()
{
    on_save_clicked();
}


void MainWindow::on_actionComo_usar_o_programa_triggered()
{
    QMessageBox::information(this,"Ajuda","Digitar ajuda aqui");
}

void MainWindow::on_actionSobre_triggered()
{
    QMessageBox::information(this,"Sobre","Digitar sobre os autores aqui");
}
