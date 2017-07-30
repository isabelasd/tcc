#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPixmap>
#include <QFileDialog>
#include <QMessageBox>
#include <QDir>


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


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    scene = new QGraphicsScene(this);
    ui->graphicsView->setScene(scene);

    QGraphicsEllipseItem mark1;


}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    QString filter = "Todos os arquivos (*.*) ;; Imagem PNG (*.png)" ;
    QString file_name = QFileDialog::getOpenFileName( this,"Abrir um arquivo",QDir::homePath(),filter ) ;
   // QMessageBox::information( this,"..",file_name ) ;

    // open image to label  - not used anymore //
    /* QPixmap pix1(file_name) ;
    int w_pic1 = ui->label_pic->width() ;
    int h_pic1 = ui->label_pic->height() ;
    ui->label_pic->setPixmap( pix1.scaled(w_pic1,h_pic1,Qt::KeepAspectRatio ) ) ; */

    // scene begin
    // add image
    scene->addPixmap(file_name);

    // add markers
    // markers are displayed in increasing number sequence, from left to right

    //central marker
    QBrush blackBrush(Qt::black);
    QPen blackpen(Qt::black);
    blackpen.setWidth(0);

    qreal xc, yc, wc, hc ;
    xc = 10;
    yc = 30;
    wc = 8 ;
    hc = 8;

    mark1 = scene ->addRect(xc,yc,wc,hc,blackpen);
    mark1->setFlag( QGraphicsItem::ItemIsMovable );


    // fingers markers
    qreal xf, yf, wf, hf ;
    xf = 30;
    yf = 30;
    wf = 4 ;
    hf = 4;

    // dedao
    mark2 = scene ->addRect(xf,yf,wf,hf,blackpen);
    mark2->setFlag( QGraphicsItem::ItemIsMovable );
    // indicador
    mark3 = scene ->addRect(xf+20,yf,wf,hf,blackpen);
    mark3->setFlag( QGraphicsItem::ItemIsMovable );
    // medio
    mark4 = scene ->addRect(xf+40,yf,wf,hf,blackpen);
    mark4->setFlag( QGraphicsItem::ItemIsMovable );
    // anelar
    mark5 = scene ->addRect(xf+60,yf,wf,hf,blackpen);
    mark5->setFlag( QGraphicsItem::ItemIsMovable );
    // minimo
    mark6 = scene ->addRect(xf+80,yf,wf,hf,blackpen);
    mark6->setFlag( QGraphicsItem::ItemIsMovable );


    file_name_cv  = file_name.toUtf8().constData() ;
}


void MainWindow::on_pushButton_2_clicked()
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

    // debug
    /*
    ui->label->setText("X1 : "    + QString::number(coordinates[0]) +
                         " Y1 : "   + QString::number(coordinates[1]) +
                         "\n X2 : " + QString::number(coordinates[3]) +
                         " Y2 : "   + QString::number(coordinates[4]) +
                         "\n X3 : " + QString::number(coordinates[5]) +
                         " Y3 : "   + QString::number(coordinates[6]) +
                         "\n X4 : " + QString::number(coordinates[7]) +
                         " Y4 : "   + QString::number(coordinates[8]) +
                         "\n X5 : " + QString::number(coordinates[9]) +
                         " Y5 : "   + QString::number(coordinates[10]) +
                         "\n X6 : " + QString::number(coordinates[11]) +
                         " Y6 : "   + QString::number(coordinates[12])
                         );

                         */

    // read image (using the path)
    image = imread( file_name_cv );

    // create image window
    namedWindow("Foto");

    // draw markers    
    rectangle(image,Point(coordinates[0],coordinates[1]), Point(coordinates[2],coordinates[3]), Scalar(0,0,0), 1);
    rectangle(image,Point(coordinates[4],coordinates[5]), Point(coordinates[6],coordinates[7]), Scalar(0,0,0), 1);
    rectangle(image,Point(coordinates[8],coordinates[9]), Point(coordinates[10],coordinates[11]), Scalar(0,0,0), 1);
    rectangle(image,Point(coordinates[12],coordinates[13]), Point(coordinates[14],coordinates[15]), Scalar(0,0,0), 1);
    rectangle(image,Point(coordinates[16],coordinates[17]), Point(coordinates[18],coordinates[19]), Scalar(0,0,0), 1);
    rectangle(image,Point(coordinates[20],coordinates[21]), Point(coordinates[22],coordinates[23]), Scalar(0,0,0), 1);

    //show image -> image loaded in BGR format
    imshow("Foto",image);

     // -----------------------
    // Gray scale
    Mat image_gray(image.size(),CV_8UC1);

    // IT MUST BE BGR TO GRAY, OTHERWISE THE GRAYSCALE ITS NOT AS IT SHOULD BE
    // comparing to MATLAB scale, this is the most similar one
    cvtColor(image,image_gray,CV_BGR2GRAY);

    imshow("Foto gray",image_gray);

    //--------------------------------------------


   // section to get average : Grayscale color space
    /*
   int intensityMarkGray1 = 0;
   int intensityMarkGray2 = 0;
   int intensityMarkGray3 = 0;
   int intensityMarkGray4 = 0;
   int intensityMarkGray5 = 0;
   int intensityMarkGray6 = 0;


   intensityMarkGray1 = getIntensityGray(image_gray, coordinates[0],coordinates[1],coordinates[2],coordinates[3]);
   intensityMarkGray2 = getIntensityGray(image_gray, coordinates[4],coordinates[5],coordinates[6],coordinates[7]);
   intensityMarkGray3 = getIntensityGray(image_gray, coordinates[8],coordinates[9],coordinates[10],coordinates[11]);
   intensityMarkGray4 = getIntensityGray(image_gray, coordinates[12],coordinates[13],coordinates[14],coordinates[15]);
   intensityMarkGray5 = getIntensityGray(image_gray, coordinates[16],coordinates[17],coordinates[18],coordinates[19]);
   intensityMarkGray6 = getIntensityGray(image_gray, coordinates[20],coordinates[21],coordinates[22],coordinates[23]);

   qDebug() << "\nintensidade 1 :" << QString::number(intensityMarkGray1) <<
               "\nintensidade 2 :" << QString::number(intensityMarkGray2) <<
               "\nintensidade 3 :" << QString::number(intensityMarkGray3) <<
               "\nintensidade 4 :" << QString::number(intensityMarkGray4) <<
               "\nintensidade 5 :" << QString::number(intensityMarkGray5) <<
               "\nintensidade 6 :" << QString::number(intensityMarkGray6)
               ;

               */
   //-------------------------------------------------

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
   qDebug() << "\nintensidade 1 : " << QString::number(intensityMark1[0]) <<
                              "," << QString::number(intensityMark1[1]) <<
                              "," << QString::number(intensityMark1[2])
               ;

   getIntensityBGR(image, coordinates[4],coordinates[5],coordinates[6],coordinates[7], intensity);
   intensityMark2[0] = intensity[0];
   intensityMark2[1] = intensity[1];
   intensityMark2[2] = intensity[2];
   qDebug() << "\nintensidade 2 : " << QString::number(intensityMark2[0]) <<
                              "," << QString::number(intensityMark2[1]) <<
                              "," << QString::number(intensityMark2[2])
               ;

   getIntensityBGR(image, coordinates[8],coordinates[9],coordinates[10],coordinates[11], intensity);
   intensityMark3[0] = intensity[0];
   intensityMark3[1] = intensity[1];
   intensityMark3[2] = intensity[2];
   qDebug() << "\nintensidade 3 : " << QString::number(intensityMark3[0]) <<
                              "," << QString::number(intensityMark3[1]) <<
                              "," << QString::number(intensityMark3[2])
               ;

   getIntensityBGR(image, coordinates[12],coordinates[13],coordinates[14],coordinates[15], intensity);
   intensityMark4[0] = intensity[0];
   intensityMark4[1] = intensity[1];
   intensityMark4[2] = intensity[2];
   qDebug() << "\nintensidade 4 : " << QString::number(intensityMark4[0]) <<
                              "," << QString::number(intensityMark4[1]) <<
                              "," << QString::number(intensityMark4[2])
               ;

   getIntensityBGR(image, coordinates[16],coordinates[17],coordinates[18],coordinates[19], intensity);
   intensityMark5[0] = intensity[0];
   intensityMark5[1] = intensity[1];
   intensityMark5[2] = intensity[2];
   qDebug() << "\nintensidade 5 : " << QString::number(intensityMark5[0]) <<
                              "," << QString::number(intensityMark5[1]) <<
                              "," << QString::number(intensityMark5[2])
               ;

   getIntensityBGR(image, coordinates[20],coordinates[21],coordinates[22],coordinates[23], intensity);
   intensityMark6[0] = intensity[0];
   intensityMark6[1] = intensity[1];
   intensityMark6[2] = intensity[2];

   qDebug() << "\nintensidade 6 : " << QString::number(intensityMark6[0]) <<
                              "," << QString::number(intensityMark6[1]) <<
                              "," << QString::number(intensityMark6[2])
               ;


   // getting temperature out of BGR

   // interpolate each channel

   fill_Color_Channel( temperature, R_channel,     R_channel_interp ) ;
   fill_Color_Channel( temperature, G_channel,     G_channel_interp ) ;
   fill_Color_Channel( temperature, B_channel,     B_channel_interp ) ;
   fill_Color_Channel( temperature, temperature, temperature_interp ) ;

   // debug section

    /*
   for( int i = 0; i < 221; i++ ) {
       qDebug() << "\nB(" << QString::number( i ) << ") = " << QString::number( B_channel_interp[ i ] ) ;
   }*/

   // ----------------------

   // search for match

    int index_min1 = find_closest_match(intensityMark1, R_channel_interp, G_channel_interp, B_channel_interp ) ;
    int index_min2 = find_closest_match(intensityMark2, R_channel_interp, G_channel_interp, B_channel_interp ) ;
    int index_min3 = find_closest_match(intensityMark3, R_channel_interp, G_channel_interp, B_channel_interp ) ;
    int index_min4 = find_closest_match(intensityMark4, R_channel_interp, G_channel_interp, B_channel_interp ) ;
    int index_min5 = find_closest_match(intensityMark5, R_channel_interp, G_channel_interp, B_channel_interp ) ;
    int index_min6 = find_closest_match(intensityMark6, R_channel_interp, G_channel_interp, B_channel_interp ) ;

    qDebug() << "\nIndex = " << QString::number(index_min1) << "  Approx Temp = " << QString::number(temperature_interp[index_min1]) ;
    qDebug() << "\nIndex = " << QString::number(index_min2) << "  Approx Temp = " << QString::number(temperature_interp[index_min2]) ;
    qDebug() << "\nIndex = " << QString::number(index_min3) << "  Approx Temp = " << QString::number(temperature_interp[index_min3]) ;
    qDebug() << "\nIndex = " << QString::number(index_min4) << "  Approx Temp = " << QString::number(temperature_interp[index_min4]) ;
    qDebug() << "\nIndex = " << QString::number(index_min5) << "  Approx Temp = " << QString::number(temperature_interp[index_min5]) ;
    qDebug() << "\nIndex = " << QString::number(index_min6) << "  Approx Temp = " << QString::number(temperature_interp[index_min6]) ;

   //-----------------------------------------------------



}
