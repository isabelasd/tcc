#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPixmap>
#include <QFileDialog>
#include <QMessageBox>


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

bool fileExists(QString path) {
    QFileInfo check_file(path);
    // check if file exists and if yes: Is it really a file and no directory?
    return check_file.exists() && check_file.isFile();
}

void MainWindow::CreateXML()
{
    // xml saving
    QDomDocument document;
    QDomElement root = document.createElement("Fotos");
    document.appendChild(root);

    for (int i =0 ; i < PICTURE_NUMBER ; i++)
    {
        QDomElement photos = document.createElement("Fotos");
        photos.setAttribute("Name", "Foto " + QString::number(i) );
        photos.setAttribute("ID", QString::number(i) );
        root.appendChild(photos);

        for (int j = 0; j < MARKERS_NUMBER ; j++)
        {
            QDomElement markers = document.createElement("Marcadores");
            markers.setAttribute("Name", "Marcador " + QString::number(j+1) );
            markers.setAttribute("Temperatura", QString::number(j+1) );
            photos.appendChild(markers);
        }
    }


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

 void ListElement(QDomElement root, QString tagname, QString attribute)
 {
     QDomNodeList items = root.elementsByTagName(tagname);
     qDebug() << "Total items = " << items.count();

     for (int i = 0 ; i < items.count(); i++)
     {
         QDomNode itemnode = items.at(i) ;

         //convert to element
         if(itemnode.isElement())
         {
             QDomElement itemele = itemnode.toElement();
             qDebug() << itemele.attribute(attribute);
         }
     }
 }



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    scene = new QGraphicsScene(this);
    ui->graphicsView->setScene(scene);

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
    }

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_diretorio_clicked()
{

    dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                    QDir::homePath(),
                                                    QFileDialog::ShowDirsOnly
                                                    | QFileDialog::DontResolveSymlinks);

    qDebug() << dir ;

    QDir image_folder(dir);
    image_folder.setNameFilters( QStringList() << "*.png" ) ;
    image_list = image_folder.entryList() ;
    for (int i = 0 ; i < image_list.size() ; i++)
    {
        qDebug() << "\nList(" << i << "):" << image_list.at(i) ;
    }

    QString first_image = dir+"/"+image_list.at(1) ;

    qDebug() << "\nprimeiro path:" << first_image ;

    //----------
    // checking if xml file exists
    // xml initialization
    if (!fileExists(dir+"/temperatura.xml"))
    {
        CreateXML();
    }

    ReadXML();

    // open image to label  - not used anymore //
    /* QPixmap pix1(file_name) ;
    int w_pic1 = ui->label_pic->width() ;
    int h_pic1 = ui->label_pic->height() ;
    ui->label_pic->setPixmap( pix1.scaled(w_pic1,h_pic1,Qt::KeepAspectRatio ) ) ; */

    // scene begin
    // add image

    scene->addPixmap(first_image);

    // add markers
    // markers are displayed in increasing number sequence, from left to right

    //central marker
    QBrush blackBrush(Qt::black);
    QPen blackpen(Qt::black);
    blackpen.setWidth(0);

    mark1 = scene ->addRect(xc,yc,wc,hc,blackpen);
    mark1->setFlag( QGraphicsItem::ItemIsMovable );


    // fingers markers


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

    path_att = dir + "/" + image_list.at(image_number);
    path_att_cv  = path_att.toUtf8().constData() ;

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
    qDebug() << "X1 : "    << QString::number(coordinates[0]) <<
            " Y1 : "   << QString::number(coordinates[1]) <<
            "\n X2 : " << QString::number(coordinates[2]) <<
            " Y2 : "   << QString::number(coordinates[3]) <<
            "\n X3 : " << QString::number(coordinates[4]) <<
            " Y3 : "   << QString::number(coordinates[5]) <<
            "\n X4 : " << QString::number(coordinates[6]) <<
            " Y4 : "   << QString::number(coordinates[7]) <<
            "\n X5 : " << QString::number(coordinates[8]) <<
            " Y5 : "   << QString::number(coordinates[9]) <<
            "\n X6 : " << QString::number(coordinates[10]) <<
            " Y6 : "   << QString::number(coordinates[11]) <<
            "X7 : "    << QString::number(coordinates[12]) <<
                        " Y7 : "   << QString::number(coordinates[13]) <<
                        "\n X8 : " << QString::number(coordinates[14]) <<
                        " Y8 : "   << QString::number(coordinates[15]) <<
                        "\n X9 : " << QString::number(coordinates[16]) <<
                        " Y9 : "   << QString::number(coordinates[17]) <<
                        "\n X10 : " << QString::number(coordinates[18]) <<
                        " Y10 : "   << QString::number(coordinates[19]) <<
                        "\n X11 : " << QString::number(coordinates[20]) <<
                        " Y11 : "   << QString::number(coordinates[21]) <<
                        "\n X12 : " << QString::number(coordinates[22]) <<
                        " Y12 : "   << QString::number(coordinates[23])
            ;



    // read image (using the path)
    image = imread( path_att_cv );

    // create image window
   // namedWindow("Foto");

    // draw markers    
    rectangle(image,Point(coordinates[0],coordinates[1]), Point(coordinates[2],coordinates[3]), Scalar(0,0,0), 1);
    rectangle(image,Point(coordinates[4],coordinates[5]), Point(coordinates[6],coordinates[7]), Scalar(0,0,0), 1);
    rectangle(image,Point(coordinates[8],coordinates[9]), Point(coordinates[10],coordinates[11]), Scalar(0,0,0), 1);
    rectangle(image,Point(coordinates[12],coordinates[13]), Point(coordinates[14],coordinates[15]), Scalar(0,0,0), 1);
    rectangle(image,Point(coordinates[16],coordinates[17]), Point(coordinates[18],coordinates[19]), Scalar(0,0,0), 1);
    rectangle(image,Point(coordinates[20],coordinates[21]), Point(coordinates[22],coordinates[23]), Scalar(0,0,0), 1);

    //show image -> image loaded in BGR format
   // imshow("Foto",image);

     // -----------------------
    // Gray scale
    Mat image_gray(image.size(),CV_8UC1);

    // IT MUST BE BGR TO GRAY, OTHERWISE THE GRAYSCALE ITS NOT AS IT SHOULD BE
    // comparing to MATLAB scale, this is the most similar one
    cvtColor(image,image_gray,CV_BGR2GRAY);

    //imshow("Foto gray",image_gray);

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

    // set mark x and y values to get markers position
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


   //-----------------------------------------------------

}

//spinbox used for bigger marker. updates its size
void MainWindow::on_spinBox_valueChanged(int arg1)
{

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

    qreal x_temp = x_markers[image_number*6] ;
    qreal y_temp = y_markers[image_number*6] ;

    // redraw marker and set reference to 0, 0 (scene reference). this is important, otherwise doesnt work.
    mark1 -> setRect(QRectF(x_markers[image_number*6],y_markers[image_number*6] , wc, hc)) ;
    mark1 -> setPos(0,0);



}


//spinbox used for smaller markers. updates the size of the markers
void MainWindow::on_spinBox_2_valueChanged(int arg1)
{
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


    // redraw marker and set reference to 0, 0 (scene reference). this is important, otherwise doesnt work.
    mark2 -> setRect(QRectF(x_markers[image_number*6+1],y_markers[image_number*6+1] , wf, hf)) ;
    mark2 -> setPos(0,0);
    mark3 -> setRect(QRectF(x_markers[image_number*6+2],y_markers[image_number*6+2] , wf, hf)) ;
    mark3 -> setPos(0,0);
    mark4 -> setRect(QRectF(x_markers[image_number*6+3],y_markers[image_number*6+3] , wf, hf)) ;
    mark4 -> setPos(0,0);
    mark5 -> setRect(QRectF(x_markers[image_number*6+4],y_markers[image_number*6+4] , wf, hf)) ;
    mark5 -> setPos(0,0);
    mark6 -> setRect(QRectF(x_markers[image_number*6+5],y_markers[image_number*6+5] , wf, hf)) ;
    mark6 -> setPos(0,0);
}

void MainWindow::on_proximo_clicked()
{
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

    QPen blackpen(Qt::black);
    blackpen.setWidth(0);
    // central marker
    mark1 = scene ->addRect(x_markers[image_number*6],y_markers[image_number*6],wc,hc,blackpen);
    mark1->setFlag( QGraphicsItem::ItemIsMovable );
    // fingers markers
    // dedao
    mark2 = scene ->addRect(x_markers[image_number*6+1],y_markers[image_number*6+1],wf,hf,blackpen);
    mark2->setFlag( QGraphicsItem::ItemIsMovable );
    // indicador
    mark3 = scene ->addRect(x_markers[image_number*6+2],y_markers[image_number*6+2],wf,hf,blackpen);
    mark3->setFlag( QGraphicsItem::ItemIsMovable );
    // medio
    mark4 = scene ->addRect(x_markers[image_number*6+3],y_markers[image_number*6+3],wf,hf,blackpen);
    mark4->setFlag( QGraphicsItem::ItemIsMovable );
    // anelar
    mark5 = scene ->addRect(x_markers[image_number*6+4],y_markers[image_number*6+4],wf,hf,blackpen);
    mark5->setFlag( QGraphicsItem::ItemIsMovable );
    // minimo
    mark6 = scene ->addRect(x_markers[image_number*6+5],y_markers[image_number*6+5],wf,hf,blackpen);
    mark6->setFlag( QGraphicsItem::ItemIsMovable );
    /*
    for (int i = 0 ; i < image_list.size() ; i++)
    {
        qDebug() << "\nList(" << i << "):" << image_list.at(i) ;
    }

    QString first_el = dir+"/"+image_list.at(1) ;

    qDebug() << "\nprimeiro path:" << first_el ;
*/

}

void MainWindow::on_anterior_clicked()
{
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

    QPen blackpen(Qt::black);
    blackpen.setWidth(0);
    // central marker
    mark1 = scene ->addRect(x_markers[image_number*6],y_markers[image_number*6],wc,hc,blackpen);
    mark1->setFlag( QGraphicsItem::ItemIsMovable );
    // fingers markers
    // dedao
    mark2 = scene ->addRect(x_markers[image_number*6+1],y_markers[image_number*6+1],wf,hf,blackpen);
    mark2->setFlag( QGraphicsItem::ItemIsMovable );
    // indicador
    mark3 = scene ->addRect(x_markers[image_number*6+2],y_markers[image_number*6+2],wf,hf,blackpen);
    mark3->setFlag( QGraphicsItem::ItemIsMovable );
    // medio
    mark4 = scene ->addRect(x_markers[image_number*6+3],y_markers[image_number*6+3],wf,hf,blackpen);
    mark4->setFlag( QGraphicsItem::ItemIsMovable );
    // anelar
    mark5 = scene ->addRect(x_markers[image_number*6+4],y_markers[image_number*6+4],wf,hf,blackpen);
    mark5->setFlag( QGraphicsItem::ItemIsMovable );
    // minimo
    mark6 = scene ->addRect(x_markers[image_number*6+5],y_markers[image_number*6+5],wf,hf,blackpen);
    mark6->setFlag( QGraphicsItem::ItemIsMovable );

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
        qDebug() << "\nLista(" << i << "):" << image_list.at(i) ;

        // updates path
        path_att = dir + "/" + image_list.at(i);
        path_att_cv  = path_att.toUtf8().constData() ;

        // recreate pixmap
        scene -> addPixmap( path_att );

        QPen blackpen(Qt::black);
        blackpen.setWidth(0);
        // central marker
        mark1 = scene ->addRect(x_markers[i*6],y_markers[i*6],wc,hc,blackpen);
        // fingers markers
        // dedao
        mark2 = scene ->addRect(x_markers[i*6+1],y_markers[i*6+1],wf,hf,blackpen);
        // indicador
        mark3 = scene ->addRect(x_markers[i*6+2],y_markers[i*6+2],wf,hf,blackpen);
        // medio
        mark4 = scene ->addRect(x_markers[i*6+3],y_markers[i*6+3],wf,hf,blackpen);
        // anelar
        mark5 = scene ->addRect(x_markers[i*6+4],y_markers[i*6+4],wf,hf,blackpen);
        // minimo
        mark6 = scene ->addRect(x_markers[i*6+5],y_markers[i*6+5],wf,hf,blackpen);

        QPixmap pixMap = this->ui->graphicsView->grab();

        file_name_saved = file_name_oiginal + QString::number(i);


        pixMap.save(dir + "/editadas/"  + file_name_saved + ".png");
       // qDebug() << "\nLista editada 1 (" << i << "):" << image_list_save.at(i) ;


    }

    dirSave.setNameFilters( QStringList() << "*.png" ) ;
    image_list_save = dirSave.entryList() ;
    for (int i = 0 ; i < image_list_save.size() ; i++)
    {
        qDebug() << "\nLista editada 2 (" << i << "):" << image_list_save.at(i) ;
    }


}

void MainWindow::ReadXML()
{
    // xml saving
    QDomDocument document;

    // Load to file
    QFile file(dir + "/temperatura.xml");

    if (!file.open(QIODevice::ReadWrite))
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

    // List the photos
    ListElement(root,"Fotos", "Name");

    qDebug() << "\nFinished reading xml" ;

    //get Markers and temperature
    QDomNodeList photos = root.elementsByTagName("Fotos");
    for(int i = 0 ; i < photos.count() ; i++)
    {
        QDomNode photonode = photos.at(i);
        // convert to an element
        if(photonode.isElement())
        {
            QDomElement photo = photonode.toElement();
            ListElement(photo,"Marcadores", "Temperatura");
        }
    }


}


void MainWindow::WriteXML()
{

}


