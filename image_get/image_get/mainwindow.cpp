#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPixmap>
#include <QFileDialog>
#include <QMessageBox>
#include <QDir>



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

    Mat image_gray(image.size(),CV_8UC1);

    // IT MUST BE BGR TO GRAY, OTHERWISE THE GRAYSCALE ITS NOT AS IT SHOULD BE
    // comparing to MATLAB scale, this is the most similar one
    cvtColor(image,image_gray,CV_BGR2GRAY);

    imshow("Foto gray",image_gray);


    // calculate average on the first rectangle

    int intensity = 0;
    int aux = 0;
    int el_count = 0 ;

    for (int i = coordinates[0] + 1; i < coordinates[2]; i++ )
    {
        for (int j = coordinates[1] + 1 ; j < coordinates[3]; j++ )
        {
            aux = image_gray.at<uchar>(Point(i,j));
            intensity = intensity + aux ;
           // aux = 0;
            //qDebug() << "\n intensity_" << i << "_" << j << ": " << QString::number(aux) << " intensity_var : " << intensity ;
            //intensity = 0 ;
            el_count++;
        }
    }
    intensity = intensity/el_count;

    qDebug() << "\naverage = " << intensity << " elementos : " << el_count ;

    /* --------------------------

      criar funcao do tipo calculate_average (imageGrayScale, xi, yi, xf, yf)


      TESTE TESTE TEST
   ------------------------------*/
   // int intensidade (const Mat &image)



    /* -----------------------------------
    // calculating average of a 4 connected-pixel value
    int n = 5;
    int intensity[n] = {};



    // get value of a pixel (grayscale)
    intensity[0] = image_gray.at<uchar>(Point(coordinates[0],coordinates[1]));
    qDebug() << "\nx top left :" << QString::number(coordinates[0]) << " y: " << QString::number(coordinates[1]) << " color:" <<  QString::number(intensity[0]) ;

    intensity[1] = image_gray.at<uchar>(Point(coordinates[0],coordinates[1]+1));
    qDebug() << "\nx above :" << QString::number(coordinates[0]) << " y: " << QString::number(coordinates[1]+1) << " color:" <<  QString::number(intensity[1]) ;

    intensity[2] = image_gray.at<uchar>(Point(coordinates[0]+1,coordinates[1]));
    qDebug() << "\nx right :" << QString::number(coordinates[0]+1) << " y: " << QString::number(coordinates[1]) << " color:" <<  QString::number(intensity[2]) ;

    intensity[3] = image_gray.at<uchar>(Point(coordinates[0],coordinates[1]-1));
    qDebug() << "\nx bellow :" << QString::number(coordinates[0]) << " y: " << QString::number(coordinates[1]-1) << " color:" <<  QString::number(intensity[3]) ;

    intensity[4] = image_gray.at<uchar>(Point(coordinates[0]-1,coordinates[1]));
    qDebug() << "\nx left :" << QString::number(coordinates[0]-1) << " y: " << QString::number(coordinates[1]) << " color:" <<  QString::number(intensity[4]) ;

    qDebug() << "\ncoordinates x :" << QString::number(coordinates[0]) << " y: " << QString::number(coordinates[2]);

    double meanColor;

    for (int i = 0 ; i < n ; i++)
    {
        meanColor += intensity[i];
    }

    meanColor = floor(meanColor/n);

    qDebug() << "\nmedia : " << QString::number(meanColor) ;

    ----------------------------------------------- */


    // get value of a BGR pixel
    /*
    Vec3b intensityBGR = image.at<Vec3b>(Point(1,1));
    uchar blue = intensityBGR.val[0];
    uchar green = intensityBGR.val[1];
    uchar red = intensityBGR.val[2];
    qDebug() << "\n2: B" <<  blue << " G:" << green <<  " R:" << red;
    */




}
