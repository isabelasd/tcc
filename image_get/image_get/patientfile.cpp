#include "patientfile.h"
#include "ui_patientfile.h"




// list elements of a xml file. used to debug or to save it to some variable
void patientFile::ListElement(QDomElement root, QString tagname, QString attribute)
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

// verifica se tem ficha do paciente em xml.
// melhorar depois para checar se existe a ficha do paciente ID x que sera encontrado no diretorio.
// esse numero ta no nome do dir das fotos
void patientFile::UpdateXML_patient()
{
    // xml saving
    QDomDocument document;
    QDomElement root = document.createElement("Paciente");
    document.appendChild(root);

    QDomElement name = document.createElement("Name");
    name.setAttribute("Name", "banco de dados " );
    root.appendChild(name);

    QDomElement age = document.createElement("Age");
    age.setAttribute("Age", "banco de dados " );
    root.appendChild(age);

    // write to file
    QFile file(dir_patient + "/patient_file.xml");

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

void patientFile::ReadXML_patient()
{
    // xml saving
    QDomDocument document;

    // Load to file
    QFile file(dir_patient + "/patient_file.xml");

    if (!file.open(QIODevice::ReadOnly))
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

    // List the elements
    ListElement(root,"Name", "Name");

    qDebug() << "\nFinished reading xml" ;


}

QString patientFile::get_dir_patient()
{
    return dir_patient;
}

void patientFile::set_dir_patient(QString dir_new)
{
    dir_patient = dir_new ;
}

bool patientFile::fileExists(QString path) {
    QFileInfo check_file(path);
    // check if file exists and if yes: Is it really a file and no directory?
    return check_file.exists() && check_file.isFile();
}


patientFile::patientFile(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::patientFile)
{
    ui->setupUi(this);


}

patientFile::~patientFile()
{
    delete ui;
}

void patientFile::on_save_clicked()
{
    // save changes in data base or xml
    // implementar aqui su , ler todos os dados de patientfile.ui e salvar no xml
    if (!fileExists(dir_patient+"/ficha.xml"))
    {
        UpdateXML_patient();
    }

    ReadXML_patient();

}

void patientFile::on_cancel_clicked()
{
    // dont save changes, just close patientFileDialog
    this->close();

}
