#include "mainwindow.h"
#include <QApplication>
#include <QFontDatabase>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    a.setApplicationName("hydrus-station");
    a.setApplicationVersion("0.1");
    a.setOrganizationName("Hydrus Project");
    a.setOrganizationDomain("br.ufsc.hydrus");
    //a.setApplicationDisplayName("Hydrus Station");

    // add the cool font for status display
    QFontDatabase::addApplicationFont("://res/digital_7/digital-7 (mono).ttf");

    MainWindow w;
    w.showMaximized();

    return a.exec();
}
