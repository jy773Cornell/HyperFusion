// Qt application entry point and main window bootstrap.
#include "frontend/widgets/MainWindow.hpp"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    MainWindow window;
    window.show();

    return app.exec();
}
