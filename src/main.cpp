#include <QtGui>
#include <roboy_interface/main_window.hpp>

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
