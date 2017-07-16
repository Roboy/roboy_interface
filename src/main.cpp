#include <QtGui>
#include <roboy_interface/main_window.hpp>

int main(int argc, char **argv) {

    MyoMaster myoMaster;
    myoMaster.initialize(argc,argv);
    myoMaster.start();

    QApplication app(argc, argv);
    interface::MainWindow w(argc,argv);
    w.myoMaster = &myoMaster;
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
