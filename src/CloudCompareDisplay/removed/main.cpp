#include "mainwindow.h"
#include <QtWidgets/QApplication>
#include<qdir.h>

int main(int argc, char *argv[])
{
	AllocConsole();
	freopen("CONOUT$", "w", stdout);
	freopen("CONOUT$", "w", stderr);
	QDir  workingDir = "D:\\vc_projects\\CloudCompareDisplay\\CloudCompareDisplay\\";
	ccGLWindow::setShaderPath(QString(workingDir.absolutePath() + "/shaders"));
	QApplication a(argc, argv);
	//main window init.
	MainWindow* mainWindow = MainWindow::TheInstance();
	if (!mainWindow)
	{
		printf("initial window failed...\n");
		return 0;
	}
	mainWindow->show();
	/*MainWindow w;
	w.show();*/
	return a.exec();
}
