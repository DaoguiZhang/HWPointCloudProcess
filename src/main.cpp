#include "mainwindow.h"
#include<windows.h>
#include <QtWidgets/QApplication>

//local
#include"hw_db_tree.h"

int main(int argc, char *argv[])
{
	AllocConsole();
	freopen("CONOUT$", "w", stdout);
	freopen("CONOUT$", "w", stderr);

	//QDir  workingDir = "G:/xht/GitHub/HUAWEI/HWPointCloudProcess/src/CloudCompareDisplay";
	QDir  workingDir = "../../src/CloudCompareDisplay";
	ccGLWindow::setShaderPath(QString(workingDir.absolutePath() + "/shaders"));
	//สตภýปฏ
	HW::HWParams::getInstance();
	QApplication a(argc, argv);
	MainWindow w;
	w.show();
	/*PolygonWindow window;
	window.show();*/
	return a.exec();
}
