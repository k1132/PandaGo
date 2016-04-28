#include <QtWidgets/QApplication>
#include "Src/pandago.h"

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	PandaGo w;
	w.show();
	return a.exec();
}
