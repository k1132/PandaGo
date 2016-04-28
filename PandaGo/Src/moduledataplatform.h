#pragma once

#include <QDebug>
#include <QDialog>
#include <QString>
#include <QFileInfo>
#include <QDateTime>
#include <QByteArray>
#include <QFileDialog>
#include <QMessageBox>

#include <vector>
#include <string>

#include "../Robix/protocols/protocol_fu.h"
#include "../Robix/protocols/protocol_pl.h"
#include "../3rdparty/sqlite3.h"

#include "infrastructure.h"
#include "ui_moduledataplatform.h"

namespace Ui {
	class DataPlatform;
}

class ModuleDataPlatform : public QDialog
{
    Q_OBJECT
public:
    explicit ModuleDataPlatform(QWidget *, DataPool *);
    virtual ~ModuleDataPlatform();

private:
    Ui::DataPlatform * ui;
    QString path;
    QFileInfo fileInfo;
    bool isRead;

    sqlite3 * db;
    DataPool * dp;
	std::string sql_count;
	std::string sql_extract;

public slots:
    void on_chooseFile_clicked();
    void on_fileAddress_textChanged(QString);
    void on_load_clicked();
};
