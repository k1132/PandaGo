#include "moduledataplatform.h"

ModuleDataPlatform::ModuleDataPlatform(QWidget * parent, DataPool * datapool) :
    QDialog(parent),
    dp(datapool),
    ui(new Ui::DataPlatform)
{
    isRead = false;
    db = nullptr;
	sql_count = "SELECT COUNT(*) FROM session";
	sql_extract = "SELECT * FROM session";
    ui->setupUi(this);
    ui->load->setDisabled(true);
    ui->fileAddress->setDisabled(true);
    ui->load->setText("Load");
}

ModuleDataPlatform::~ModuleDataPlatform()
{
    delete ui;
}

void ModuleDataPlatform::on_chooseFile_clicked()
{
    //重置数据信息和洁敏
    isRead = false;
    dp->clear();
    ui->load->setText("Load");

    path = QFileDialog::getOpenFileName(this, tr("Select a DB File"), "/", "SQLITE3 DB Files(*.db)");
    fileInfo = QFileInfo(path);
    QString dialog = "Name: " + fileInfo.fileName() + ";\r\n" + "Size: " + QString::number((int)(fileInfo.size()/(1024*1024))) + " MB;\r\n" + "Time: " + fileInfo.lastModified().toString()+".";

    ui->fileAddress->setText(path);
    ui->fileInfomation->setText(dialog);

    (fileInfo.size()>0) ? (ui->load->setDisabled(false)) : (ui->load->setDisabled(true));
    ui->fileAddress->setDisabled(false);
}

void ModuleDataPlatform::on_fileAddress_textChanged (QString p)
{
    isRead = false;
    ui->load->setText("Load");

    path = p;
    fileInfo = QFileInfo(p);
    QString dialog = "Name: " + fileInfo.fileName() + ";\r\n" + "Size: " + QString::number((int)(fileInfo.size()/(1024*1024))) + " MB;\r\n" + "Time: " + fileInfo.lastModified().toString()+".";
    ui->fileInfomation->setText(dialog);

    (fileInfo.size()>0) ? (ui->load->setDisabled(false)) : (ui->load->setDisabled(true));
    ui->fileAddress->setDisabled(false);
}

void ModuleDataPlatform::on_load_clicked()
{
    if(isRead)
    {
        emit this->close();
        return;
    }
    //清空数据池
    dp->clear();
	//设置数据池的属性
	dp->name = fileInfo.fileName();
	dp->path = fileInfo.absolutePath();
    //尝试打开db文件,失败则清理缓冲区并返回
    if(SQLITE_OK != sqlite3_open_v2(path.toStdString().c_str(), &db, SQLITE_OPEN_READONLY, nullptr))
    {
        QMessageBox::warning(this,"ERROR","SQL ERROR: Error open DB!\n",QMessageBox::Yes);
        return;
    }
    //获得记录个数， 构建并清空缓存
    sqlite3_stmt * stmt_ct(nullptr);
    if(SQLITE_OK != sqlite3_prepare_v2(db, sql_count.c_str(), (int)sql_count.length(), &stmt_ct, nullptr))
    {
        QMessageBox::warning(this, "ERROR", "SQL ERROR: Failed to prepare for count(*)!\n", QMessageBox::Yes);
        sqlite3_close_v2(db);
        return;
    }
    if(SQLITE_ROW == sqlite3_step(stmt_ct))
    {
        dp->end = sqlite3_column_int(stmt_ct,0);
        dp->sysFlow = new PLContext[dp->end];
        memset(dp->sysFlow, 0, sizeof(PLContext)*dp->end);
    }
    sqlite3_finalize(stmt_ct);
	//读取数据，放入堆上分配的数据结构
	unsigned int idx(0);//设置缓存写入位置指示器
	PLContext * plc(nullptr);
	sqlite3_stmt * stmt_extract(nullptr);
	sqlite3_exec(db, "BEGIN TRANSACTION", nullptr, nullptr, nullptr);
	sqlite3_prepare_v2(db, sql_extract.c_str(), (int)sql_extract.length(), &stmt_extract, nullptr);
    while(SQLITE_ROW == sqlite3_step(stmt_extract))
    {
		const PL_FUNC_INPUT * ppfi = (PL_FUNC_INPUT *)sqlite3_column_blob(stmt_extract, 1);
		const PL_CS_DATA * ppcd = (PL_CS_DATA *)sqlite3_column_blob(stmt_extract, 2);
		PLContext * pp = dp->sysFlow + idx;
		pp->sn = idx++;
		memcpy(&pp->pl_func_input, ppfi, sizeof(PL_FUNC_INPUT));
		memcpy(&pp->pl_cs_data, ppcd, sizeof(PL_CS_DATA));
		memset(&pp->pl_prm_data, 0, sizeof(PL_PRM_DATA));
		memset(&pp->pl_cs_data_sim, 0, sizeof(PL_CS_DATA));
		memset(&pp->pl_local_data, 0, sizeof(PL_LOCAL_DATA));
		pp->clock = 0;
		//sqlite3_reset(stmt_extract);
    }
	sqlite3_finalize(stmt_extract);
	sqlite3_exec(db, "COMMIT TRANSACTION", nullptr, nullptr, nullptr);
    sqlite3_close(db);
    //处理数据池的相关信息
    dp->cur = 0;
    (dp->end >1) ? (dp->status = DataPoolValid) : (dp->status = DataPoolEmpty);

    //后处理
    isRead = true;
    ui->load->setText("Close");
    ui->fileInfomation->append("DB Info: frames:" + QString::number(dp->end) + "\r\n");
    return;
}
