#include "MagPredictorWidget.h"
#include <QApplication>
#include <iostream>
#include <DbgHelp.h>

using namespace std;

//程式异常捕获
LONG ApplicationCrashHandler(EXCEPTION_POINTERS *pException)
{
    /*
      ***保存数据代码***  创建 Dump 文件
    */
    QString time = QTime::currentTime().toString("HH_mm_ss_zzz");
    QString fileName = "AppCrash_" + time + ".dmp";
    HANDLE hDumpFile = CreateFile((LPCWSTR)(fileName).utf16(),
                                  GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

    if( hDumpFile != INVALID_HANDLE_VALUE){
        //Dump信息
        MINIDUMP_EXCEPTION_INFORMATION dumpInfo;
        dumpInfo.ExceptionPointers = pException;
        dumpInfo.ThreadId = GetCurrentThreadId();
        dumpInfo.ClientPointers = TRUE;

        //写入Dump文件内容
        MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(),
                          hDumpFile, MiniDumpNormal, &dumpInfo, NULL, NULL);
    }

    //这里弹出一个错误对话框并退出程序
    EXCEPTION_RECORD* record = pException->ExceptionRecord;
    QString errCode(QString::number(record->ExceptionCode, 16)),
            errAdr(QString::number((uint)record->ExceptionAddress,16)), errMod;
    QMessageBox::critical(NULL,QStringLiteral("程序崩溃"),
        QStringLiteral("<FONT size=4><div><b>对于发生的错误，表示诚挚的歉意</b><br/></div>")
      + QStringLiteral("<div>错误代码：%1</div><div>错误地址：%2</div></FONT>").arg(errCode).arg(errAdr),
        QMessageBox::Ok);

    return EXCEPTION_EXECUTE_HANDLER;
}


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

//    QTextCodec *codec = QTextCodec::codecForName("UTF-8");
//    QTextCodec::setCodecForLocale(codec);
    SetUnhandledExceptionFilter((LPTOP_LEVEL_EXCEPTION_FILTER)ApplicationCrashHandler);//注冊异常捕获函数

    MagPredictorWidget w;
    w.show();

//    coilTest(1);

    return a.exec();
}
