#check Qt version
QT_VERSION = $$[QT_VERSION]
QT_VERSION = $$split(QT_VERSION, ".")
QT_VER_MAJ = $$member(QT_VERSION, 0)
QT_VER_MIN = $$member(QT_VERSION, 1)

win32 {
    lessThan(QT_VER_MAJ, 5) | lessThan(QT_VER_MIN, 10) {
        message(Qt version < 5.10)

        CONFIG(debug, debug|release) {
            LIBS += $$PWD/qt5.5.1+vs2012/GLC_libd.lib
            CVSrcCopyFile = $$PWD/qt5.5.1+vs2012/debug/*

        } else {
            LIBS += $$PWD/qt5.5.1+vs2012/GLC_lib.lib
            CVSrcCopyFile = $$PWD/qt5.5.1+vs2012/release/*

        }
    } else {
        message(Qt version >= 5.10)
        CONFIG(debug, debug|release) {
            LIBS += $$PWD/qt5.15.2+vs2019/GLC_libd.lib
            CVSrcCopyFile = $$PWD/qt5.15.2+vs2019/debug/*

        } else {
            LIBS += $$PWD/qt5.15.2+vs2019/GLC_lib.lib
            CVSrcCopyFile = $$PWD/qt5.15.2+vs2019/release/*

        }
    }
}

CVSrcCopyFile = $$replace(CVSrcCopyFile, /, \\)
CVSrcCopyDir = $$OUT_PWD/bin
CVSrcCopyDir = $$replace(CVSrcCopyDir, /, \\)
QMAKE_PRE_LINK += echo d | xcopy /y $$CVSrcCopyFile $$CVSrcCopyDir
message($$QMAKE_PRE_LINK)
