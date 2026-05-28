/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 69 (Qt 6.10.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../mainwindow.h"
#include <QtGui/qtextcursor.h>
#include <QtGui/qscreen.h>
#include <QtCharts/qlineseries.h>
#include <QtCharts/qabstractbarseries.h>
#include <QtCharts/qvbarmodelmapper.h>
#include <QtCharts/qboxplotseries.h>
#include <QtCharts/qcandlestickseries.h>
#include <QtCore/qabstractitemmodel.h>
#include <QtCharts/qpieseries.h>
#include <QtCore/qabstractitemmodel.h>
#include <QtCharts/qboxplotseries.h>
#include <QtCore/qabstractitemmodel.h>
#include <QtCharts/qpieseries.h>
#include <QtCharts/qpieseries.h>
#include <QtCore/qabstractitemmodel.h>
#include <QtCharts/qxyseries.h>
#include <QtCharts/qxyseries.h>
#include <QtCore/qabstractitemmodel.h>
#include <QtCore/qabstractitemmodel.h>
#include <QtCharts/qboxplotseries.h>
#include <QtCore/qabstractitemmodel.h>
#include <QtCharts/qpieseries.h>
#include <QtCore/qabstractitemmodel.h>
#include <QtCharts/qxyseries.h>
#include <QtCore/qabstractitemmodel.h>
#include <QtCore/qmetatype.h>

#include <QtCore/qtmochelpers.h>

#include <memory>


#include <QtCore/qxptype_traits.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 69
#error "This file was generated using the moc from 6.10.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
QT_WARNING_DISABLE_GCC("-Wuseless-cast")
namespace {
struct qt_meta_tag_ZN10MainWindowE_t {};
} // unnamed namespace

template <> constexpr inline auto MainWindow::qt_create_metaobjectdata<qt_meta_tag_ZN10MainWindowE_t>()
{
    namespace QMC = QtMocConstants;
    QtMocHelpers::StringRefStorage qt_stringData {
        "MainWindow",
        "Every10ms",
        "",
        "on_openPortPushButton_clicked",
        "decodeMPU",
        "OnRxQSerialPort1",
        "on_SendPushButton_clicked",
        "decodeData",
        "uint8_t*",
        "datosRx",
        "on_cleanTxPushButton_clicked",
        "on_cleanRxPushButton_clicked",
        "on_cleanInfoPushButton_clicked",
        "on_ModePushButton_clicked",
        "on_MPU_calibrate_clicked",
        "coordenasMPU",
        "n",
        "X",
        "Y",
        "on_socket_readyRead",
        "on_socket_connected",
        "on_socket_disconnected",
        "on_udp_readyRead",
        "on_connect_pushButton_clicked",
        "on_SET_FRECUENCY_HB_BUTTON_clicked",
        "enviarComando",
        "uint8_t",
        "cmd",
        "payloadData",
        "on_pushButton_clicked",
        "on_GetAlivePushButton_clicked",
        "on_Kp_pushButton_clicked",
        "on_Ki_pushButton_clicked",
        "on_Kd_pushButton_clicked",
        "on_OffPushButton_clicked",
        "on_Screen1PushButton_clicked",
        "on_Screen2PushButton_clicked",
        "on_LDL_pushButton_clicked",
        "on_RDL_pushButton_clicked",
        "sendWifiConfig",
        "ssid",
        "pass",
        "on_REDpushButton_clicked",
        "actualizarRotacionRobot",
        "pitch",
        "yaw",
        "roll",
        "actualizarPitch3D",
        "enviarFloat",
        "valor",
        "enviarInt16",
        "int16_t",
        "enviarYawConfig",
        "kp",
        "kd",
        "curveMultiplier",
        "filterAlpha",
        "steeringStep",
        "steeringLimit",
        "enviarFlConfig",
        "flSetpoint",
        "motionMs",
        "balanceMs",
        "enviarTurnManeuver",
        "targetAngleDeg",
        "wheelMode",
        "wheelSelect",
        "innerWheelPercent",
        "buildUnerV1",
        "flags",
        "unerCrc16Ccitt",
        "uint16_t",
        "data",
        "isCriticalCommand",
        "processUnerV1Datagram",
        "processTcpUnerStream",
        "handleUnerV1Packet",
        "seq",
        "payload",
        "processPendingAckRetry",
        "on_PID_Alpha_pushButton_clicked",
        "on_Home_pushButton_clicked",
        "on_Ejecucion_pushButton_clicked",
        "on_ScreenCalibrar_pushbutton_clicked",
        "on_SistemasDeControl_pushButton_clicked",
        "on_Advanced_pushButton_clicked",
        "on_Setpoint_spinBox_textChanged",
        "arg1",
        "on_StartCalibratePushButton_clicked",
        "on_StopCalibratePushButton_clicked",
        "on_SP_yaw_pushButton_clicked",
        "on_Kd_yaw_pushButton_clicked",
        "on_Kp_yaw_pushButton_clicked",
        "on_ChangeModeIddlePushButton_clicked",
        "on_ChangeModeRCPushButton_clicked",
        "on_ChangeModeFLPushButton_clicked",
        "on_Setpoint_doubleSpinBox_textChanged",
        "on_Screen2PushButton_2_clicked",
        "on_Setpoint_FL_doubleSpinBox_textChanged",
        "on_Setpoint_FL_spinBox_textChanged",
        "on_Ajust_RC_Setpoint_spinBox_textChanged",
        "on_Limite_error_spinBox_textChanged"
    };

    QtMocHelpers::UintData qt_methods {
        // Slot 'Every10ms'
        QtMocHelpers::SlotData<void()>(1, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_openPortPushButton_clicked'
        QtMocHelpers::SlotData<void()>(3, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'decodeMPU'
        QtMocHelpers::SlotData<void()>(4, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'OnRxQSerialPort1'
        QtMocHelpers::SlotData<void()>(5, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_SendPushButton_clicked'
        QtMocHelpers::SlotData<void()>(6, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'decodeData'
        QtMocHelpers::SlotData<void(uint8_t *)>(7, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { 0x80000000 | 8, 9 },
        }}),
        // Slot 'on_cleanTxPushButton_clicked'
        QtMocHelpers::SlotData<void()>(10, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_cleanRxPushButton_clicked'
        QtMocHelpers::SlotData<void()>(11, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_cleanInfoPushButton_clicked'
        QtMocHelpers::SlotData<void()>(12, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_ModePushButton_clicked'
        QtMocHelpers::SlotData<void()>(13, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_MPU_calibrate_clicked'
        QtMocHelpers::SlotData<void()>(14, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'coordenasMPU'
        QtMocHelpers::SlotData<void(int, int, int)>(15, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { QMetaType::Int, 16 }, { QMetaType::Int, 17 }, { QMetaType::Int, 18 },
        }}),
        // Slot 'on_socket_readyRead'
        QtMocHelpers::SlotData<void()>(19, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_socket_connected'
        QtMocHelpers::SlotData<void()>(20, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_socket_disconnected'
        QtMocHelpers::SlotData<void()>(21, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_udp_readyRead'
        QtMocHelpers::SlotData<void()>(22, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_connect_pushButton_clicked'
        QtMocHelpers::SlotData<void()>(23, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_SET_FRECUENCY_HB_BUTTON_clicked'
        QtMocHelpers::SlotData<void()>(24, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'enviarComando'
        QtMocHelpers::SlotData<void(uint8_t, const QByteArray &)>(25, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { 0x80000000 | 26, 27 }, { QMetaType::QByteArray, 28 },
        }}),
        // Slot 'on_pushButton_clicked'
        QtMocHelpers::SlotData<void()>(29, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_GetAlivePushButton_clicked'
        QtMocHelpers::SlotData<void()>(30, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_Kp_pushButton_clicked'
        QtMocHelpers::SlotData<void()>(31, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_Ki_pushButton_clicked'
        QtMocHelpers::SlotData<void()>(32, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_Kd_pushButton_clicked'
        QtMocHelpers::SlotData<void()>(33, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_OffPushButton_clicked'
        QtMocHelpers::SlotData<void()>(34, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_Screen1PushButton_clicked'
        QtMocHelpers::SlotData<void()>(35, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_Screen2PushButton_clicked'
        QtMocHelpers::SlotData<void()>(36, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_LDL_pushButton_clicked'
        QtMocHelpers::SlotData<void()>(37, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_RDL_pushButton_clicked'
        QtMocHelpers::SlotData<void()>(38, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'sendWifiConfig'
        QtMocHelpers::SlotData<void(QString, QString)>(39, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { QMetaType::QString, 40 }, { QMetaType::QString, 41 },
        }}),
        // Slot 'on_REDpushButton_clicked'
        QtMocHelpers::SlotData<void()>(42, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'actualizarRotacionRobot'
        QtMocHelpers::SlotData<void(float, float, float)>(43, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { QMetaType::Float, 44 }, { QMetaType::Float, 45 }, { QMetaType::Float, 46 },
        }}),
        // Slot 'actualizarPitch3D'
        QtMocHelpers::SlotData<void(float, float, float)>(47, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { QMetaType::Float, 44 }, { QMetaType::Float, 45 }, { QMetaType::Float, 46 },
        }}),
        // Slot 'enviarFloat'
        QtMocHelpers::SlotData<void(uint8_t, float)>(48, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { 0x80000000 | 26, 27 }, { QMetaType::Float, 49 },
        }}),
        // Slot 'enviarInt16'
        QtMocHelpers::SlotData<void(uint8_t, int16_t)>(50, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { 0x80000000 | 26, 27 }, { 0x80000000 | 51, 49 },
        }}),
        // Slot 'enviarYawConfig'
        QtMocHelpers::SlotData<void(float, float, float, float, float, float)>(52, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { QMetaType::Float, 53 }, { QMetaType::Float, 54 }, { QMetaType::Float, 55 }, { QMetaType::Float, 56 },
            { QMetaType::Float, 57 }, { QMetaType::Float, 58 },
        }}),
        // Slot 'enviarFlConfig'
        QtMocHelpers::SlotData<void(float, quint16, quint16)>(59, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { QMetaType::Float, 60 }, { QMetaType::UShort, 61 }, { QMetaType::UShort, 62 },
        }}),
        // Slot 'enviarTurnManeuver'
        QtMocHelpers::SlotData<void(float, quint8, quint8, quint8)>(63, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { QMetaType::Float, 64 }, { QMetaType::UChar, 65 }, { QMetaType::UChar, 66 }, { QMetaType::UChar, 67 },
        }}),
        // Slot 'enviarTurnManeuver'
        QtMocHelpers::SlotData<void(float, quint8, quint8)>(63, 2, QMC::AccessPrivate | QMC::MethodCloned, QMetaType::Void, {{
            { QMetaType::Float, 64 }, { QMetaType::UChar, 65 }, { QMetaType::UChar, 66 },
        }}),
        // Slot 'buildUnerV1'
        QtMocHelpers::SlotData<QByteArray(uint8_t, uint8_t, const QByteArray &)>(68, 2, QMC::AccessPrivate, QMetaType::QByteArray, {{
            { 0x80000000 | 26, 27 }, { 0x80000000 | 26, 69 }, { QMetaType::QByteArray, 28 },
        }}),
        // Slot 'unerCrc16Ccitt'
        QtMocHelpers::SlotData<uint16_t(const QByteArray &)>(70, 2, QMC::AccessPrivate, 0x80000000 | 71, {{
            { QMetaType::QByteArray, 72 },
        }}),
        // Slot 'isCriticalCommand'
        QtMocHelpers::SlotData<bool(uint8_t) const>(73, 2, QMC::AccessPrivate, QMetaType::Bool, {{
            { 0x80000000 | 26, 27 },
        }}),
        // Slot 'processUnerV1Datagram'
        QtMocHelpers::SlotData<void(const QByteArray &)>(74, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { QMetaType::QByteArray, 72 },
        }}),
        // Slot 'processTcpUnerStream'
        QtMocHelpers::SlotData<void()>(75, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'handleUnerV1Packet'
        QtMocHelpers::SlotData<void(uint8_t, uint8_t, uint8_t, const QByteArray &)>(76, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { 0x80000000 | 26, 27 }, { 0x80000000 | 26, 69 }, { 0x80000000 | 26, 77 }, { QMetaType::QByteArray, 78 },
        }}),
        // Slot 'processPendingAckRetry'
        QtMocHelpers::SlotData<void()>(79, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_PID_Alpha_pushButton_clicked'
        QtMocHelpers::SlotData<void()>(80, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_Home_pushButton_clicked'
        QtMocHelpers::SlotData<void()>(81, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_Ejecucion_pushButton_clicked'
        QtMocHelpers::SlotData<void()>(82, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_ScreenCalibrar_pushbutton_clicked'
        QtMocHelpers::SlotData<void()>(83, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_SistemasDeControl_pushButton_clicked'
        QtMocHelpers::SlotData<void()>(84, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_Advanced_pushButton_clicked'
        QtMocHelpers::SlotData<void()>(85, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_Setpoint_spinBox_textChanged'
        QtMocHelpers::SlotData<void(const QString &)>(86, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { QMetaType::QString, 87 },
        }}),
        // Slot 'on_StartCalibratePushButton_clicked'
        QtMocHelpers::SlotData<void()>(88, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_StopCalibratePushButton_clicked'
        QtMocHelpers::SlotData<void()>(89, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_SP_yaw_pushButton_clicked'
        QtMocHelpers::SlotData<void()>(90, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_Kd_yaw_pushButton_clicked'
        QtMocHelpers::SlotData<void()>(91, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_Kp_yaw_pushButton_clicked'
        QtMocHelpers::SlotData<void()>(92, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_ChangeModeIddlePushButton_clicked'
        QtMocHelpers::SlotData<void()>(93, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_ChangeModeRCPushButton_clicked'
        QtMocHelpers::SlotData<void()>(94, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_ChangeModeFLPushButton_clicked'
        QtMocHelpers::SlotData<void()>(95, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_Setpoint_doubleSpinBox_textChanged'
        QtMocHelpers::SlotData<void(const QString &)>(96, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { QMetaType::QString, 87 },
        }}),
        // Slot 'on_Screen2PushButton_2_clicked'
        QtMocHelpers::SlotData<void()>(97, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_Setpoint_FL_doubleSpinBox_textChanged'
        QtMocHelpers::SlotData<void(const QString &)>(98, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { QMetaType::QString, 87 },
        }}),
        // Slot 'on_Setpoint_FL_spinBox_textChanged'
        QtMocHelpers::SlotData<void(const QString &)>(99, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { QMetaType::QString, 87 },
        }}),
        // Slot 'on_Ajust_RC_Setpoint_spinBox_textChanged'
        QtMocHelpers::SlotData<void(const QString &)>(100, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { QMetaType::QString, 87 },
        }}),
        // Slot 'on_Limite_error_spinBox_textChanged'
        QtMocHelpers::SlotData<void(const QString &)>(101, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { QMetaType::QString, 87 },
        }}),
    };
    QtMocHelpers::UintData qt_properties {
    };
    QtMocHelpers::UintData qt_enums {
    };
    return QtMocHelpers::metaObjectData<MainWindow, qt_meta_tag_ZN10MainWindowE_t>(QMC::MetaObjectFlag{}, qt_stringData,
            qt_methods, qt_properties, qt_enums);
}
Q_CONSTINIT const QMetaObject MainWindow::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_staticMetaObjectStaticContent<qt_meta_tag_ZN10MainWindowE_t>.stringdata,
    qt_staticMetaObjectStaticContent<qt_meta_tag_ZN10MainWindowE_t>.data,
    qt_static_metacall,
    nullptr,
    qt_staticMetaObjectRelocatingContent<qt_meta_tag_ZN10MainWindowE_t>.metaTypes,
    nullptr
} };

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    auto *_t = static_cast<MainWindow *>(_o);
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: _t->Every10ms(); break;
        case 1: _t->on_openPortPushButton_clicked(); break;
        case 2: _t->decodeMPU(); break;
        case 3: _t->OnRxQSerialPort1(); break;
        case 4: _t->on_SendPushButton_clicked(); break;
        case 5: _t->decodeData((*reinterpret_cast<std::add_pointer_t<uint8_t*>>(_a[1]))); break;
        case 6: _t->on_cleanTxPushButton_clicked(); break;
        case 7: _t->on_cleanRxPushButton_clicked(); break;
        case 8: _t->on_cleanInfoPushButton_clicked(); break;
        case 9: _t->on_ModePushButton_clicked(); break;
        case 10: _t->on_MPU_calibrate_clicked(); break;
        case 11: _t->coordenasMPU((*reinterpret_cast<std::add_pointer_t<int>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<int>>(_a[2])),(*reinterpret_cast<std::add_pointer_t<int>>(_a[3]))); break;
        case 12: _t->on_socket_readyRead(); break;
        case 13: _t->on_socket_connected(); break;
        case 14: _t->on_socket_disconnected(); break;
        case 15: _t->on_udp_readyRead(); break;
        case 16: _t->on_connect_pushButton_clicked(); break;
        case 17: _t->on_SET_FRECUENCY_HB_BUTTON_clicked(); break;
        case 18: _t->enviarComando((*reinterpret_cast<std::add_pointer_t<uint8_t>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<QByteArray>>(_a[2]))); break;
        case 19: _t->on_pushButton_clicked(); break;
        case 20: _t->on_GetAlivePushButton_clicked(); break;
        case 21: _t->on_Kp_pushButton_clicked(); break;
        case 22: _t->on_Ki_pushButton_clicked(); break;
        case 23: _t->on_Kd_pushButton_clicked(); break;
        case 24: _t->on_OffPushButton_clicked(); break;
        case 25: _t->on_Screen1PushButton_clicked(); break;
        case 26: _t->on_Screen2PushButton_clicked(); break;
        case 27: _t->on_LDL_pushButton_clicked(); break;
        case 28: _t->on_RDL_pushButton_clicked(); break;
        case 29: _t->sendWifiConfig((*reinterpret_cast<std::add_pointer_t<QString>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<QString>>(_a[2]))); break;
        case 30: _t->on_REDpushButton_clicked(); break;
        case 31: _t->actualizarRotacionRobot((*reinterpret_cast<std::add_pointer_t<float>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<float>>(_a[2])),(*reinterpret_cast<std::add_pointer_t<float>>(_a[3]))); break;
        case 32: _t->actualizarPitch3D((*reinterpret_cast<std::add_pointer_t<float>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<float>>(_a[2])),(*reinterpret_cast<std::add_pointer_t<float>>(_a[3]))); break;
        case 33: _t->enviarFloat((*reinterpret_cast<std::add_pointer_t<uint8_t>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<float>>(_a[2]))); break;
        case 34: _t->enviarInt16((*reinterpret_cast<std::add_pointer_t<uint8_t>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<int16_t>>(_a[2]))); break;
        case 35: _t->enviarYawConfig((*reinterpret_cast<std::add_pointer_t<float>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<float>>(_a[2])),(*reinterpret_cast<std::add_pointer_t<float>>(_a[3])),(*reinterpret_cast<std::add_pointer_t<float>>(_a[4])),(*reinterpret_cast<std::add_pointer_t<float>>(_a[5])),(*reinterpret_cast<std::add_pointer_t<float>>(_a[6]))); break;
        case 36: _t->enviarFlConfig((*reinterpret_cast<std::add_pointer_t<float>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<quint16>>(_a[2])),(*reinterpret_cast<std::add_pointer_t<quint16>>(_a[3]))); break;
        case 37: _t->enviarTurnManeuver((*reinterpret_cast<std::add_pointer_t<float>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<quint8>>(_a[2])),(*reinterpret_cast<std::add_pointer_t<quint8>>(_a[3])),(*reinterpret_cast<std::add_pointer_t<quint8>>(_a[4]))); break;
        case 38: _t->enviarTurnManeuver((*reinterpret_cast<std::add_pointer_t<float>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<quint8>>(_a[2])),(*reinterpret_cast<std::add_pointer_t<quint8>>(_a[3]))); break;
        case 39: { QByteArray _r = _t->buildUnerV1((*reinterpret_cast<std::add_pointer_t<uint8_t>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<uint8_t>>(_a[2])),(*reinterpret_cast<std::add_pointer_t<QByteArray>>(_a[3])));
            if (_a[0]) *reinterpret_cast<QByteArray*>(_a[0]) = std::move(_r); }  break;
        case 40: { uint16_t _r = _t->unerCrc16Ccitt((*reinterpret_cast<std::add_pointer_t<QByteArray>>(_a[1])));
            if (_a[0]) *reinterpret_cast<uint16_t*>(_a[0]) = std::move(_r); }  break;
        case 41: { bool _r = _t->isCriticalCommand((*reinterpret_cast<std::add_pointer_t<uint8_t>>(_a[1])));
            if (_a[0]) *reinterpret_cast<bool*>(_a[0]) = std::move(_r); }  break;
        case 42: _t->processUnerV1Datagram((*reinterpret_cast<std::add_pointer_t<QByteArray>>(_a[1]))); break;
        case 43: _t->processTcpUnerStream(); break;
        case 44: _t->handleUnerV1Packet((*reinterpret_cast<std::add_pointer_t<uint8_t>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<uint8_t>>(_a[2])),(*reinterpret_cast<std::add_pointer_t<uint8_t>>(_a[3])),(*reinterpret_cast<std::add_pointer_t<QByteArray>>(_a[4]))); break;
        case 45: _t->processPendingAckRetry(); break;
        case 46: _t->on_PID_Alpha_pushButton_clicked(); break;
        case 47: _t->on_Home_pushButton_clicked(); break;
        case 48: _t->on_Ejecucion_pushButton_clicked(); break;
        case 49: _t->on_ScreenCalibrar_pushbutton_clicked(); break;
        case 50: _t->on_SistemasDeControl_pushButton_clicked(); break;
        case 51: _t->on_Advanced_pushButton_clicked(); break;
        case 52: _t->on_Setpoint_spinBox_textChanged((*reinterpret_cast<std::add_pointer_t<QString>>(_a[1]))); break;
        case 53: _t->on_StartCalibratePushButton_clicked(); break;
        case 54: _t->on_StopCalibratePushButton_clicked(); break;
        case 55: _t->on_SP_yaw_pushButton_clicked(); break;
        case 56: _t->on_Kd_yaw_pushButton_clicked(); break;
        case 57: _t->on_Kp_yaw_pushButton_clicked(); break;
        case 58: _t->on_ChangeModeIddlePushButton_clicked(); break;
        case 59: _t->on_ChangeModeRCPushButton_clicked(); break;
        case 60: _t->on_ChangeModeFLPushButton_clicked(); break;
        case 61: _t->on_Setpoint_doubleSpinBox_textChanged((*reinterpret_cast<std::add_pointer_t<QString>>(_a[1]))); break;
        case 62: _t->on_Screen2PushButton_2_clicked(); break;
        case 63: _t->on_Setpoint_FL_doubleSpinBox_textChanged((*reinterpret_cast<std::add_pointer_t<QString>>(_a[1]))); break;
        case 64: _t->on_Setpoint_FL_spinBox_textChanged((*reinterpret_cast<std::add_pointer_t<QString>>(_a[1]))); break;
        case 65: _t->on_Ajust_RC_Setpoint_spinBox_textChanged((*reinterpret_cast<std::add_pointer_t<QString>>(_a[1]))); break;
        case 66: _t->on_Limite_error_spinBox_textChanged((*reinterpret_cast<std::add_pointer_t<QString>>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_staticMetaObjectStaticContent<qt_meta_tag_ZN10MainWindowE_t>.strings))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 67)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 67;
    }
    if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 67)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 67;
    }
    return _id;
}
QT_WARNING_POP
