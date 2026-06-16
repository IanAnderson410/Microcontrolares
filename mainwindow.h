#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include <QTextEdit>
#include <QMessageBox>
#include <QtGlobal>
#include <QInputDialog>
#include <QTimer>
#include <QtMath>
#include <QColor>
#include <QLabel>
#include <QWidget>
#include <QPainter>
#include <QVector>
#include <QPointF>
#include <QTcpSocket>
#include <QTcpServer>
#include <QUdpSocket>
#include <QNetworkDatagram>
#include <QUrl>
#include <QQuickItem>
#include <QHostAddress>
#include <QPushButton>
#include <QLabel>
#include <QVector3D>
#include <QtCharts>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>

#include <math.h>
#include <cmath>
#include <qpaintbox.h>

#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis> // ¡Este te falta para que funcionen los ejes!

/*
    Consideraciones:
        -Mandar Alive cada un segundo

*/
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void Every10ms();

    void on_openPortPushButton_clicked();
    /**
     * @brief decodeMPU decodifica la información del sensor giroscopio y acelerometro MPU6050
     *
     * La funcion toma la variable global QString str y lo desglosa
     *
     * */
    void decodeMPU();

    void OnRxQSerialPort1();
    /**
     * @brief ReciveDataRx
     */
    void on_SendPushButton_clicked();    

    void decodeData(uint8_t *datosRx);
    void on_cleanTxPushButton_clicked();
    void on_cleanRxPushButton_clicked();
    void on_cleanInfoPushButton_clicked();
    /**
     * @brief on_ModePushButton_clicked
     *
     * This funtion choose the state of the car, the default is "WAITING" and if the port
     * is closed for any reason, the car switches to this mode automatically
     */
    void on_ModePushButton_clicked();
    /**
     * @brief DrawRadar
     * DrawRadar draws a radar using a servo motor and an ultrasonic sensor. Every 100ms
     * the servo rotate 2 degrees and the sensor measure the distance. This information
     * is transformed in to a graphic of an marine radar and indicate if there are object
     * close to the car.
     */
    void on_MPU_calibrate_clicked();
    void coordenasMPU(int n, int X, int Y);
    // Slots propios para eventos del Socket
    void on_socket_readyRead(); // Cuando llegan datos
    void on_socket_connected(); // Cuando se logra conectar
    void on_socket_disconnected(); // Si se cae la conexión
    void on_udp_readyRead();
    void on_connect_pushButton_clicked();
    void on_SET_FRECUENCY_HB_BUTTON_clicked();
    bool enviarComando(uint8_t cmd, const QByteArray &payloadData);
    void on_pushButton_clicked();
    void on_GetAlivePushButton_clicked();
    void on_Kp_pushButton_clicked();
    void on_Ki_pushButton_clicked();
    void on_Kd_pushButton_clicked();
    void on_OffPushButton_clicked();
    void on_Screen1PushButton_clicked();
    void on_Screen2PushButton_clicked();
    void on_LDL_pushButton_clicked();
    void on_RDL_pushButton_clicked();
    void sendWifiConfig(QString ssid, QString pass);
    void on_REDpushButton_clicked();




    void actualizarRotacionRobot(float pitch, float yaw, float roll);
    void actualizarPitch3D(float pitch, float yaw, float roll);
    void enviarFloat(uint8_t cmd, float valor);
    void enviarInt16(uint8_t cmd, int16_t valor);
    void enviarYawConfig(float kp, float kd, float curveMultiplier, float filterAlpha, float steeringStep, float steeringLimit, float turnBiasDeg, quint16 preBiasDelayMs);
    void enviarFlConfig(float flSetpoint, quint16 motionMs, quint16 balanceMs);
    void enviarTurnManeuver(float targetAngleDeg, quint8 wheelMode, quint8 wheelSelect, quint8 innerWheelPercent = 0, float turnBiasDeg = 1.0f, quint16 preBiasDelayMs = 300U);
    QByteArray buildUnerV1(uint8_t cmd, uint8_t flags, const QByteArray &payloadData);
    uint16_t unerCrc16Ccitt(const QByteArray &data);
    bool isCriticalCommand(uint8_t cmd) const;
    void processUnerV1Datagram(const QByteArray &data);
    void processTcpUnerStream();
    void handleUnerV1Packet(uint8_t cmd, uint8_t flags, uint8_t seq, const QByteArray &payload);
    void sendAccelRunawayConfig();
    void updateAccelRunawayModeButton(bool enabled);
    QString buildAccelLogCsv() const;
    void updateAccelLogPreview();
    void processPendingAckRetry();
    void on_PID_Alpha_pushButton_clicked();

    void on_Home_pushButton_clicked();

    void on_Ejecucion_pushButton_clicked();

    void on_ScreenCalibrar_pushbutton_clicked();

    void on_SistemasDeControl_pushButton_clicked();

    void on_Advanced_pushButton_clicked();


    void on_Setpoint_spinBox_textChanged(const QString &arg1);

//    void on_MODO_IDDLE_pushButton_clicked();
//    void on_MODO_RC_pushButton_clicked();
 //   void on_MOD_FOLLOWLINE_pushButton_clicked();

    void on_StartCalibratePushButton_clicked();

    void on_StopCalibratePushButton_clicked();

    void on_SP_yaw_pushButton_clicked();

    void on_Kd_yaw_pushButton_clicked();

    void on_Kp_yaw_pushButton_clicked();

    void on_ChangeModeIddlePushButton_clicked();

    void on_ChangeModeRCPushButton_clicked();

    void on_ChangeModeFLPushButton_clicked();

    void on_Setpoint_doubleSpinBox_textChanged(const QString &arg1);

    void on_Screen2PushButton_2_clicked();

    void on_Setpoint_FL_doubleSpinBox_textChanged(const QString &arg1);
    void on_Setpoint_FL_spinBox_textChanged(const QString &arg1);

    void on_Ajust_RC_Setpoint_spinBox_textChanged(const QString &arg1);

    void on_Limite_error_spinBox_textChanged(const QString &arg1);

    void on_accelAdaptiveLimitSpin_textChanged(const QString &arg1);

    void on_accelLogStartButton_clicked();

protected:
    void closeEvent(QCloseEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;
private:
    Ui::MainWindow  *ui;
    QSerialPort     *QSerialPort1;
    QPaintBox       *QPaintBox1;
    QTimer          *QTimer1;

    QTcpSocket      *socket;
    QTcpServer      *tcpServer;
    QTcpSocket      *tcpClient = nullptr;
    QUdpSocket      *udpSocket;
    uint8_t         unerNextSeq = 0;
    uint8_t         unerLastBuiltSeq = 0;
    bool            unerAckWaiting = false;
    uint8_t         unerAckCmd = 0;
    uint8_t         unerAckSeq = 0;
    uint8_t         unerAckRetries = 0;
    uint16_t        unerAckWaitTicks = 0;
    QByteArray      unerAckFrame;
    QHostAddress    udpRemoteAddress;
    quint16         udpRemotePort = 8888;
    quint16         udpLocalPort = 8888;
    bool            udpReady = false;
    bool            telemetryUdpEnabled = false;
    bool            turnManeuverTelemetryActive = false;
    bool            accelLogReceiving = false;
    quint8          accelLogCaptureId = 0;
    quint16         accelLogExpectedSamples = 0;

    QByteArray      m_buffer;


    uint32_t        counterRotation=0;
    uint32_t        counter1=0;
    QLineSeries *seriesA;
    QLineSeries *seriesB;
    QLineSeries *seriesC;
    QLineSeries *seriesD;
    int tiempoX = 0;
    int tiempoX2 = 0;
    void actualizarGrafico();
    void actualizarGraficoPID();
/*
#pragma pack(push, 1)
    typedef struct {
        // --- IMU & Movimiento (24 bytes) ---
        int16_t acc_x, acc_y, acc_z;    // [0-5]
        int16_t gyro_pitch, gyro_yaw;   // [6-9]
        float pitch_angle;              // [10-13]
        float pos_x;                    // [14-17]
        float pos_y;                    // [18-21]
        float velocidad;                // [22-25]
        uint16_t sensores_ir[8];        // [26-41] Nuevos datos IR
        uint8_t modo;                   // [42] (IDLE, FOLLOW, ETC)
        uint8_t estado_sistema;         // [43] (0=Inicio, 1=Calibrado, 2=Batería Baja, etc.)
    } PayloadData_t;
#pragma pack(pop)
*/

#pragma pack(push, 1)
    typedef struct __attribute__((packed)) {
        int16_t     acc_x, acc_y, acc_z;    // 6 bytes - Datos crudos
        int16_t     gyro_pitch, gyro_yaw;   // 4 bytes - Pitch (Y) y Yaw (Z)

        float       pitch_filtrado;         // 4 bytes - Pitch filtrado
        float       yaw_filtrado;           // 4 bytes - Yaw filtrado

        float       pos_x;                  // 4 bytes - Trayectoria X
        float       pos_y;                  // 4 bytes - Trayectoria Y
        float       velocidad;              // 4 bytes - Velocidad lineal
        uint8_t     modo;                   // 1 byte  - IDLE, FOLLOW_LINE, etc.
        uint16_t    IR[8];                  // 16 bytes- Sensores IR
        uint8_t     infoAdicional;          // 1 byte  - Info Adicional
    } PayloadData_t;
#pragma pack(pop)

#pragma pack(push, 1)
    typedef struct {
        int16_t     acc_x, acc_y, acc_z;
        int16_t     pitch_cdeg;
    } PayloadDataMinV1_t;
#pragma pack(pop)

#pragma pack(push, 1)
    typedef struct {
        int16_t     acc_x, acc_y, acc_z;
        int16_t     gyro_pitch, gyro_yaw;
        int16_t     pitch_cdeg;
        int16_t     roll_cdeg;
        int16_t     yaw_cdeg;
    } PayloadDataMpuV1_t;
#pragma pack(pop)

#pragma pack(push, 1)
    typedef struct {
        int16_t     acc_x, acc_y, acc_z;
        int16_t     gyro_pitch, gyro_yaw;
        int16_t     pitch_cdeg;
        int16_t     roll_cdeg;
        int16_t     yaw_cdeg;
        int32_t     pos_x_mm;
        int32_t     pos_y_mm;
        int16_t     velocidad_mm_s;
        uint8_t     modo;
    } PayloadDataStateV1_t;
#pragma pack(pop)

#pragma pack(push, 1)
    typedef struct {
        int16_t     acc_x, acc_y, acc_z;
        int16_t     gyro_pitch, gyro_yaw;
        int16_t     pitch_cdeg;
        int16_t     roll_cdeg;
        int16_t     yaw_cdeg;
        int32_t     pos_x_mm;
        int32_t     pos_y_mm;
        int16_t     velocidad_mm_s;
        uint8_t     modo;
        uint16_t    IR[8];
        uint8_t     infoAdicional;
        uint8_t     accelFlags;
    } PayloadDataV1_t;
#pragma pack(pop)

    typedef struct {
        quint16 sampleIndex;
        qint16 accelXRaw;
        qint16 pitchCdeg;
    } AccelLogRow;
    QVector<AccelLogRow> accelLogRows;

    float accel_angle_filtrado = 0.0f;
    float accelLineal = 0.0f;

    //payload: 26 + 16 + 2 = 44 Bytes.
    //Tamaño del Paquete: Header (4) + Len (1) + Token (1) + CMD (1) + Payload (44) + Checksum (1) = 52 Bytes.
    typedef union{
        int32_t i32;
        int16_t i16[2];
        int8_t  i8[4];
    }_uDatos;
    typedef struct{
        QString str;
        QString Ax;
        QString Ay;
        QString Az;
        QString Gx;
        QString Gy;
        QString Gz;
    }_sMPU6050;
    float currentDegrees;           /*!< grados actuales del MPU, va de 0 a 360*/
  //  uint16_t currentDegreesRaw;     /*!< Valores en crudo sin unidad bien definida*/
    _sMPU6050   MPU;
    _uDatos     dato;
    uint8_t     buffTx[256];
    uint8_t     buffRx[256];

    uint8_t indexTx;
    uint8_t indexRx;

    uint8_t bytesRx;
    bool    readMPU=0;

    float dt = 0.01; // tiempo entre muestras, en segundos

    uint32_t every100ms;

    //====My Flags====
    bool RADAR=false;       // while this flag is true all modes are stoped and the car star the radar mode
    bool WHEELSAREMOVE=false;  // while this flag is true the user can´t use the wheels
    //==end of My Flags==
    typedef enum{
        WAITING =0,
        MODE_1, //Seguidor de linea
        MODE_2, //Esquivar obstaculo
        MODE_3, //Mantener distancia
    }_eMode;
    uint8_t state = WAITING;

    typedef enum{
        header_U,
        header_N,
        header_E,
        header_R,
        nBytes,
        token,
        payload
    }_ePro;

    _ePro proState;

    /*Variables para los comandos*/

    typedef union {
    double  d32;
    float f32;
    int i32;
    unsigned int ui32;
    unsigned short ui16[2];
    short i16[2];
    uint8_t ui8[4];
    char chr[4];
    unsigned char uchr[4];
    int8_t  i8[4];
    }_udat;
    _udat myWord;

    /**
     * @brief _sValue
     *
     *Almacena los datos de las diferentes piezas de hardware
     * */
    typedef struct{
        int16_t servo;
        int16_t distance;
        int16_t m1_vel;
        int16_t m2_vel;
    }_sValue;
    _sValue currentValue;
    int16_t angulo= 92;
    uint16_t cmdID;

    typedef struct{
    uint8_t timeOut;
    uint8_t cheksum;
    uint8_t payLoad[256];
    uint8_t nBytes;
    uint8_t index;
    }_sDatos ;

    _sDatos rxData, rxDataUdp;

    int contadorAlive=0;

    typedef struct{
        uint8_t thisIsWhite;
        uint8_t setThisBlack;
    }_sSetColor;
    _sSetColor currentColors;

    bool isGettingVelocity = false;


#define UNER_HEADER_STR "UNER"
#define UNER_TOKEN      0xAA
#define UNER_V1_VERSION 1
#define UNER_V1_FLAG_ACK_REQUIRED 0x01
#define UNER_V1_FLAG_ACK 0x02
    enum {
        MODO_IDDLE 			= 	0,
        MODO_RC				=	1,
        MODO_FL_INICIO      =   2,
        MODO_FL_BUSQUEDA_INICIAL = 3,
        MODO_FL_SIGUIENDO   =   4,
        MODO_FOLLOWLINE 	= 	MODO_FL_BUSQUEDA_INICIAL
    };
    enum {
        // Sistema y Heartbeat
        CMD_ALIVE       			= 0, 		/*!< Busca confirmar conexión inalambrica						*/
        CMD_ACK 	      			= 1, 		/*!< Confirma conexión inalambrica								*/
        CMD_SET_HB      			= 2, 		/*!< Configurar intervalo de Heartbeat							*/
        CMD_CHANGE_MODE				= 3,		/*!< Cambiar entre los modos  IDLE, FOLLOW_LINE, RC				*/
        CMD_CALIBRATE   			= 5, 		/*!< Calibración de MPU6050										*/
        CMD_START       			= 6, 		/*!< Activar motores / Inicio de balanceo						*/
        CMD_STOP        			= 7, 		/*!< Parada de emergencia / Motores a 0							*/
        CMD_TCP_CONNECTED			= 8,		/*!< Comando utilizado en la conexión del TCP y el robot		*/
        CMD_CHANGE_OLED_SCREEN  	= 9,		/*!< Cambia o apaga la pantalla OLED							*/
        //RADIO CONTROL
        CMD_RC    					= 10, 		/*!< Movimiento manual (adelante, atrás, giros)					*/
        //MOTORES GENERAL
        CMD_CHANGE_DEADLINE_LEFT 	= 12,		/*!< Ajustar  Deadband del motor izquierdo						*/
        CMD_CHANGE_DEADLINE_RIGHT 	= 13,		/*!< Ajustar  Deadband del motor derecho						*/
        CMD_CHANGE_SETPOINT 		= 14, 		/*!< Ajustar  Setpoint de los motores*/
        CMD_DEFINE_ZERO_SETPOINT	= 15, 		/*!< Ajustar  Setpoint de los motores*/
        CMD_ONOFFMOTORS 			= 16, 		/*!< Prender y apagar motores*/
        CMD_DATA 					= 17,		/*!< El robot manda datos de la unidad Sensitiva*/
        //PID SISTEMA BALANCEO
        CMD_PID_PITCH_KP      		= 20, 		/*!< Ajustar Término Proporcional del PID basado en grado de libertad Pitch*/
        CMD_PID_PITCH_KI      		= 21, 		/*!< Ajustar Término Integral del PID basado en grado de libertad Pitch*/
        CMD_PID_PITCH_KD      		= 22, 		/*!< Ajustar Término Derivativa del PID basado en grado de libertad Pitch*/
        CMD_PID_ALPHA 				= 23, 		/*!< Ajustar  Alpha del del filtro complementario utilizado en el PID basado en grado de libertad Pitch*/


        CMD_PID_PITCH_LIM_INCLI		= 24, 		/*!< Ajustar Término Proporcional del PID basado en grado de libertad Pitch*/
        CMD_PID_PITCH_CORECCION_RCSP= 25, 		/*!< Ajustar Término Integral del PID basado en grado de libertad Pitch*/

        CMD_RC_MOVE_5_CM            = 30, 		/*!< Ajustar Término Integral del PID basado en grado de libertad Pitch*/

        //PID SISTEMA DE GIRO
        CMD_PID_YAW_KP      		= 40, 		/*!< Ajustar Término Proporcional del PID basado en grado de libertad Yaw*/
        CMD_PID_YAW_KD      		= 41, 		/*!< Ajustar Término Integral del PID basado en grado de libertad Yaw*/
        CMD_PID_YAW_SP      		= 42, 		/*!< Ajustar Término Derivativa del PID basado en grado de libertad Yaw*/

        CMD_PID_YAW_MULTIPLICADOR  	= 43, 		/*!< Ajustar Término Proporcional del PID basado en grado de libertad Yaw*/
        //SENSORES
        CMD_IR_INICIAR_CALIBRACION	= 44, 		/*!< Ajustar Término Proporcional del PID basado en grado de libertad Yaw*/
        CMD_IR_DETENER_CALIBRACION  = 45, 		/*!< Ajustar Término Integral del PID basado en grado de libertad Yaw*/
        //NETWORK
        CMD_NETWORK_CHANGE_SSID		= 50,
        CMD_NETWORK_CHANGE_PASSWORD	= 51,
        CMD_TELEMETRY_START         = 60,
        CMD_TELEMETRY_STOP          = 61,
        CMD_HTTP_SOFTAP             = 62,
        CMD_SET_YAW_PD              = 63,
        CMD_SET_YAW_CONFIG          = 64,
        CMD_SET_FL_CONFIG           = 65,
        CMD_TURN_MANEUVER           = 66,
        CMD_OBSTACLE_FOLLOW         = 67,
        CMD_ACCEL_LOG_START         = 69,
        CMD_ACCEL_LOG_CHUNK         = 70,
        CMD_ACCEL_LOG_DONE          = 71,
        CMD_ACCEL_RUNAWAY_CONFIG    = 72
        // CMD_TELEMETRY   			= 0xA0, 	/*!< Envío de ángulos, velocidad y sensores IR	*/
        // CMD_LOG_MSG     			= 0xA1,  	/*!< Envío de mensajes de texto para debug		*/
    };

// Estructura única de recepción/envío
// Usamos "packed" para asegurar que no haya espacios vacíos en memoria
#pragma pack(push, 1)
    typedef struct {
        uint8_t header[4];      // "UNER"
        uint8_t length;         // CMD + N_Payload + Checksum
        uint8_t token;          // ':'
        uint8_t cmd;            // ID del comando
        uint8_t payload[64];    // Buffer genérico (sobra espacio)
        // El checksum no lo ponemos en el struct fijo porque su posición varía
    } UnerPacket_t;
#pragma pack(pop)

    int8_t last_sp_sent = 0;
    int16_t last_st_sent = 0;
    uint8_t last_act_sent = 0;

    bool keyUp = false;
    bool keyDown = false;
    bool keyLeft = false;
    bool keyRight = false;

};
#endif // MAINWINDOW_H
