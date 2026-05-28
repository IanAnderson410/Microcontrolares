/*/**
  ******************************************************************************
  * @file           : mainwindow.cpp
  * @brief          :   Archivo principal de la ventana grÃ¡fica para el proyecto del 11 de junio de 2025 de
  *                     Microcontroladores en Ing. MecatrÃ³nica de fcal UNER en Concordia, Entre RÃ­os, Argentina.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *******************[Modificaciones]*****************************
  *
  *	2/6 	Se modifico la recepciÃ³n y transmisiÃ³n de datos
  * 3/6     SE MODIFICO POR COMPLETO el tipo de dato recibido y enviado. Ver en UNION
  *         Se encaro agregar un calibrador para el MPU con un offset dinÃ¡mico
  *
  *
  ******************************************************************************
  */
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPushButton>
#include <QLineEdit>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QGridLayout>
#include <QtEndian>
#include <cstring>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    //===================[ TCP SOCKET ]===================//
    socket = new QTcpSocket(this);
    connect(socket, &QTcpSocket::readyRead,this, &MainWindow::on_socket_readyRead);
    connect(socket, &QTcpSocket::connected,this, &MainWindow::on_socket_connected);
    connect(socket, &QTcpSocket::disconnected,this, &MainWindow::on_socket_disconnected);
    tcpServer = new QTcpServer(this);
    connect(tcpServer, &QTcpServer::newConnection, this, [this]() {
        if (tcpClient != nullptr) {
            tcpClient->disconnectFromHost();
            tcpClient->deleteLater();
            tcpClient = nullptr;
        }
        tcpClient = tcpServer->nextPendingConnection();
        QTcpSocket *client = tcpClient;
        ui->InfoTextEdit->append("TCP cliente conectado: " + client->peerAddress().toString() +
                                 ":" + QString::number(client->peerPort()));
        ui->TPCCONECTED->setStyleSheet("background-color: rgb(0, 255, 0); border-radius: 10px; border: 1px solid black;");
        connect(client, &QTcpSocket::readyRead, this, [this, client]() {
            if (tcpClient != client) {      return;}
            m_buffer.append(client->readAll());
            processTcpUnerStream();
        });
        connect(client, &QTcpSocket::disconnected, this, [this, client]() {
            ui->InfoTextEdit->append("TCP cliente desconectado.");
            ui->TPCCONECTED->setStyleSheet("background-color: rgb(255, 255, 0); border-radius: 10px; border: 1px solid black;");
            if (tcpClient == client) {
                tcpClient = nullptr;
            }
            client->deleteLater();
        });
    });
    udpSocket = new QUdpSocket(this);
    connect(udpSocket, &QUdpSocket::readyRead, this, &MainWindow::on_udp_readyRead);
    QSerialPort1 = new QSerialPort(this);
//    QPaintBox1  = new QPaintBox( 0, 0, ui->widget); //Create adentro del widget
  //  QPaintBox1  = new QPaintBox( 0, 0, ui->widget); //Create adentro del widget

    //===================[ Timer ]===================//
    QTimer1     = new QTimer(this);
    connect(QSerialPort1, &QSerialPort:: readyRead, this, &MainWindow::OnRxQSerialPort1);
    connect(QTimer1, &QTimer::timeout, this, &MainWindow::Every10ms);
    QTimer1->start(50);
    currentDegrees=0;

    //===================[ Modelos 3D ]===================//
    ui->quickWidget->setSource(QUrl::fromLocalFile("C:/Users/ianan/STM32CubeIDE/workspace_2.0.0/N20/qcomunication/3dmodels/main.qml"));
    ui->quickWidget->setResizeMode(QQuickWidget::SizeRootObjectToView);

    ui->PitchQuickWidget->setSource(QUrl::fromLocalFile("C:/Users/ianan/STM32CubeIDE/workspace_2.0.0/N20/qcomunication/3dmodels/pitchmodel.qml"));
    ui->PitchQuickWidget->setResizeMode(QQuickWidget::SizeRootObjectToView);
// este qquickwidget me crashea la interfaZ
//ui->VistaSuperiorQuickWidget->setSource(QUrl::fromLocalFile("C:/Users/ianan/STM32CubeIDE/workspace_2.0.0/N20/qcomunication/3dmodels/vistaSuperior.qml"));
// ui->VistaSuperiorQuickWidget->setResizeMode(QQuickWidget::SizeRootObjectToView);

    //===================[ Graficos ]===================//
    // Si quisieramos asociar un slider a un modelo 3d podemos hacerlo asÃ­:
    //  connect(ui->verticalSlider, &QSlider::valueChanged, this, [=](int val){   actualizarRotacionRobot((float)val, 45.0f, 0.0f);   });
    // connect(ui->verticalSlider, &QSlider::valueChanged, this, [=](int val){   actualizarRotacionRobot((float)val, 45.0f, 0.0f);   });
    seriesA = new QLineSeries();
    seriesB = new QLineSeries();
    seriesC = new QLineSeries();
    seriesD = new QLineSeries();

    seriesA->setName("P");
    seriesB->setName("I");
    seriesC->setName("D");
    seriesD->setName("OUT");

    QChart *chart = new QChart();

    chart->addSeries(seriesA);
    chart->addSeries(seriesB);
    chart->addSeries(seriesC);
    chart->addSeries(seriesD);
    chart->setTitle("Monitoreo en Tiempo Real PID");
    QValueAxis *ejeX = new QValueAxis();
    ejeX->setRange(0, 100);                  // Ventana inicial de 100 muestras
    ejeX->setTitleText("Tiempo (muestras)"); // Etiqueta del eje
    ejeX->setLabelFormat("%d");              // Formato entero
    ejeX->setTickCount(11);                  // Cantidad de lÃ­neas divisorias verticales
    QValueAxis *ejeY = new QValueAxis();
    ejeY->setRange(-3000, 3000);                 // Rango para el Pitch (ej. -45Â° a +45Â°)
    ejeY->setTitleText("Ãngulo Pitch (Â°)");
    ejeY->setTickCount(7);                   // LÃ­neas divisorias horizontales
    chart->addAxis(ejeX, Qt::AlignBottom);
    chart->addAxis(ejeY, Qt::AlignLeft);
    seriesA->attachAxis(ejeX);
    seriesA->attachAxis(ejeY);
    seriesB->attachAxis(ejeX);
    seriesB->attachAxis(ejeY);

    seriesC->attachAxis(ejeX);
    seriesC->attachAxis(ejeY);
    seriesD->attachAxis(ejeX);
    seriesD->attachAxis(ejeY);

    ui->graphicsView->setChart(chart);
    ui->graphicsView->setRenderHint(QPainter::Antialiasing);

    ui->IR1_progressBar->setRange(0, 4095);
    ui->IR2_progressBar->setRange(0, 4095);
    ui->IR3_progressBar->setRange(0, 4095);
    ui->IR4_progressBar->setRange(0, 4095);
    ui->IR5_progressBar->setRange(0, 4095);
    ui->IR6_progressBar->setRange(0, 4095);
    ui->IR7_progressBar->setRange(0, 4095);
    ui->IR8_progressBar->setRange(0, 4095);

    ui->IR1_progressBar->setValue(0);
    ui->IR2_progressBar->setValue(0);
    ui->IR3_progressBar->setValue(0);
    ui->IR4_progressBar->setValue(0);
    ui->IR5_progressBar->setValue(0);
    ui->IR6_progressBar->setValue(0);
    ui->IR7_progressBar->setValue(0);
    ui->IR8_progressBar->setValue(0);

    ui->InfoTextEdit->setFocusPolicy(Qt::NoFocus); // agregar al ui directamente y borrar esta linea

    ui->aliveUdpButton->setToolTip("Envia ALIVE\\r\\n por UDP al ESP01");
    connect(ui->aliveUdpButton, &QPushButton::clicked, this, &MainWindow::on_GetAlivePushButton_clicked);

    ui->telemetryUdpButton->setToolTip("Inicia o detiene la telemetria UDP");
    connect(ui->telemetryUdpButton, &QPushButton::clicked, this, [this]() {
        if (!udpReady) {
            ui->InfoTextEdit->append("Error: UDP no esta listo.");
            return;
        }

        QByteArray datos;

        if (!telemetryUdpEnabled) {
            enviarComando(CMD_TELEMETRY_START, datos);
            telemetryUdpEnabled = true;
            ui->telemetryUdpButton->setText("STOP TEL");
            ui->TxTextEdit->append("UDP TX: CMD_TELEMETRY_START v1");
        } else {
            enviarComando(CMD_TELEMETRY_STOP, datos);
            telemetryUdpEnabled = false;
            ui->telemetryUdpButton->setText("START TEL");
            ui->TxTextEdit->append("UDP TX: CMD_TELEMETRY_STOP v1");
        }
    });

    ui->httpSoftApButton->setToolTip("Cambia el ESP01 a SoftAP HTTP para configurar HB desde el celular");
    connect(ui->httpSoftApButton, &QPushButton::clicked, this, [this]() {
        QByteArray datos;
        enviarComando(CMD_HTTP_SOFTAP, datos);
        ui->TxTextEdit->append("TX: CMD_HTTP_SOFTAP v1");
        ui->InfoTextEdit->append("El robot pasara a HTTP SoftAP. Conectate a N20_ROBOT y abri http://192.168.4.1");
    });

    connect(ui->obstacleOledButton, &QPushButton::clicked, this, [this]() {
        enviarInt16(CMD_CHANGE_OLED_SCREEN, 4);
        ui->TxTextEdit->append("PC: CMD_CHANGE_OLED_SCREEN OBS");
    });

    ui->yawKpEdit->setToolTip("Kp del PD de Yaw");
    ui->yawKdEdit->setToolTip("Kd del PD de Yaw");
    ui->yawPdButton->setToolTip("Configura Kp y Kd de YAW con los campos KpY y KdY");
    connect(ui->yawPdButton, &QPushButton::clicked, this, [this]() {
        QByteArray payload;
        float kp = ui->yawKpEdit->text().toFloat();
        float kd = ui->yawKdEdit->text().toFloat();
        quint32 kpRaw;
        quint32 kdRaw;
        memcpy(&kpRaw, &kp, sizeof(kpRaw));
        memcpy(&kdRaw, &kd, sizeof(kdRaw));
        kpRaw = qToLittleEndian<quint32>(kpRaw);
        kdRaw = qToLittleEndian<quint32>(kdRaw);
        payload.append(reinterpret_cast<const char*>(&kpRaw), sizeof(kpRaw));
        payload.append(reinterpret_cast<const char*>(&kdRaw), sizeof(kdRaw));
        enviarComando(CMD_SET_YAW_PD, payload);
        ui->TxTextEdit->append("TX: SET YAW PD Kp=" + QString::number(kp) +
                               " Kd=" + QString::number(kd));
    });

    auto configureSpin = [](QDoubleSpinBox *spin, double value, double min, double max, double step, int decimals) {
        spin->setRange(min, max);
        spin->setSingleStep(step);
        spin->setDecimals(decimals);
        spin->setValue(value);
        spin->setKeyboardTracking(false);
    };

    configureSpin(ui->yawCfgKp, 100.0, 0.0, 20000.0, 100.0, 1);
    configureSpin(ui->yawCfgKd, 0.0, 0.0, 5000.0, 10.0, 1);
    configureSpin(ui->yawCfgSp, 1.5, -10.0, 10.0, 0.05, 2);
    configureSpin(ui->yawCfgMul, 0.010, 0.0, 1.000, 0.001, 4);
    configureSpin(ui->yawCfgAlpha, 0.70, 0.0, 0.995, 0.01, 3);
    configureSpin(ui->yawCfgStep, 90.0, 1.0, 2000.0, 10.0, 0);
    configureSpin(ui->yawCfgLimit, 500.0, 0.0, 4000.0, 25.0, 0);
    configureSpin(ui->flCfgMotionMs, 200.0, 20.0, 2000.0, 10.0, 0);
    configureSpin(ui->flCfgBalanceMs, 400.0, 20.0, 5000.0, 10.0, 0);
    configureSpin(ui->turnAngleSpin, 90.0, -360.0, 360.0, 5.0, 1);
    configureSpin(ui->turnArcPercentSpin, 50.0, 0.0, 100.0, 5.0, 0);
    ui->turnArcPercentSpin->setSuffix("%");
    ui->turnArcPercentSpin->setEnabled(false);

    ui->turnModeCombo->clear();
    ui->turnModeCombo->addItem("2 wheels", 0);
    ui->turnModeCombo->addItem("1 wheel", 1);
    ui->turnModeCombo->addItem("Arc", 2);
    ui->turnWheelCombo->clear();
    ui->turnWheelCombo->addItem("Left", 0);
    ui->turnWheelCombo->addItem("Right", 1);
    ui->turnWheelCombo->setEnabled(false);

    connect(ui->yawCfgSend, &QPushButton::clicked, this, [this]() {
        enviarYawConfig(static_cast<float>(ui->yawCfgKp->value()),
                        static_cast<float>(ui->yawCfgKd->value()),
                        static_cast<float>(ui->yawCfgMul->value()),
                        static_cast<float>(ui->yawCfgAlpha->value()),
                        static_cast<float>(ui->yawCfgStep->value()),
                        static_cast<float>(ui->yawCfgLimit->value()));
    });

    connect(ui->flCfgSend, &QPushButton::clicked, this, [this]() {
        enviarFlConfig(static_cast<float>(ui->yawCfgSp->value()),
                       static_cast<quint16>(ui->flCfgMotionMs->value()),
                       static_cast<quint16>(ui->flCfgBalanceMs->value()));
    });

    connect(ui->yawCfgOled, &QPushButton::clicked, this, [this]() {
        enviarInt16(CMD_CHANGE_OLED_SCREEN, 1);
    });

    connect(ui->turnModeCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int) {
        const uint mode = ui->turnModeCombo->currentData().toUInt();
        ui->turnWheelCombo->setEnabled(mode == 1U || mode == 2U);
        ui->turnArcPercentSpin->setEnabled(mode == 2U);
    });

    connect(ui->turnStartButton, &QPushButton::clicked, this, [this]() {
        float targetAngleDeg = static_cast<float>(ui->turnAngleSpin->value());
        if (qAbs(targetAngleDeg) < 1.0f) {
            ui->InfoTextEdit->append("TURN: angulo minimo 1 grado.");
            return;
        }

        enviarTurnManeuver(targetAngleDeg,
                           static_cast<quint8>(ui->turnModeCombo->currentData().toUInt()),
                           static_cast<quint8>(ui->turnWheelCombo->currentData().toUInt()),
                           static_cast<quint8>(ui->turnArcPercentSpin->value()));
    });
}
MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::Every10ms(){
    processPendingAckRetry();

    counterRotation+=1;
   // actualizarRotacionRobot(-90.0f, counterRotation, 45.0f);
    actualizarRotacionRobot(-90.0f, counterRotation, 0.0f);
    actualizarGrafico();
    actualizarGraficoPID();
    //float ax
    if(QSerialPort1->isOpen()){

    }
    /*
     * UDP no requiere reconexiones periÃƒÂ³dicas: el botÃƒÂ³n Connect fija el destino
     * y abre el puerto local una sola vez.
     */
    /*else if(ui->connect_pushButton->text() == "Disconnect To STM32"){
        QByteArray datos;
        datos.append((char)0); // Agregamos el dato al payload
        enviarComando(CMD_TCP_CONNECTED, datos);
        socket->disconnectFromHost(); // Si ya estaba conectado, desconecta

    }*/

    counter1 ++;
    if(counter1 >= 3){
        counter1 =0;
        int8_t setpoint_send = 0;
        int16_t steering_send = 0;
        uint8_t active = 0;

        // LÃ³gica de flechas
        if (keyUp)    setpoint_send = 125;
        else if (keyDown)  setpoint_send = -125;
        if (keyLeft)  steering_send = -450;
        else if (keyRight) steering_send = 450;
        if (keyUp || keyDown || keyLeft || keyRight) active = 1;
        if (active || setpoint_send != last_sp_sent||steering_send != last_st_sent ||active != last_act_sent){
            QByteArray payload;
            payload.append((char)active);
            payload.append((char)setpoint_send);
            qint16 steeringLittleEndian = qToLittleEndian<qint16>(steering_send);
            payload.append(reinterpret_cast<const char*>(&steeringLittleEndian), sizeof(steeringLittleEndian));
            enviarComando(CMD_RC, payload);
            last_sp_sent = setpoint_send;
            last_st_sent = steering_send;
            last_act_sent = active;
            ui->textEdit->append("TX RC -> Cambio detectado");
        }

    }
}
void MainWindow::keyPressEvent(QKeyEvent *event) {
    if(event->isAutoRepeat()) return; // Ignoramos la repeticiÃ³n automÃ¡tica
    switch (event->key()) {
    case Qt::Key_Up:    keyUp = true;    break;
    case Qt::Key_Down:  keyDown = true;  break;
    case Qt::Key_Left:  keyLeft = true;  break;
    case Qt::Key_Right: keyRight = true; break;
    }
}
void MainWindow::keyReleaseEvent(QKeyEvent *event) {
    if(event->isAutoRepeat()) return;
    switch (event->key()) {
    case Qt::Key_Up:    keyUp = false;    break;
    case Qt::Key_Down:  keyDown = false;  break;
    case Qt::Key_Left:  keyLeft = false;  break;
    case Qt::Key_Right: keyRight = false; break;
    }
}
void MainWindow::closeEvent(QCloseEvent *event) {
        QByteArray datos;
        datos.append((char)0); // Agregamos el dato al payload
        enviarComando(CMD_TCP_CONNECTED, datos);
        socket->disconnectFromHost();
        event->accept();
}

void MainWindow::on_openPortPushButton_clicked(){
    if( QSerialPort1->isOpen()){
        ui->openPortPushButton->setText("OPEN");
        QSerialPort1->close();
        state=WAITING;
    }else{
        QSerialPort1->setPortName(ui->OpenPortcomboBox->currentText());
        QSerialPort1->setBaudRate(QSerialPort::Baud115200);
        if(QSerialPort1->open(QSerialPort::ReadWrite)){
            ui->openPortPushButton->setText("CLOSE");
        }
    }
}
void MainWindow::decodeMPU(){}   //end function
void MainWindow::OnRxQSerialPort1(){
    unsigned char *buf;                         // Buffer dinamico
    int count;
    count = QSerialPort1->bytesAvailable();     // Esta linea pregunta cuantos datos llegaron para
    bytesRx = QSerialPort1->bytesAvailable();
        // leer en el buffer
    if(count <= 0)                              // Si es menor o igual a cero algo esta mal
        return;
    buf = new unsigned char[count];    //Reservamos memoria una cantidad count bytes
    QSerialPort1->read((char*)buf, count);             // Leemos el buffer una cantidad count de bytes
 //   QSerialPort1->read((char*)buffRx, count);             // Leemos el buffer una cantidad count de bytes
    MPU.str = "";
    delete [] buf;
}
void MainWindow::on_SendPushButton_clicked(){
    if(QSerialPort1->isOpen()){
    }
}
void MainWindow::decodeData(uint8_t *datosRx){
    if(QSerialPort1->isOpen()){
        int32_t length = sizeof(*datosRx)/sizeof(datosRx[0]);
        QString str, strOut;
        for(int i = 1; i<length; i++){
            if(isalnum(datosRx[i]))
                str = str + QString("%1").arg(char(datosRx[i]));
            else
                str = str +QString("%1").arg(datosRx[i],2,16,QChar('0'));
        }
        ui->InfoTextEdit->append("(MBED->PC)->decodeData (" + str + ")");
        switch (buffRx[0]) {
        default:
            str = str + "UNKNOWN COMMAND ";
            ui->RxTextEdit->append(str);
        }
        cmdID = '\0';
    }
}
void MainWindow::on_cleanTxPushButton_clicked(){        ui->TxTextEdit->clear();    }
void MainWindow::on_cleanRxPushButton_clicked(){        ui->RxTextEdit->clear();    }
void MainWindow::on_cleanInfoPushButton_clicked(){      ui->InfoTextEdit->clear();  }
void MainWindow::on_ModePushButton_clicked(){
    char mode[30];
    if(QSerialPort1->isOpen()){
        state ++;
        QPainter paint(QPaintBox1->getCanvas());
        QPen pen;
        QPoint centerCircle;
        if(state > 4 ) state = WAITING;

        switch(state){
        case WAITING:
            strcpy( mode, "WAITING");    
            break;
        case MODE_1:
            strcpy( mode, "Seguidor de Linea");
            break;
        case MODE_2:
            strcpy( mode, "Esquivar ObstÃ¡culo");
                break;
        case MODE_3:
            strcpy( mode, "Mantener Distancia");
            break;
        }
    }else{
        strcpy( mode, "WAITING");
        state=WAITING;
    }
}
void MainWindow::on_MPU_calibrate_clicked()
{
    if(udpReady || socket->state() == QAbstractSocket::ConnectedState){
        enviarComando(CMD_CALIBRATE, QByteArray());
        ui->InfoTextEdit->append("PC: CMD5, CALIBRAR MPU ");
    } else {
        ui->InfoTextEdit->append("Error: No estÃ¡s conectado.");
    }
}
void MainWindow::coordenasMPU(int n, int X, int Y){
    Q_UNUSED(n);
    Q_UNUSED(X);
    Q_UNUSED(Y);
}

void MainWindow::on_socket_readyRead()
{
    m_buffer.append(socket->readAll());
    // Mientras haya al menos 6 bytes (el tamaÃ±o mÃ­nimo de la cabecera)
    while (m_buffer.size() >= 6){
        int index = m_buffer.indexOf("UNER");
        if (index == -1) {  // Si no hay UNER, limpiar si el buffer crece mucho y salir
            if (m_buffer.size() > 512) m_buffer.clear();
            return;
        }
        if (index > 0) { // Alinear el buffer para que empiece exactamente en "UNER"
            m_buffer.remove(0, index);
        }
        if (m_buffer.size() < 6) return; // Esperar mÃ¡s datos
        // ESCUDO 1: Validar el TOKEN. Si no es 0xAA, este "UNER" es falso (datos basura).

        if ((uint8_t)m_buffer.at(5) != UNER_TOKEN) {
            m_buffer.remove(0, 1); // Borramos la 'U' falsa y el while vuelve a buscar
            continue;
        }
        uint8_t largoProtocolo = (uint8_t)m_buffer.at(4);
        int totalBytesEsperados = 6 + largoProtocolo;
        if (totalBytesEsperados > 100) { // Si el paquete es muy largo lo descartamos
            m_buffer.remove(0, 1);
            continue;
        }
        if (m_buffer.size() < totalBytesEsperados) return; // si el paquete llego cortado, esperamos al prÃ³ximo ciclo
        QByteArray paquete = m_buffer.left(totalBytesEsperados);
        uint8_t checksum_calculado = 0;
        for (int i = 0; i < totalBytesEsperados - 1; i++) { //  validaciÃ³n matematica (CHECKSUM). XOR de todos los bytes inclusive los del HEADER.
            checksum_calculado ^= (uint8_t)paquete.at(i);
        }
        uint8_t checksum_recibido = (uint8_t)paquete.at(totalBytesEsperados - 1);
        if (checksum_calculado != checksum_recibido) {// El paquete se corrompiÃ³ en el aire (WiFi) o es un falso positivo.
            qDebug() << "ERROR DE CHECKSUM. Descartando paquete.";
            m_buffer.remove(0, 1);
            continue;
        }
        m_buffer.remove(0, totalBytesEsperados);
        uint8_t cmd = (uint8_t)paquete.at(6);
        switch (cmd) {
        case CMD_ACK:
        case CMD_ALIVE:{ // CMD_ALIVE
            ui->RxTextEdit->append("ALIVE RECIBIDO");
            QByteArray datos;
            datos.append((char)0); // Agregamos el dato al payload
            enviarComando(CMD_ACK, datos);
            break;}
        case CMD_CHANGE_SETPOINT:
            ui->RxTextEdit->append("Â¡Robot confirma: Setpoint actualizado!");
            break;
        case CMD_PID_PITCH_KP:
            ui->RxTextEdit->append("Â¡Robot confirma: Kp actualizado!");
            break;
            // (Lo mismo para Ki y Kd)
        case CMD_DATA:
        {
            // --- CAMBIO DE TAMAÃ‘O EN LA EXTRACCIÃ“N ---
            QByteArray payload = paquete.mid(7, 48); // Extraemos 48 bytes (antes 44)
            if (payload.size() < qsizetype(sizeof(PayloadData_t))) break;
            const PayloadData_t* datos = reinterpret_cast<const PayloadData_t*>(payload.constData());
            // --- 1. ActualizaciÃ³n rÃ¡pida de UI ---
            ui->MPU_Ax->setText(QString::number(datos->acc_x));
            ui->MPU_Ay->setText(QString::number(datos->acc_y));
            ui->MPU_Az->setText(QString::number(datos->acc_z));

            ui->Pitch_LineEdit->setText(QString::number(datos->pitch_filtrado, 'f', 2));
            ui->AngleX_LineEdit->setText(QString::number(datos->gyro_pitch, 'f', 2));
           // float PCF = (float)datos->pitch_filtrado;
            actualizarPitch3D((float)datos->pitch_filtrado - 90.0f , 90.0f, 0.0f);
            ui->IR_1_LineEdit->setText(QString::number(datos->IR[0]));
            ui->IR_2_LineEdit->setText(QString::number(datos->IR[1]));
            ui->IR_3_LineEdit->setText(QString::number(datos->IR[2]));
            ui->IR_4_LineEdit->setText(QString::number(datos->IR[3]));
            ui->IR_5_LineEdit->setText(QString::number(datos->IR[4]));
            ui->IR_6_LineEdit->setText(QString::number(datos->IR[5]));
            ui->IR_7_LineEdit->setText(QString::number(datos->IR[6]));
            ui->IR_8_LineEdit->setText(QString::number(datos->IR[7]));
            ui->IR1_progressBar->setValue(datos->IR[0]);
            ui->IR2_progressBar->setValue(datos->IR[1]);
            ui->IR3_progressBar->setValue(datos->IR[2]);
            ui->IR4_progressBar->setValue(datos->IR[3]);
            ui->IR5_progressBar->setValue(datos->IR[4]);
            ui->IR6_progressBar->setValue(datos->IR[5]);
            ui->IR7_progressBar->setValue(datos->IR[6]);
            ui->IR8_progressBar->setValue(datos->IR[7]);

       //     ui->PosXLineEdit->setText(QString::number(datos->pos_x));
        //    ui->PosYLineEdit->setText(QString::number(datos->pos_y));

            ui->P_LineEdit->setText(QString::number(datos->pos_x));
            ui->I_LineEdit_3->setText(QString::number(datos->pos_y));
            ui->D_LineEdit_4->setText(QString::number(datos->velocidad));
            ui->OUT_LineEdit_5->setText(QString::number(datos->yaw_filtrado));
            break;
        }
        default:
            // Ahora si salta esto, es porque mandaste un CMD que no tenÃ©s en el switch.
            ui->RxTextEdit->append("CMD Desconocido, pero paquete vÃ¡lido: " + QString::number(cmd));
            break;
        }
    }
}

void MainWindow::on_socket_connected()
{
    ui->InfoTextEdit->append("Â¡CONEXIÃ“N EXITOSA!");
    ui->connect_pushButton->setText("Disconnect STM32");
    QByteArray datos;
    datos.append((char)1); // Agregamos el dato al payload
    enviarComando(CMD_TCP_CONNECTED, datos);
    ui->TPCCONECTED->setStyleSheet("background-color: rgb(0, 255, 0); border-radius: 10px; border: 1px solid black;");
}
void MainWindow::on_socket_disconnected(){
    ui->InfoTextEdit->append("Desconectado");
    ui->connect_pushButton->setText("Connect STM32");
    ui->TPCCONECTED->setStyleSheet("background-color: rgb(255, 0, 0); border-radius: 10px; border: 1px solid black;");
}
void MainWindow::on_udp_readyRead()
{
    while (udpSocket->hasPendingDatagrams()) {
        QNetworkDatagram datagram = udpSocket->receiveDatagram();
        QByteArray data = datagram.data();

        if (data.startsWith("UNER")) {
            processUnerV1Datagram(data);
        } else if (data == QByteArray("ACK\r\n")) {
            ui->InfoTextEdit->append("ACK recibido desde STM32");
            ui->TPCCONECTED->setStyleSheet("background-color: rgb(0, 255, 0); border-radius: 10px; border: 1px solid black;");
        } else {
            QString text = QString::fromLatin1(data);

            ui->RxTextEdit->append("UDP RX [" + datagram.senderAddress().toString() + ":" +
                                   QString::number(datagram.senderPort()) + "] " +
                                   QString(data.toHex(' ')).toUpper() + " | " + text.trimmed());
        }
    }
}
void MainWindow::on_connect_pushButton_clicked()
{
    QString ip = ui->TypeIP_lineEdit->text();
    quint16 port = ui->TypePORT_lineEdit->text().toUShort();

    if (!udpReady) {
        udpRemoteAddress = QHostAddress(ip);
        udpRemotePort = port;

        if (udpRemoteAddress.isNull() || udpRemotePort == 0) {
            ui->InfoTextEdit->append("IP o puerto UDP invÃƒÂ¡lido.");
            return;
        }

        if (udpSocket->state() != QAbstractSocket::BoundState) {
            if (!udpSocket->bind(QHostAddress::AnyIPv4, udpLocalPort, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint)) {
                ui->InfoTextEdit->append("No pude abrir UDP local en puerto " + QString::number(udpLocalPort) +
                                         ": " + udpSocket->errorString());
                return;
            }
        }

        if (!tcpServer->isListening()) {
            if (!tcpServer->listen(QHostAddress::AnyIPv4, port)) {
                ui->InfoTextEdit->append("No pude abrir TCP server en puerto " + QString::number(port) +
                                         ": " + tcpServer->errorString());
                udpSocket->close();
                return;
            }
        }

        udpReady = true;
        ui->connect_pushButton->setText("Disconnect STM32");
        ui->TPCCONECTED->setStyleSheet("background-color: rgb(255, 255, 0); border-radius: 10px; border: 1px solid black;");
        ui->InfoTextEdit->append("UDP/TCP listo. Local PC UDP: " +
                                 udpSocket->localAddress().toString() + ":" + QString::number(udpSocket->localPort()) +
                                 " | TCP server: 0.0.0.0:" + QString::number(port) +
                                 " | Destino UDP STM32: " + udpRemoteAddress.toString() + ":" + QString::number(udpRemotePort));
    } else {
        udpReady = false;
        if (tcpClient != nullptr) {
            tcpClient->disconnectFromHost();
        }
        tcpServer->close();
        udpSocket->close();
        ui->connect_pushButton->setText("Connect STM32");
        ui->TPCCONECTED->setStyleSheet("background-color: rgb(255, 0, 0); border-radius: 10px; border: 1px solid black;");
        ui->InfoTextEdit->append("UDP/TCP cerrado.");
    }
}
void MainWindow::on_SET_FRECUENCY_HB_BUTTON_clicked()
{
    if(udpReady || socket->state() == QAbstractSocket::ConnectedState){
        int valor = ui->HB_FRECUENCY_LineEdit->text().toInt();
        QByteArray datos;
        datos.append((char)valor); // Agregamos el dato al payload
        enviarComando(CMD_SET_HB, datos);
        ui->TxTextEdit->append("PC: Enviado CMD Heartbeat con valor " + QString::number(valor));
    } else {
        ui->InfoTextEdit->append("Error: No estÃ¡s conectado.");
    }
}
uint16_t MainWindow::unerCrc16Ccitt(const QByteArray &data)
{
    uint16_t crc = 0xFFFF;

    for (char raw : data) {
        crc ^= (static_cast<uint16_t>(static_cast<uint8_t>(raw)) << 8);

        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }

    return crc;
}

QByteArray MainWindow::buildUnerV1(uint8_t cmd, uint8_t flags, const QByteArray &payloadData)
{
    QByteArray paquete;

    if (payloadData.size() > 64) {
        return paquete;
    }

    unerLastBuiltSeq = unerNextSeq++;

    paquete.append("UNER", 4);
    paquete.append(static_cast<char>(UNER_V1_VERSION));
    paquete.append(static_cast<char>(cmd));
    paquete.append(static_cast<char>(flags));
    paquete.append(static_cast<char>(unerLastBuiltSeq));
    paquete.append(static_cast<char>(payloadData.size()));
    paquete.append(payloadData);

    uint16_t crc = unerCrc16Ccitt(paquete);
    paquete.append(static_cast<char>(crc & 0xFF));
    paquete.append(static_cast<char>((crc >> 8) & 0xFF));

    return paquete;
}

void MainWindow::processPendingAckRetry()
{
    bool tcpReady = tcpClient != nullptr && tcpClient->state() == QAbstractSocket::ConnectedState;

    if ((!udpReady && !tcpReady) || !unerAckWaiting) {
        return;
    }

    unerAckWaitTicks++;

    if (unerAckWaitTicks < 10) {
        return;
    }

    unerAckWaitTicks = 0;

    if (unerAckRetries >= 3) {
        ui->InfoTextEdit->append("ACK timeout cmd=" + QString::number(unerAckCmd) +
                                 " seq=" + QString::number(unerAckSeq));
        unerAckWaiting = false;
        unerAckFrame.clear();
        return;
    }

    qint64 written = -1;
    if (tcpReady) {
        written = tcpClient->write(unerAckFrame);
    } else {
        written = udpSocket->writeDatagram(unerAckFrame, udpRemoteAddress, udpRemotePort);
    }
    unerAckRetries++;

    if (written == unerAckFrame.size()) {
        ui->InfoTextEdit->append("Reintento UNER cmd=" + QString::number(unerAckCmd) +
                                 " seq=" + QString::number(unerAckSeq) +
                                 " intento=" + QString::number(unerAckRetries));
    } else {
        ui->InfoTextEdit->append("Error reintentando UNER");
    }
}

bool MainWindow::isCriticalCommand(uint8_t cmd) const
{
    switch (cmd) {
    case CMD_CHANGE_MODE:
    case CMD_CALIBRATE:
    case CMD_START:
    case CMD_STOP:
    case CMD_CHANGE_OLED_SCREEN:
    case CMD_CHANGE_DEADLINE_LEFT:
    case CMD_CHANGE_DEADLINE_RIGHT:
    case CMD_CHANGE_SETPOINT:
    case CMD_DEFINE_ZERO_SETPOINT:
    case CMD_ONOFFMOTORS:
    case CMD_PID_PITCH_KP:
    case CMD_PID_PITCH_KI:
    case CMD_PID_PITCH_KD:
    case CMD_PID_ALPHA:
    case CMD_PID_PITCH_LIM_INCLI:
    case CMD_PID_PITCH_CORECCION_RCSP:
    case CMD_PID_YAW_KP:
    case CMD_PID_YAW_KD:
    case CMD_PID_YAW_SP:
    case CMD_PID_YAW_MULTIPLICADOR:
    case CMD_IR_INICIAR_CALIBRACION:
    case CMD_IR_DETENER_CALIBRACION:
    case CMD_NETWORK_CHANGE_SSID:
    case CMD_NETWORK_CHANGE_PASSWORD:
    case CMD_TELEMETRY_START:
    case CMD_TELEMETRY_STOP:
    case CMD_HTTP_SOFTAP:
    case CMD_SET_YAW_PD:
    case CMD_SET_YAW_CONFIG:
    case CMD_SET_FL_CONFIG:
    case CMD_TURN_MANEUVER:
        return true;
    default:
        return false;
    }
}

void MainWindow::processUnerV1Datagram(const QByteArray &data)
{
    if (data.size() < 11) {
        return;
    }

    if (data.mid(0, 4) != "UNER") {
        return;
    }

    uint8_t version = static_cast<uint8_t>(data.at(4));
    uint8_t cmd = static_cast<uint8_t>(data.at(5));
    uint8_t flags = static_cast<uint8_t>(data.at(6));
    uint8_t seq = static_cast<uint8_t>(data.at(7));
    uint8_t len = static_cast<uint8_t>(data.at(8));
    int total = 4 + 1 + 1 + 1 + 1 + 1 + len + 2;

    if (version != UNER_V1_VERSION || data.size() != total) {
        return;
    }

    QByteArray withoutCrc = data.left(total - 2);
    uint16_t crcCalc = unerCrc16Ccitt(withoutCrc);
    uint16_t crcRx = static_cast<uint8_t>(data.at(total - 2)) |
                     (static_cast<uint16_t>(static_cast<uint8_t>(data.at(total - 1))) << 8);

    if (crcCalc != crcRx) {
        ui->RxTextEdit->append("UNER v1 CRC invalido");
        return;
    }

    handleUnerV1Packet(cmd, flags, seq, data.mid(9, len));
}

void MainWindow::processTcpUnerStream()
{
    while (m_buffer.size() >= 11) {
        int index = m_buffer.indexOf("UNER");
        if (index < 0) {
            if (m_buffer.size() > 256) {
                m_buffer.clear();
            }
            return;
        }

        if (index > 0) {
            m_buffer.remove(0, index);
        }

        if (m_buffer.size() < 11) {
            return;
        }

        uint8_t len = static_cast<uint8_t>(m_buffer.at(8));
        int total = 4 + 1 + 1 + 1 + 1 + 1 + len + 2;
        if (total > 220) {
            m_buffer.remove(0, 1);
            continue;
        }

        if (m_buffer.size() < total) {
            return;
        }

        QByteArray frame = m_buffer.left(total);
        m_buffer.remove(0, total);
        processUnerV1Datagram(frame);
    }
}

void MainWindow::handleUnerV1Packet(uint8_t cmd, uint8_t flags, uint8_t seq, const QByteArray &payload)
{
    Q_UNUSED(flags);

    switch (cmd) {
    case CMD_ACK:
        if (payload.size() >= 3) {
            uint8_t ackCmd = static_cast<uint8_t>(payload.at(0));
            uint8_t ackSeq = static_cast<uint8_t>(payload.at(1));
            uint8_t ackStatus = static_cast<uint8_t>(payload.at(2));

            ui->InfoTextEdit->append("ACK v1 cmd=" + QString::number(ackCmd) +
                                     " seq=" + QString::number(ackSeq) +
                                     " status=" + QString::number(ackStatus));

            if (ackCmd == CMD_TURN_MANEUVER) {
                QString statusText;
                switch (ackStatus) {
                case 0:
                    statusText = "TURN iniciado";
                    break;
                case 1:
                    statusText = "TURN rechazado: angulo/modo fuera de rango";
                    break;
                case 2:
                    statusText = "TURN rechazado: payload invalido";
                    break;
                case 3:
                    statusText = "TURN rechazado: requiere modo RC y motores activos";
                    break;
                case 4:
                    statusText = "TURN rechazado: datos MPU invalidos o detenidos";
                    break;
                default:
                    statusText = "TURN rechazado: estado " + QString::number(ackStatus);
                    break;
                }
                ui->InfoTextEdit->append(statusText);
            }

            if (unerAckWaiting && ackCmd == unerAckCmd && ackSeq == unerAckSeq) {
                unerAckWaiting = false;
                unerAckFrame.clear();
            }
        } else {
            ui->InfoTextEdit->append("ACK v1 seq=" + QString::number(seq));
        }
        ui->TPCCONECTED->setStyleSheet("background-color: rgb(0, 255, 0); border-radius: 10px; border: 1px solid black;");
        break;

    case CMD_DATA:
    {
        if (payload.size() == static_cast<int>(sizeof(PayloadDataMinV1_t))) {
            const uchar *p = reinterpret_cast<const uchar *>(payload.constData());
            PayloadDataMinV1_t datos;

            datos.acc_x = qFromLittleEndian<qint16>(p + 0);
            datos.acc_y = qFromLittleEndian<qint16>(p + 2);
            datos.acc_z = qFromLittleEndian<qint16>(p + 4);
            datos.pitch_cdeg = qFromLittleEndian<qint16>(p + 6);

            ui->MPU_Ax->setText(QString::number(datos.acc_x));
            ui->MPU_Ay->setText(QString::number(datos.acc_y));
            ui->MPU_Az->setText(QString::number(datos.acc_z));
            ui->Pitch_LineEdit->setText(QString::number(datos.pitch_cdeg / 100.0f, 'f', 2));
            actualizarPitch3D((datos.pitch_cdeg / 100.0f) - 90.0f, 90.0f, 0.0f);
            break;
        }

        if (payload.size() == static_cast<int>(sizeof(PayloadDataMpuV1_t))) {
            const uchar *p = reinterpret_cast<const uchar *>(payload.constData());
            PayloadDataMpuV1_t datos;

            datos.acc_x = qFromLittleEndian<qint16>(p + 0);
            datos.acc_y = qFromLittleEndian<qint16>(p + 2);
            datos.acc_z = qFromLittleEndian<qint16>(p + 4);
            datos.gyro_pitch = qFromLittleEndian<qint16>(p + 6);
            datos.gyro_yaw = qFromLittleEndian<qint16>(p + 8);
            datos.pitch_cdeg = qFromLittleEndian<qint16>(p + 10);
            datos.roll_cdeg = qFromLittleEndian<qint16>(p + 12);
            datos.yaw_cdeg = qFromLittleEndian<qint16>(p + 14);

            ui->MPU_Ax->setText(QString::number(datos.acc_x));
            ui->MPU_Ay->setText(QString::number(datos.acc_y));
            ui->MPU_Az->setText(QString::number(datos.acc_z));
            ui->Pitch_LineEdit->setText(QString::number(datos.pitch_cdeg / 100.0f, 'f', 2));
            ui->Yaw_lineEdit->setText(QString::number(datos.yaw_cdeg / 100.0f, 'f', 2));
            ui->AngleX_LineEdit->setText(QString::number(datos.roll_cdeg / 100.0f, 'f', 2));
            actualizarPitch3D((datos.pitch_cdeg / 100.0f) - 90.0f,
                              datos.yaw_cdeg / 100.0f,
                              datos.roll_cdeg / 100.0f);
            break;
        }

        if (payload.size() == static_cast<int>(sizeof(PayloadDataStateV1_t))) {
            const uchar *p = reinterpret_cast<const uchar *>(payload.constData());
            PayloadDataStateV1_t datos;

            datos.acc_x = qFromLittleEndian<qint16>(p + 0);
            datos.acc_y = qFromLittleEndian<qint16>(p + 2);
            datos.acc_z = qFromLittleEndian<qint16>(p + 4);
            datos.gyro_pitch = qFromLittleEndian<qint16>(p + 6);
            datos.gyro_yaw = qFromLittleEndian<qint16>(p + 8);
            datos.pitch_cdeg = qFromLittleEndian<qint16>(p + 10);
            datos.roll_cdeg = qFromLittleEndian<qint16>(p + 12);
            datos.yaw_cdeg = qFromLittleEndian<qint16>(p + 14);
            datos.pos_x_mm = qFromLittleEndian<qint32>(p + 16);
            datos.pos_y_mm = qFromLittleEndian<qint32>(p + 20);
            datos.velocidad_mm_s = qFromLittleEndian<qint16>(p + 24);
            datos.modo = static_cast<uint8_t>(payload.at(26));

            ui->MPU_Ax->setText(QString::number(datos.acc_x));
            ui->MPU_Ay->setText(QString::number(datos.acc_y));
            ui->MPU_Az->setText(QString::number(datos.acc_z));
            ui->Pitch_LineEdit->setText(QString::number(datos.pitch_cdeg / 100.0f, 'f', 2));
            ui->Yaw_lineEdit->setText(QString::number(datos.yaw_cdeg / 100.0f, 'f', 2));
            ui->AngleX_LineEdit->setText(QString::number(datos.roll_cdeg / 100.0f, 'f', 2));
            actualizarPitch3D((datos.pitch_cdeg / 100.0f) - 90.0f,
                              datos.yaw_cdeg / 100.0f,
                              datos.roll_cdeg / 100.0f);

            ui->P_LineEdit->setText(QString::number(datos.pos_x_mm / 1000.0f));
            ui->I_LineEdit_3->setText(QString::number(datos.pos_y_mm / 1000.0f));
            const float velocity_mm_s = datos.velocidad_mm_s * (9.80665f / 16384.0f) * 1000.0f;
            ui->D_LineEdit_4->setText(QString::number(velocity_mm_s, 'f', 1));
            static uint16_t velocityLogDiv = 0;
            if (++velocityLogDiv >= 20) {
                velocityLogDiv = 0;
                ui->RxTextEdit->append(QString("VEL_IMU: %1 mm/s").arg(velocity_mm_s, 0, 'f', 1));
            }
            ui->OUT_LineEdit_5->setText(QString::number(datos.yaw_cdeg / 100.0f, 'f', 2));

            QString modoTexto = QString::number(datos.modo);
            if (datos.modo == MODO_IDDLE) {
                modoTexto = "IDDLE";
            } else if (datos.modo == MODO_RC) {
                modoTexto = "RC";
            } else if (datos.modo == MODO_FL_INICIO) {
                modoTexto = "FL CAL";
            } else if (datos.modo == MODO_FL_BUSQUEDA_INICIAL) {
                modoTexto = "FL BUSQ";
            } else if (datos.modo == MODO_FL_SIGUIENDO) {
                modoTexto = "FL SIG";
            }
            ui->MODO_LABEL->setText("Modo: " + modoTexto);
            break;
        }

        if (payload.size() < static_cast<int>(sizeof(PayloadDataV1_t))) {
            ui->RxTextEdit->append("CMD_DATA v1 corto: " + QString::number(payload.size()));
            break;
        }

        const uchar *p = reinterpret_cast<const uchar *>(payload.constData());
        PayloadDataV1_t datos;
        datos.acc_x = qFromLittleEndian<qint16>(p + 0);
        datos.acc_y = qFromLittleEndian<qint16>(p + 2);
        datos.acc_z = qFromLittleEndian<qint16>(p + 4);
        datos.gyro_pitch = qFromLittleEndian<qint16>(p + 6);
        datos.gyro_yaw = qFromLittleEndian<qint16>(p + 8);
        datos.pitch_cdeg = qFromLittleEndian<qint16>(p + 10);
        datos.roll_cdeg = qFromLittleEndian<qint16>(p + 12);
        datos.yaw_cdeg = qFromLittleEndian<qint16>(p + 14);
        datos.pos_x_mm = qFromLittleEndian<qint32>(p + 16);
        datos.pos_y_mm = qFromLittleEndian<qint32>(p + 20);
        datos.velocidad_mm_s = qFromLittleEndian<qint16>(p + 24);
        datos.modo = static_cast<uint8_t>(payload.at(26));

        for (int i = 0; i < 8; i++) {
            datos.IR[i] = qFromLittleEndian<quint16>(p + 27 + (i * 2));
        }

        datos.infoAdicional = static_cast<uint8_t>(payload.at(43));
        bool turnActive = (datos.infoAdicional & 0x40) != 0;
        if (turnActive != turnManeuverTelemetryActive) {
            turnManeuverTelemetryActive = turnActive;
            ui->InfoTextEdit->append(turnActive ? "TURN activo en firmware" : "TURN finalizado en firmware");
        }

        ui->MPU_Ax->setText(QString::number(datos.acc_x));
        ui->MPU_Ay->setText(QString::number(datos.acc_y));
        ui->MPU_Az->setText(QString::number(datos.acc_z));
        ui->Pitch_LineEdit->setText(QString::number(datos.pitch_cdeg / 100.0f, 'f', 2));
        ui->Yaw_lineEdit->setText(QString::number(datos.yaw_cdeg / 100.0f, 'f', 2));
        ui->AngleX_LineEdit->setText(QString::number(datos.roll_cdeg / 100.0f, 'f', 2));
        actualizarPitch3D((datos.pitch_cdeg / 100.0f) - 90.0f,
                          datos.yaw_cdeg / 100.0f,
                          datos.roll_cdeg / 100.0f);

        ui->IR_1_LineEdit->setText(QString::number(datos.IR[0]));
        ui->IR_2_LineEdit->setText(QString::number(datos.IR[1]));
        ui->IR_3_LineEdit->setText(QString::number(datos.IR[2]));
        ui->IR_4_LineEdit->setText(QString::number(datos.IR[3]));
        ui->IR_5_LineEdit->setText(QString::number(datos.IR[4]));
        ui->IR_6_LineEdit->setText(QString::number(datos.IR[5]));
        ui->IR_7_LineEdit->setText(QString::number(datos.IR[6]));
        ui->IR_8_LineEdit->setText(QString::number(datos.IR[7]));
        ui->IR1_progressBar->setValue(datos.IR[0]);
        ui->IR2_progressBar->setValue(datos.IR[1]);
        ui->IR3_progressBar->setValue(datos.IR[2]);
        ui->IR4_progressBar->setValue(datos.IR[3]);
        ui->IR5_progressBar->setValue(datos.IR[4]);
        ui->IR6_progressBar->setValue(datos.IR[5]);
        ui->IR7_progressBar->setValue(datos.IR[6]);
        ui->IR8_progressBar->setValue(datos.IR[7]);

        ui->P_LineEdit->setText(QString::number(datos.pos_x_mm / 1000.0f));
        ui->I_LineEdit_3->setText(QString::number(datos.pos_y_mm / 1000.0f));
        const float velocity_mm_s = datos.velocidad_mm_s * (9.80665f / 16384.0f) * 1000.0f;
        ui->D_LineEdit_4->setText(QString::number(velocity_mm_s, 'f', 1));
        static uint16_t velocityLogDiv = 0;
        if (++velocityLogDiv >= 20) {
            velocityLogDiv = 0;
            ui->RxTextEdit->append(QString("VEL_IMU: %1 mm/s").arg(velocity_mm_s, 0, 'f', 1));
        }
        ui->OUT_LineEdit_5->setText(QString::number(datos.yaw_cdeg / 100.0f, 'f', 2));
        if (datos.modo == MODO_RC) {
            ui->MODO_LABEL->setText(turnActive ? "Modo: RC TURN" : "Modo: RC");
        }
        break;
    }

    default:
        ui->RxTextEdit->append("UNER v1 cmd=" + QString::number(cmd) +
                               " seq=" + QString::number(seq) +
                               " len=" + QString::number(payload.size()));
        break;
    }
}

void MainWindow::enviarComando(uint8_t cmd, const QByteArray &payloadData) {
    bool tcpReady = tcpClient != nullptr && tcpClient->state() == QAbstractSocket::ConnectedState;

    if (!tcpReady && !udpReady) {
        ui->InfoTextEdit->append("Error: enlace UDP/TCP no listo.");
        return;
    }

    uint8_t flags = isCriticalCommand(cmd) ? UNER_V1_FLAG_ACK_REQUIRED : 0;

    if (cmd == CMD_ALIVE) {
        flags |= UNER_V1_FLAG_ACK_REQUIRED;
    }

    if ((flags & UNER_V1_FLAG_ACK_REQUIRED) && unerAckWaiting) {
        ui->InfoTextEdit->append("Comando pendiente de ACK cmd=" + QString::number(unerAckCmd) +
                                 " seq=" + QString::number(unerAckSeq));
        return;
    }

    QByteArray paquete = buildUnerV1(cmd, flags, payloadData);

    if (paquete.isEmpty()) {
        ui->InfoTextEdit->append("No pude armar UNER v1 para cmd " + QString::number(cmd));
        return;
    }

    qint64 written = -1;
    if (tcpReady) {
        written = tcpClient->write(paquete);
    } else {
        written = udpSocket->writeDatagram(paquete, udpRemoteAddress, udpRemotePort);
    }

    if (written != paquete.size()) {
        ui->InfoTextEdit->append("Error enviando UNER v1");
    } else if (flags & UNER_V1_FLAG_ACK_REQUIRED) {
        unerAckWaiting = true;
        unerAckCmd = cmd;
        unerAckSeq = unerLastBuiltSeq;
        unerAckRetries = 0;
        unerAckWaitTicks = 0;
        unerAckFrame = paquete;
    }
}
void MainWindow::enviarFloat(uint8_t cmd, float valor) {
    QByteArray buffer;
    // Copiamos los 4 bytes del float tal cual estÃ¡n en memoria
    quint32 raw;
    memcpy(&raw, &valor, sizeof(raw));
    raw = qToLittleEndian<quint32>(raw);
    buffer.append(reinterpret_cast<const char*>(&raw), sizeof(raw));
    enviarComando(cmd, buffer);
}
void MainWindow::enviarYawConfig(float kp, float kd, float curveMultiplier, float filterAlpha, float steeringStep, float steeringLimit) {
    QByteArray payload;

    auto appendFloatLE = [&payload](float value) {
        quint32 raw;
        memcpy(&raw, &value, sizeof(raw));
        raw = qToLittleEndian<quint32>(raw);
        payload.append(reinterpret_cast<const char*>(&raw), sizeof(raw));
    };

    appendFloatLE(kp);
    appendFloatLE(kd);
    appendFloatLE(curveMultiplier);
    appendFloatLE(filterAlpha);
    appendFloatLE(steeringStep);
    appendFloatLE(steeringLimit);

    enviarComando(CMD_SET_YAW_CONFIG, payload);
    ui->TxTextEdit->append(QString("TX: YAW CONFIG Kp=%1 Kd=%2 SpeedRed=%3 Alpha=%4 Step=%5 Limit=%6")
                               .arg(kp, 0, 'f', 1)
                               .arg(kd, 0, 'f', 1)
                               .arg(curveMultiplier, 0, 'f', 4)
                               .arg(filterAlpha, 0, 'f', 2)
                               .arg(steeringStep, 0, 'f', 0)
                               .arg(steeringLimit, 0, 'f', 0));
}

void MainWindow::enviarFlConfig(float flSetpoint, quint16 motionMs, quint16 balanceMs) {
    QByteArray payload;

    quint32 spRaw;
    memcpy(&spRaw, &flSetpoint, sizeof(spRaw));
    spRaw = qToLittleEndian<quint32>(spRaw);
    payload.append(reinterpret_cast<const char*>(&spRaw), sizeof(spRaw));

    quint16 motionRaw = qToLittleEndian<quint16>(motionMs);
    payload.append(reinterpret_cast<const char*>(&motionRaw), sizeof(motionRaw));

    quint16 balanceRaw = qToLittleEndian<quint16>(balanceMs);
    payload.append(reinterpret_cast<const char*>(&balanceRaw), sizeof(balanceRaw));

    enviarComando(CMD_SET_FL_CONFIG, payload);
    ui->TxTextEdit->append(QString("TX: FL CONFIG SP=%1 MOVE=%2ms BAL=%3ms")
                               .arg(flSetpoint, 0, 'f', 2)
                               .arg(motionMs)
                               .arg(balanceMs));
}

void MainWindow::enviarTurnManeuver(float targetAngleDeg, quint8 wheelMode, quint8 wheelSelect, quint8 innerWheelPercent) {
    QByteArray payload;

    qint32 angleCdeg32 = qRound(targetAngleDeg * 100.0f);
    if (angleCdeg32 > 32767) {
        angleCdeg32 = 32767;
    } else if (angleCdeg32 < -32768) {
        angleCdeg32 = -32768;
    }

    qint16 angleCdeg = qToLittleEndian<qint16>(static_cast<qint16>(angleCdeg32));
    payload.append(reinterpret_cast<const char*>(&angleCdeg), sizeof(angleCdeg));
    payload.append(static_cast<char>(wheelMode));
    payload.append(static_cast<char>(wheelSelect));
    if (wheelMode == 2U) {
        payload.append(static_cast<char>(innerWheelPercent));
    }

    enviarComando(CMD_TURN_MANEUVER, payload);
    QString modeText = "2 wheels";
    if (wheelMode == 1U) {
        modeText = "1 wheel";
    } else if (wheelMode == 2U) {
        modeText = "arc";
    }

    ui->TxTextEdit->append(QString("TX: TURN angle=%1 deg mode=%2 wheel=%3 inner=%4%")
                               .arg(targetAngleDeg, 0, 'f', 1)
                               .arg(modeText)
                               .arg(wheelSelect == 0 ? "Left" : "Right")
                               .arg(innerWheelPercent));
}

void MainWindow::enviarInt16(uint8_t cmd, int16_t valor) {
    QByteArray buffer;
    qint16 littleEndianValue = qToLittleEndian<qint16>(valor);
    buffer.append(reinterpret_cast<const char*>(&littleEndianValue), sizeof(littleEndianValue));
    enviarComando(cmd, buffer);
}
void MainWindow::on_pushButton_clicked()
{
    if(udpReady || socket->state() == QAbstractSocket::ConnectedState){
        if(ui->pushButton->text() == "MotorIsOn"){
            ui->pushButton->setText("MotorIsOff");
            QByteArray datos;
            datos.append((char)1); // Agregamos el dato al payload
            enviarComando(CMD_ONOFFMOTORS, datos);
            ui->InfoTextEdit->append("PC: MOTORS ON ");
        }
        else{
            if(ui->pushButton->text() == "MotorIsOff"){
                ui->pushButton->setText("MotorIsOn");
                QByteArray datos;
                datos.append((char)0); // Agregamos el dato al payload

                enviarComando(CMD_ONOFFMOTORS, datos);
                ui->InfoTextEdit->append("PC: MOTORS OFF ");
            }
        }
    } else {
        ui->InfoTextEdit->append("Error: No estÃ¡s conectado.");
    }
}
void MainWindow::on_GetAlivePushButton_clicked(){
    if (udpReady || (tcpClient != nullptr && tcpClient->state() == QAbstractSocket::ConnectedState)) {
        QByteArray datos;
        enviarComando(CMD_ALIVE, datos);
        ui->TxTextEdit->append("TX: CMD_ALIVE v1 -> " + udpRemoteAddress.toString() + ":" +
                               QString::number(udpRemotePort));
    } else {
        ui->InfoTextEdit->append("Error: UDP/TCP no esta listo.");
    }

    return;

    if(udpReady || socket->state() == QAbstractSocket::ConnectedState){
            QByteArray datos;
            datos.append((char)0); // Agregamos el dato al payload
            enviarComando(CMD_ALIVE, datos);
            ui->TxTextEdit->append("PC: ALIVE");
    } else {
        ui->InfoTextEdit->append("Error: No estÃ¡s conectado.");
    }
}
void MainWindow::on_Kp_pushButton_clicked(){
    if(udpReady || socket->state() == QAbstractSocket::ConnectedState){
        float valor = ui->Kp_LineEdit->text().toFloat();
        enviarFloat(CMD_PID_PITCH_KP, valor);
        ui->TxTextEdit->append("PC: Cambia el valor de KP a " + QString::number(valor));
    } else {
        ui->TxTextEdit->append("Error: No estÃ¡s conectado.");
    }
}
void MainWindow::on_Ki_pushButton_clicked(){
    if(udpReady || socket->state() == QAbstractSocket::ConnectedState){
        float valor = ui->Ki_LineEdit->text().toFloat();
        enviarFloat(CMD_PID_PITCH_KI, valor);
        ui->TxTextEdit->append("PC: Cambia el valor de KI a " + QString::number(valor));
    } else {
        ui->TxTextEdit->append("Error: No estÃ¡s conectado.");
    }
}
void MainWindow::on_Kd_pushButton_clicked(){
    if(udpReady || socket->state() == QAbstractSocket::ConnectedState){
        float valor = ui->Kd_LineEdit->text().toFloat();
         enviarFloat(CMD_PID_PITCH_KD, valor);
        ui->TxTextEdit->append("PC: Cambia el valor de KD a " + QString::number(valor));
    } else {
        ui->TxTextEdit->append("Error: No estÃ¡s conectado.");
    }
}
void MainWindow::on_Setpoint_spinBox_textChanged(const QString &arg1){
    if(udpReady || socket->state() == QAbstractSocket::ConnectedState){
        int16_t valor = arg1.toInt();
        enviarInt16(CMD_CHANGE_SETPOINT, valor);
        ui->TxTextEdit->append("PC: SetPoint : " + QString::number(valor));
    } else {
        ui->TxTextEdit->append("Error: No estÃ¡s conectado.");
    }
}
void MainWindow::on_PID_Alpha_pushButton_clicked(){
    if(udpReady || socket->state() == QAbstractSocket::ConnectedState){
        float valor = ui->PID_Alpha_LineEdit->text().toFloat();// ui->Kd_LineEdit->text().toInt();
        enviarFloat(CMD_PID_ALPHA, valor);
  //      ui->TxTextEdit->append("PC: SetPoint : " + QString::number(valor));
    } else {
        ui->TxTextEdit->append("Error: No estÃ¡s conectado.");
    }
}
void MainWindow::on_OffPushButton_clicked(){
    enviarInt16(CMD_CHANGE_OLED_SCREEN, 0);
    ui->TxTextEdit->append("PC: CMD_CHANGE_OLED_SCREEN");
    ui->InfoTextEdit->append("PC: Screen 2");
}
void MainWindow::on_Screen1PushButton_clicked(){
    enviarInt16(CMD_CHANGE_OLED_SCREEN, 1);
    ui->TxTextEdit->append("PC: CMD_CHANGE_OLED_SCREEN");
    ui->InfoTextEdit->append("PC: Screen 2");
}
void MainWindow::on_Screen2PushButton_clicked(){
    enviarInt16(CMD_CHANGE_OLED_SCREEN, 2);
    ui->TxTextEdit->append("PC: CMD_CHANGE_OLED_SCREEN");
    ui->InfoTextEdit->append("PC: Screen 2");
}

void MainWindow::on_LDL_pushButton_clicked(){
    if(udpReady || socket->state() == QAbstractSocket::ConnectedState){
        int16_t valor = ui->LDL_LineEdit->text().toInt();
        enviarInt16(CMD_CHANGE_DEADLINE_LEFT, valor);
    }
}
void MainWindow::on_RDL_pushButton_clicked(){
    if(udpReady || socket->state() == QAbstractSocket::ConnectedState){
        int16_t valor = ui->RDL_LineEdit->text().toInt();
        enviarInt16(CMD_CHANGE_DEADLINE_RIGHT, valor);
    }
}
void MainWindow::sendWifiConfig(QString ssid, QString pass) {
    QByteArray payload;
    payload.append(ssid.toUtf8());
    payload.append(";");
    payload.append(pass.toUtf8());

    // Estructura segÃºn Protocolo UNER
    // Length = 1(CMD) + N(Payload) + 1(Checksum)
    uint8_t cmd = 20;
    uint8_t length = 1 + payload.size() + 1;
    uint8_t token = UNER_TOKEN; // Debe coincidir con el valor en el ESP-01

    QByteArray packet;
    packet.append("UNER", 4);               // Header [0..3]
    packet.append(static_cast<char>(length)); // Length [4]
    packet.append(static_cast<char>(token));  // Token [5]
    packet.append(static_cast<char>(cmd));    // CMD [6]
    packet.append(payload);                   // Payload [7...7+N]

    // CÃ¡lculo del Checksum: XOR de todos los bytes inclusive HEADER
    uint8_t checksum = 0;
    for(int i = 0; i < packet.size(); i++) {
        checksum ^= static_cast<uint8_t>(packet.at(i));
    }
    packet.append(static_cast<char>(checksum)); // Checksum final
    if(socket->isOpen()){
        socket->write(packet);
      //  socket->flush(); // Asegura el envÃ­o inmediato
    }
}
void MainWindow::on_REDpushButton_clicked()
{
    if(ui->REDlineEdit->text().isEmpty() || ui->PASSWORDlineEdit->text().isEmpty()){
        // Mostrar un mensaje de error en la UI
        return;
    }
    sendWifiConfig(ui->REDlineEdit->text(), ui->PASSWORDlineEdit->text());
}
void MainWindow::actualizarRotacionRobot(float pitch, float yaw, float roll) {
    if (!ui->quickWidget->rootObject()) return;
    QObject *robot = ui->quickWidget->rootObject()->findChild<QObject*>("robotVisual");
    if (robot) {
        robot->setProperty("eulerRotation", QVector3D(pitch, yaw, roll));
    } else {
        qDebug() << "Sigo sin encontrar robotVisual. RevisÃ¡ el objectName en el Model.";
    }
}
void MainWindow::actualizarPitch3D(float pitch, float yaw, float roll) {
    if (!ui->PitchQuickWidget->rootObject()) return;
    QObject *robot = ui->PitchQuickWidget->rootObject()->findChild<QObject*>("robotVisual");

    if (robot) {
        robot->setProperty("eulerRotation", QVector3D(pitch, yaw, roll));
    } else {
        qDebug() << "Sigo sin encontrar robotVisual. RevisÃ¡ el objectName en el Model.";
    }
}
void MainWindow::actualizarGrafico() {
    // AquÃ­ obtendrÃ­as tus datos reales del sistema
    double P_VALUE = ui->P_LineEdit->text().toDouble();
    double I_VALUE = ui->I_LineEdit_3->text().toDouble();
    double D_VALUE = ui->D_LineEdit_4->text().toDouble();
    double OUT_VALUE = ui->OUT_LineEdit_5->text().toDouble();

    // AÃ±adir puntos a las series
    seriesA->append(tiempoX, P_VALUE);
    seriesB->append(tiempoX, I_VALUE);
    seriesC->append(tiempoX, D_VALUE);
    seriesD->append(tiempoX, OUT_VALUE);
    // AÃ±adir puntos a las series
    // Desplazar el eje X para que se vea como un osciloscopio
    if (tiempoX > 100) {
        // Obtenemos el eje X del grÃ¡fico y lo desplazamos


        ui->graphicsView->chart()->axes(Qt::Horizontal).first()->setRange(tiempoX - 100, tiempoX);
    }
    tiempoX++;
}
void MainWindow::actualizarGraficoPID() {
    double P_VALUE = ui->P_LineEdit->text().toDouble();
    double I_VALUE = ui->I_LineEdit_3->text().toDouble();
    // AÃ±adir puntos a las series
    seriesA->append(tiempoX2, P_VALUE);
    seriesB->append(tiempoX2, I_VALUE);
  //  seriesA->append(tiempoX2, D_VALUE);
   // seriesB->append(tiempoX2, OUT_VALUE);
    // Desplazar el eje X para que se vea como un osciloscopio
    if (tiempoX2 > 100) {
        // Obtenemos el eje X del grÃ¡fico y lo desplazamos
       // ui->graphicsView_2->chart()->axes(Qt::Horizontal).first()->setRange(tiempoX2 - 100, tiempoX2);
    }
    tiempoX2++;
}
void MainWindow::on_Home_pushButton_clicked(){                  ui->stackedWidget->setCurrentIndex(2);}
void MainWindow::on_Ejecucion_pushButton_clicked(){             ui->stackedWidget->setCurrentIndex(1);}
void MainWindow::on_ScreenCalibrar_pushbutton_clicked(){        ui->stackedWidget->setCurrentIndex(3);}
void MainWindow::on_SistemasDeControl_pushButton_clicked(){     ui->stackedWidget->setCurrentIndex(0);}
void MainWindow::on_Advanced_pushButton_clicked(){              ui->stackedWidget->setCurrentWidget(ui->PaginaComandosAvanzados);}
/*
void MainWindow::on_MODO_IDDLE_pushButton_clicked()
{
    if(udpReady || socket->state() == QAbstractSocket::ConnectedState) {
        enviarInt16(CMD_CHANGE_MODE, MODO_IDDLE);
        ui->MODO_LABEL->setText("Modo: iddle");
    }
}

void MainWindow::on_MODO_RC_pushButton_clicked()
{
    if(udpReady || socket->state() == QAbstractSocket::ConnectedState){
        enviarInt16(CMD_CHANGE_MODE, MODO_RC);
        ui->MODO_LABEL->setText("Modo: RC");
    }
}
void MainWindow::on_MOD_FOLLOWLINE_pushButton_clicked()
{
    if(udpReady || socket->state() == QAbstractSocket::ConnectedState){
        enviarInt16(CMD_CHANGE_MODE, MODO_FOLLOWLINE);
        ui->MODO_LABEL->setText("Modo: Follow Line");
    }
}
*/
void MainWindow::on_StartCalibratePushButton_clicked(){
    if(udpReady || socket->state() == QAbstractSocket::ConnectedState){
        enviarInt16(CMD_IR_INICIAR_CALIBRACION, 0);
        ui->InfoTextEdit->append("PC: Inicia calibracion IR de linea");
    }
}
void MainWindow::on_StopCalibratePushButton_clicked(){
    if(udpReady || socket->state() == QAbstractSocket::ConnectedState){
        enviarInt16(CMD_IR_DETENER_CALIBRACION, 0);
        ui->InfoTextEdit->append("PC: Detiene calibracion IR de linea");
    }
}
void MainWindow::on_SP_yaw_pushButton_clicked(){
    if(udpReady || socket->state() == QAbstractSocket::ConnectedState){
     //   float dato = ui->Setpoint_FL_doubleSpinBox->text().toFloat();
      //  enviarFloat(CMD_PID_YAW_SP, dato);
    }
}
void MainWindow::on_Kd_yaw_pushButton_clicked(){
    if(udpReady || socket->state() == QAbstractSocket::ConnectedState){
        float dato = ui->Kd_yaw_LineEdit->text().toFloat();
        enviarFloat(CMD_PID_YAW_KD, dato);
        }
}
void MainWindow::on_Kp_yaw_pushButton_clicked(){
    if(udpReady || socket->state() == QAbstractSocket::ConnectedState){
        float valor = ui->Kp_yaw_LineEdit->text().toFloat();
        enviarFloat(CMD_PID_YAW_KP, valor);
        ui->TxTextEdit->append("PC: Cambia el valor de KP a " + QString::number(valor));
    } else {
        ui->TxTextEdit->append("Error: No estÃ¡s conectado.");
    }
}
void MainWindow::on_ChangeModeIddlePushButton_clicked(){
    if(udpReady || socket->state() == QAbstractSocket::ConnectedState){

        enviarInt16( CMD_CHANGE_MODE, MODO_IDDLE);
        ui->MODO_LABEL->setText("Modo: IDDLE");
        ui->TPCCONECTED_2->setStyleSheet("background-color: rgb(0, 255, 0); border-radius: 10px; border: 1px solid black;");
    }

}
void MainWindow::on_ChangeModeRCPushButton_clicked(){
    if(udpReady || socket->state() == QAbstractSocket::ConnectedState){

        enviarInt16( CMD_CHANGE_MODE, MODO_RC);
        ui->MODO_LABEL->setText("Modo: RC");
        ui->TPCCONECTED_2->setStyleSheet("background-color: rgb(255, 0, 0); border-radius: 10px; border: 1px solid black;");
    }

}
void MainWindow::on_ChangeModeFLPushButton_clicked(){
    if(udpReady || socket->state() == QAbstractSocket::ConnectedState){
        enviarInt16( CMD_CHANGE_MODE, MODO_FL_BUSQUEDA_INICIAL);
        ui->MODO_LABEL->setText("Modo: FL BUSQUEDA");
        ui->TPCCONECTED_2->setStyleSheet("background-color: rgb(0, 0, 255); border-radius: 10px; border: 1px solid black;");
    }

}
void MainWindow::on_Setpoint_doubleSpinBox_textChanged(const QString &arg1)
{
    if(udpReady || socket->state() == QAbstractSocket::ConnectedState){
        int16_t valor = arg1.toFloat();
        enviarFloat(CMD_CHANGE_SETPOINT, valor);
        ui->TxTextEdit->append("PC: SetPoint : " + QString::number(valor));
    } else {
        ui->TxTextEdit->append("Error: No estÃ¡s conectado.");
    }
}


void MainWindow::on_Screen2PushButton_2_clicked()
{
    enviarInt16(CMD_CHANGE_OLED_SCREEN, 3);
    ui->TxTextEdit->append("PC: CMD_CHANGE_OLED_SCREEN");
    ui->InfoTextEdit->append("PC: Screen 3");
}


void MainWindow::on_Setpoint_FL_doubleSpinBox_textChanged(const QString &arg1)
{
    if(udpReady || socket->state() == QAbstractSocket::ConnectedState){
        float valor = arg1.toFloat();
        enviarFloat(CMD_CHANGE_SETPOINT, valor);
        ui->TxTextEdit->append("PC: SetPoint : " + QString::number(valor));
    } else {
        ui->TxTextEdit->append("Error: No estÃ¡s conectado.");
    }
}
void MainWindow::on_Ajust_RC_Setpoint_spinBox_textChanged(const QString &arg1){
    if(udpReady || socket->state() == QAbstractSocket::ConnectedState){
        float valor = arg1.toFloat();
        enviarFloat(CMD_PID_PITCH_CORECCION_RCSP, valor);
    }
}
void MainWindow::on_Limite_error_spinBox_textChanged(const QString &arg1){
    if(udpReady || socket->state() == QAbstractSocket::ConnectedState){
        float valor = arg1.toFloat();
        enviarFloat(CMD_PID_PITCH_LIM_INCLI, valor);
    }

}


void MainWindow::on_Setpoint_FL_spinBox_textChanged(const QString &arg1)
{

}

