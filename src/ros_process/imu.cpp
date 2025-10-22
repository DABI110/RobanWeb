#include "ros_process/imu.h"
#include "socket_process/websocketworker.h"
#include "util/load_param.hpp"

ImuMonitor::ImuMonitor(WebSocketWorker *worker, QObject *parent)
    : QObject(parent), m_worker(worker)
{
    imu_topic_name = loadTopicFromConfig("imu_topic");
}

ImuMonitor::~ImuMonitor() {}

// 订阅IMU话题
void ImuMonitor::start(){
    qDebug() << "ImuMonitor::start() - 准备订阅IMU话题";
    if(!m_worker) {
        qDebug() << "ImuMonitor::start() - WebSocketWorker为空，无法订阅";
        return;
    }
    
    // 检查话题名是否有效
    if(imu_topic_name.isEmpty()) {
        qDebug() << "ImuMonitor::start() - 话题名为空，使用默认值";
        imu_topic_name = "/MediumSize/SensorHub/Imu";
    }
    
    qDebug() << "ImuMonitor::start() - 订阅话题: " << imu_topic_name;
    
    // send subscribe request for IMU
    QJsonObject subscribeMsg;
    subscribeMsg["op"] = "subscribe";
    subscribeMsg["topic"] = imu_topic_name;
    subscribeMsg["type"] = "sensor_msgs/Imu";
    QJsonDocument doc(subscribeMsg);
    QString payload = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
    QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload));
}

void ImuMonitor::onMessageReceived(const QString &message){
    // 解析JSON消息
    QJsonDocument doc = QJsonDocument::fromJson(message.toUtf8());
    if(!doc.isObject()) return;
    QJsonObject obj = doc.object();

    // 检查操作类型
    if (obj["op"].toString() != "publish") return;
    
    // 检查话题
    QString topic = obj["topic"].toString();
    if (topic.isEmpty()) return; // 跳过空话题
    
    // 确保是我们关心的IMU话题
    if (topic != imu_topic_name) return;
    
    // 检查消息对象
    if (!obj.contains("msg") || !obj["msg"].isObject()) return;
    QJsonObject msgObj = obj["msg"].toObject();
    
    // 处理方向信息
    if (msgObj.contains("orientation") && msgObj["orientation"].isObject()) {
        QJsonObject ori = msgObj["orientation"].toObject();
        double w = ori.value("w").toDouble();
        double x = ori.value("x").toDouble();
        double y = ori.value("y").toDouble();
        double z = ori.value("z").toDouble();
        emit orientationUpdated(w, x, y, z);
    }

    // 处理角速度信息
    if (msgObj.contains("angular_velocity") && msgObj["angular_velocity"].isObject()) {
        QJsonObject ang = msgObj["angular_velocity"].toObject();
        double ax = ang.value("x").toDouble();
        double ay = ang.value("y").toDouble();
        double az = ang.value("z").toDouble();
        emit angularVelocityUpdated(ax, ay, az);
    }

    // 处理线性加速度信息
    if (msgObj.contains("linear_acceleration") && msgObj["linear_acceleration"].isObject()) {
        QJsonObject lin = msgObj["linear_acceleration"].toObject();
        double lx = lin.value("x").toDouble();
        double ly = lin.value("y").toDouble();
        double lz = lin.value("z").toDouble();
        emit linearAccelerationUpdated(lx, ly, lz);
    }
}
