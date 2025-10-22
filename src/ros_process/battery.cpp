#include "ros_process/battery.h"
#include "socket_process/websocketworker.h"
#include "util/load_param.hpp"

// Map voltage to percent using a simple linear mapping as placeholder
// You can replace this with the more complex table from the python script if needed
static const double MIN_VOLTAGE = 10.0; // corresponding to 0%
static const double MAX_VOLTAGE = 12.46; // corresponding to 100%

BatteryMonitor::BatteryMonitor(WebSocketWorker *worker, QObject *parent)
    : QObject(parent), m_worker(worker)
{
    battery_topic_name = loadTopicFromConfig("battery_topic");
}
BatteryMonitor::~BatteryMonitor() {}

// 订阅电量话题
void BatteryMonitor::start()
{
    qDebug() << "BatteryMonitor::start() - 准备订阅电量话题";
    if (!m_worker) {
        qDebug() << "BatteryMonitor::start() - WebSocketWorker为空，无法订阅";
        return;
    }
    
    // 检查话题名是否有效
    if(battery_topic_name.isEmpty()) {
        qDebug() << "BatteryMonitor::start() - 话题名为空，跳过订阅";
        return;
    }
    
    qDebug() << "BatteryMonitor::start() - 订阅话题: " << battery_topic_name;
    
    // send subscribe request for BatteryState
    QJsonObject subscribeMsg;
    subscribeMsg["op"] = "subscribe";
    subscribeMsg["topic"] = battery_topic_name;
    subscribeMsg["type"] = "sensor_msgs/BatteryState";
    QJsonDocument doc(subscribeMsg);
    QString payload = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
    QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload));
}





void BatteryMonitor::onMessageReceived(const QString &message)
{
    // 解析JSON消息
    QJsonDocument doc = QJsonDocument::fromJson(message.toUtf8());
    if (!doc.isObject()) return;
    QJsonObject obj = doc.object();

    // 检查操作类型
    if (obj["op"].toString() != "publish") return;
    
    // 检查话题
    QString topic = obj["topic"].toString();
    if (topic.isEmpty()) return; // 跳过空话题
    
    // 确保是我们关心的电池话题
    if (topic != battery_topic_name) return;
    
    // 检查消息对象
    if (!obj.contains("msg") || !obj["msg"].isObject()) return;
    QJsonObject msgObj = obj["msg"].toObject();
    
    // 提取电压字段
    if (!msgObj.contains("voltage")) return;
    double voltage = msgObj["voltage"].toDouble();
    
    // 转换电压为百分比并发送信号
    int percent = voltageToPercent(voltage);
    emit batteryLevelChanged(percent);
}

int BatteryMonitor::voltageToPercent(double voltage) const
{
    if (voltage <= MIN_VOLTAGE) return 0;
    if (voltage >= MAX_VOLTAGE) return 100;
    double t = (voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE);
    return static_cast<int>(t * 100.0 + 0.5);
}


