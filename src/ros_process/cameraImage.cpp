#include "ros_process/cameraImage.h"
#include "socket_process/websocketworker.h"
#include "util/load_param.hpp"

CameraImageMonitor::CameraImageMonitor(WebSocketWorker *worker, QObject *parent, const QString &topic_name)
    : QObject(parent), m_worker(worker)
{
    // store provided topic locally to avoid lifetime issues with caller-owned strings
    act_topic_name = topic_name;
    init();
    topic_parse();
}

CameraImageMonitor::~CameraImageMonitor() {}

// 设置显示尺寸
void CameraImageMonitor::setTargetSize(const QSize &size) {
    m_targetSize = size;
}
// 设置最大帧率
void CameraImageMonitor::setMaxFps(int fps) {
    if (fps <= 0) return;
    m_frameIntervalMs = 1000 / fps;
}

void CameraImageMonitor::topic_parse(){
    // 从配置文件加载所有话题信息
    QDir d(QCoreApplication::applicationDirPath());
    QStringList configPaths = {
        d.filePath("config/topic_config.yaml"),
        d.filePath("../config/topic_config.yaml"),
        QDir::current().filePath("config/topic_config.yaml")
    };
    
    QString configPath;
    bool configFound = false;
    for (const QString &path : configPaths) {
        QFile f(path);
        if (f.exists()) {
            configPath = path;
            configFound = true;
            qDebug() << "找到配置文件: " << path;
            break;
        }
    }
    
    if (!configFound) {
        qDebug() << "警告: 未找到话题配置文件！";
        qDebug() << "尝试过的路径: " << configPaths;
    } else {
        qDebug() << "正在从配置文件加载话题: " << configPath;
    }

    // cameraCompressed_topic_name = loadTopicFromConfig("cameraCompressed_topic");
    cameraCompressed_topic_name = "/camera/color/image_raw/compressed"   ;
    // cameraCompressed_topic_type = loadTopicFromConfig("cameraCompressed_topic_type");
    cameraCompressed_topic_type = "sensor_msgs/CompressedImage";
    // cameraRaw_topic_name = loadTopicFromConfig("cameraRaw_topic");
    // cameraRaw_topic_type = loadTopicFromConfig("cameraRaw_topic_type");
    cameraRaw_topic_name =  "/camera/color/image_raw"  ;
    cameraRaw_topic_type = "sensor_msgs/Image";
    // featureImageRaw_topic_name = loadTopicFromConfig("featureImageRaw_topic");
    // featureImageRaw_topic_type = loadTopicFromConfig("featureImageRaw_topic_type");
    featureImageRaw_topic_name = "/SLAM/FeaturePoint/Image";
    featureImageRaw_topic_type = "sensor_msgs/Image";
    // featureImageCompressed_topic_name = loadTopicFromConfig("featureImageCompressed_topic");
    // featureImageCompressed_topic_type = loadTopicFromConfig("featureImageCompressed_topic_type");
    featureImageCompressed_topic_name =  "/SLAM/FeaturePoint/Image/compressed"    ;
    featureImageCompressed_topic_type =  "sensor_msgs/CompressedImage";
    // 打印调试信息，检查配置是否成功加载
    qDebug() << "话题解析结果:";
    qDebug() << "相机压缩话题: " << (cameraCompressed_topic_name.isEmpty() ? "空" : cameraCompressed_topic_name) 
             << ", 类型: " << (cameraCompressed_topic_type.isEmpty() ? "空" : cameraCompressed_topic_type);
    qDebug() << "相机原始话题: " << (cameraRaw_topic_name.isEmpty() ? "空" : cameraRaw_topic_name) 
             << ", 类型: " << (cameraRaw_topic_type.isEmpty() ? "空" : cameraRaw_topic_type);
    qDebug() << "特征点压缩话题: " << (featureImageCompressed_topic_name.isEmpty() ? "空" : featureImageCompressed_topic_name)
             << ", 类型: " << (featureImageCompressed_topic_type.isEmpty() ? "空" : featureImageCompressed_topic_type);
    qDebug() << "特征点原始话题: " << (featureImageRaw_topic_name.isEmpty() ? "空" : featureImageRaw_topic_name)
             << ", 类型: " << (featureImageRaw_topic_type.isEmpty() ? "空" : featureImageRaw_topic_type);

    // act_topic_name may have been set by constructor from provided topic; if empty, choose defaults
    if (act_topic_name.isEmpty() && !cameraCompressed_topic_name.isEmpty()) {
        act_topic_name = cameraCompressed_topic_name;
        qDebug() << "使用默认话题: " << act_topic_name;
    }
    
    // 根据话题名称设置对应的类型
    if(!act_topic_name.isEmpty()) {
        if(act_topic_name == cameraCompressed_topic_name || act_topic_name == featureImageCompressed_topic_name){
            act_topic_type = cameraCompressed_topic_type;
        }else if(act_topic_name == cameraRaw_topic_name || act_topic_name == featureImageRaw_topic_name){
            act_topic_type = cameraRaw_topic_type;  
        }else{
            // 默认使用压缩图像话题
            if(!cameraCompressed_topic_name.isEmpty()) {
                act_topic_name = cameraCompressed_topic_name;
                act_topic_type = cameraCompressed_topic_type;
                qDebug() << "使用默认压缩图像话题: " << act_topic_name;
            }
        }
    } else {
        qDebug() << "警告: 无法设置有效的话题名称";
    }
    
    qDebug() << "最终使用的话题: " << act_topic_name << ", 类型: " << act_topic_type;
}

void CameraImageMonitor::init(){
    m_lastDecodeTimer.start();
}

// 订阅图像话题
void CameraImageMonitor::start(){
    qDebug() << "开始订阅话题流程...";
    
    if(!m_worker) {
        qDebug() << "错误: WebSocket工作线程为空，无法订阅话题";
        return;
    }
    
    qDebug() << "正在检查要订阅的话题...";
    qDebug() << "当前设置的实际话题: " << (act_topic_name.isEmpty() ? "空" : act_topic_name)
             << ", 类型: " << (act_topic_type.isEmpty() ? "空" : act_topic_type);
    
    // 只订阅IMU话题

    if(!cameraCompressed_topic_name.isEmpty()) {
        // 订阅IMU话题
        QJsonObject subscribeMsg;
        subscribeMsg["op"] = "subscribe";
        subscribeMsg["topic"] = cameraCompressed_topic_name;
        subscribeMsg["type"] = cameraCompressed_topic_type;
        QJsonDocument doc(subscribeMsg);
        QString payload = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
        // qDebug() << "订阅IMU话题: " << imu_topic;
        QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload));
    } else {
        qDebug() << "IMU话题为空，跳过订阅";
    }

    if(!featureImageCompressed_topic_name.isEmpty()) {
        // 订阅IMU话题
        QJsonObject subscribeMsg;
        subscribeMsg["op"] = "subscribe";
        subscribeMsg["topic"] = featureImageCompressed_topic_name;
        subscribeMsg["type"] = featureImageCompressed_topic_type;
        QJsonDocument doc(subscribeMsg);
        QString payload = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
        // qDebug() << "订阅IMU话题: " << imu_topic;
        QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload));
    } else {
        qDebug() << "IMU话题为空，跳过订阅";
    }
    
    // 暂时不订阅相机话题，直到我们解决空话题问题
    qDebug() << "跳过相机话题订阅，专注于修复空话题错误";
}

void CameraImageMonitor::stop() {
    qDebug() << "开始取消订阅话题流程...";
    
    if (!m_worker) {
        qDebug() << "错误: WebSocket工作线程为空，无法取消订阅话题";
        return;
    }
    
    // QString imu_topic = loadTopicFromConfig("imu_topic");
    
    // 暂时不取消订阅相机话题，因为我们也没有订阅它
    qDebug() << "跳过取消订阅相机话题";
    
    // 检查IMU话题是否为空，不为空则取消订阅
    if(!cameraCompressed_topic_name.isEmpty()) {
        QJsonObject unsub;
        unsub["op"] = "unsubscribe";
        unsub["topic"] = cameraCompressed_topic_name;
        QJsonDocument doc(unsub);
        QString payload = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
        // qDebug() << "取消订阅IMU话题: " << cameraCompressed_topic_name;
        QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload));
    }

    if(!featureImageCompressed_topic_name.isEmpty()) {
        QJsonObject unsub;
        unsub["op"] = "unsubscribe";
        unsub["topic"] = featureImageCompressed_topic_name;
        QJsonDocument doc(unsub);
        QString payload = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
        // qDebug() << "取消订阅IMU话题: " << cameraCompressed_topic_name;
        QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload));
    }
    
    // clear cached image
    {
        QMutexLocker locker(&m_latestMutex);
        m_latestImage = QImage();
        qDebug() << "已清除缓存图像";
    }
}

// 转换 JSON 为 QByteArray
static QByteArray jsonDataToByteArray(const QJsonValue &dataVal) {
    if (dataVal.isString()) {
        // base64 encoded string (could be compressed image like JPEG)
        QString s = dataVal.toString();
        return QByteArray::fromBase64(s.toUtf8());
    } else if (dataVal.isArray()) {
        QJsonArray arr = dataVal.toArray();
        QByteArray out;
        out.reserve(arr.size());
        for (const QJsonValue &v : arr) {
            int iv = v.toInt();
            out.append(static_cast<char>(iv & 0xFF));
        }
        return out;
    }
    return QByteArray();
}

// 处理接收数据
void CameraImageMonitor::onMessageReceived(const QString &message) {
    QJsonDocument doc = QJsonDocument::fromJson(message.toUtf8());
    if (!doc.isObject()) {
        qDebug() << "CameraImageMonitor: 接收到无效的JSON数据";
        return;
    }
    QJsonObject obj = doc.object();

    if (obj["op"].toString() == "publish") {
        QString topic = obj["topic"].toString();
        if (topic.isEmpty()) {
            qDebug() << "CameraImageMonitor: 接收到的消息没有话题名";
            return;
        }
        
        QJsonObject msgObj = obj["msg"].toObject();
        if (msgObj.isEmpty()) {
            qDebug() << "CameraImageMonitor: 接收到的消息没有内容，话题: " << topic;
            return;
        }

        //qDebug() << "收到话题消息: " << topic;

        // 检查是否是当前订阅的话题
        if (topic != act_topic_name) {
            // 忽略不是当前订阅的话题
            return;
        }

        // compressed image path: 处理压缩图像消息
        if (act_topic_type.contains("CompressedImage")) {
            // sensor_msgs/CompressedImage: has fields 'format' and 'data'
            QString format = msgObj.value("format").toString();
            QJsonValue dataVal = msgObj.value("data");
            QByteArray bytes = jsonDataToByteArray(dataVal);
            if (bytes.isEmpty()) {
                qDebug() << "CameraImageMonitor: 压缩图像数据为空，话题: " << topic;
                return;
            }

            QImage img = QImage::fromData(bytes);
            if (img.isNull()) {
                qDebug() << "CameraImageMonitor: 解码压缩图像失败，格式 = " << format << " 字节数 = " << bytes.size();
                return;
            }

            // throttle and store scaled image in cache (worker thread)
            qint64 elapsed = m_lastDecodeTimer.elapsed();
            if (elapsed < m_frameIntervalMs) return;
            m_lastDecodeTimer.restart();

            QImage toStore;
            if (!m_targetSize.isEmpty() && img.size() != m_targetSize) {
                toStore = img.scaled(m_targetSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
            } else {
                toStore = img;
            }
            // normalize pixel format to avoid rendering artifacts and dangling buffers
            toStore = toStore.convertToFormat(QImage::Format_RGBA8888);
            {
                QMutexLocker locker(&m_latestMutex);
                m_latestImage = toStore;
                qDebug() << "成功处理压缩图像，话题: " << topic << " 尺寸: " << toStore.width() << "x" << toStore.height();
            }
            return;
        } 
        // 处理原始图像消息
        else if (act_topic_type.contains("Image")) {
            int width = msgObj.value("width").toInt();
            int height = msgObj.value("height").toInt();
            QString encoding = msgObj.value("encoding").toString();
            QJsonValue dataVal = msgObj.value("data");
            
            if (width <= 0 || height <= 0) {
                qDebug() << "CameraImageMonitor: 无效的图像尺寸，宽: " << width << " 高: " << height;
                return;
            }
            
            // 将图像数据转为 QByteArray
            QByteArray bytes = jsonDataToByteArray(dataVal);
            if (bytes.isEmpty()) {
                qDebug() << "CameraImageMonitor: 原始图像数据为空";
                return;
            }

            // First try to decode as compressed image (JPEG/PNG) even for raw topic payloads
            QImage img = QImage::fromData(bytes);
            if (!img.isNull()) {
                // throttle by max FPS (avoid excessive decoding)
                qint64 elapsed = m_lastDecodeTimer.elapsed();
                if (elapsed < m_frameIntervalMs) return;
                m_lastDecodeTimer.restart();

                // scale in worker thread if requested and store into latest cache
                QImage toStore;
                if (!m_targetSize.isEmpty() && img.size() != m_targetSize) {
                    toStore = img.scaled(m_targetSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
                } else {
                    toStore = img;
                }
                toStore = toStore.convertToFormat(QImage::Format_RGBA8888);
                {
                    QMutexLocker locker(&m_latestMutex);
                    m_latestImage = toStore;
                    qDebug() << "成功处理压缩格式的原始图像，话题: " << topic << " 尺寸: " << toStore.width() << "x" << toStore.height();
                }
                return;
            }

            // Otherwise interpret as raw pixel buffer. Determine bytes per pixel
            int bpp = 3;
            QImage::Format fmt = QImage::Format_Invalid;
            if (encoding == "mono8" || encoding == "gray" || encoding == "mono") {
                bpp = 1;
                fmt = QImage::Format_Grayscale8;
            } else if (encoding == "rgb8" || encoding == "rgb24") {
                bpp = 3;
                fmt = QImage::Format_RGB888;
            } else if (encoding == "bgr8") {
                bpp = 3;
                // we'll construct as BGR and swap
                fmt = QImage::Format_BGR888;
            } else if (encoding == "rgba8" || encoding == "rgba32") {
                bpp = 4;
                fmt = QImage::Format_RGBA8888;
            } else {
                // fallback assume RGB888
                bpp = 3;
                fmt = QImage::Format_RGB888;
                qDebug() << "CameraImageMonitor: 未知编码格式，默认使用RGB888: " << encoding;
            }

            int expected = width * height * bpp;
            if (bytes.size() < expected) {
                qDebug() << "CameraImageMonitor: 原始缓冲区太小: " << bytes.size() << " 期望: " << expected << " 编码: " << encoding;
                return;
            }

            int bytesPerLine = width * bpp;
            if (fmt == QImage::Format_BGR888) {
                QImage tmp(reinterpret_cast<const uchar*>(bytes.constData()), width, height, bytesPerLine, fmt);
                img = tmp.rgbSwapped().copy();
            } else {
                QImage tmp(reinterpret_cast<const uchar*>(bytes.constData()), width, height, bytesPerLine, fmt);
                img = tmp.copy();
            }

            if (!img.isNull()) {
                qint64 elapsed = m_lastDecodeTimer.elapsed();
                if (elapsed < m_frameIntervalMs) return;
                m_lastDecodeTimer.restart();

                QImage toStore;
                if (!m_targetSize.isEmpty() && img.size() != m_targetSize) {
                    toStore = img.scaled(m_targetSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
                } else {
                    toStore = img;
                }
                toStore = toStore.convertToFormat(QImage::Format_RGBA8888);
                {
                    QMutexLocker locker(&m_latestMutex);
                    m_latestImage = toStore;
                    qDebug() << "成功处理原始图像，话题: " << topic << " 编码: " << encoding << " 尺寸: " << toStore.width() << "x" << toStore.height();
                }
            } else {
                qDebug() << "CameraImageMonitor: 创建图像失败，编码: " << encoding;
            }
        } else {
            qDebug() << "CameraImageMonitor: 未知的消息类型: " << act_topic_type << " 话题: " << topic;
        }
    }
}

void CameraImageMonitor::requestFrame()
{
    QImage snapshot;
    {
        QMutexLocker locker(&m_latestMutex);
        if (m_latestImage.isNull()) return;
        snapshot = m_latestImage;
        // clear to avoid re-sending same frame repeatedly
        m_latestImage = QImage();
    }
    // emit from whichever thread called requestFrame; UI will receive via queued connection
    emit imageReceived(snapshot);
}
