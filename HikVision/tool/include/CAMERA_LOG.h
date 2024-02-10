#ifndef _LOG_H_
#define _LOG_H_

#include <fstream>
#include "opencv2/opencv.hpp"

class LOG {
private:
    bool logLevel = true;
    std::ofstream stream;
    double saveTime = (double) cv::getTickCount();

    /**
     * ��������
     */
    static LOG &instance() {
        static LOG logger;
        return logger;
    }

    /**
     * �����־ʱ���
     * @return ʱ���
     */
    static std::string getTime() {
#ifdef _MSC_VER
        time_t time_seconds = time(0);
        static struct tm ltm;
        localtime_s(&ltm, &time_seconds);
        return tostring(ltm.tm_hour) + ":" + tostring(ltm.tm_min) + ":" + tostring(ltm.tm_sec);
#else
        time_t now = time(0);
        tm *ltm = localtime(&now);
        return tostring(ltm->tm_hour) + ":" + tostring(ltm->tm_min) + ":" + tostring(ltm->tm_sec);
#endif
    }

    /**
     * ��ñ���ͼƬ��ʱ���
     * @return ʱ���
     */
    static std::string getSaveImgTime() {
#ifdef _MSC_VER
        time_t time_seconds = time(0);
        struct tm ltm;
        localtime_s(&ltm, &time_seconds);
        return tostring(1 + ltm.tm_mon) + "_" + tostring(ltm.tm_mday) + "_" + tostring(ltm.tm_hour) + "_" + tostring(ltm.tm_min) + "_" + tostring(ltm.tm_sec);
#else
        time_t now = time(0);
        tm *ltm = localtime(&now);
        return tostring(1 + ltm->tm_mon) + "_" + tostring(ltm->tm_mday) + "_" + tostring(ltm->tm_hour) + "_" + tostring(ltm->tm_min) + "_" + tostring(ltm->tm_sec);
#endif
    }

    /**
     * �ж���һ�κ���һ��ʱ�����Ƿ����1��
     * @return
     */
    static bool timeDelay() {
        double time = ((double) cv::getTickCount() - instance().saveTime) / cv::getTickFrequency();
        return time > 1.0f;
    }

public:

    ~LOG() {
        stream.close();
    }

    /**
     * ������־�洢λ�úͱ���ͼƬλ��
     * @param logPath ��־�洢λ��
     * @param saveImgPath ����ͼƬλ��
     */
    static void setDestination(const std::string &logPath) {
        std::cout << "Save log to : " << logPath << std::endl;

        //����־�洢�ļ�
        instance().stream.open(logPath, std::ios::out);
#ifdef _MSC_VER
        time_t time_seconds = time(0);
        struct tm ltm;
        localtime_s(&ltm, &time_seconds);
        info("Now time : " + tostring(1900 + ltm.tm_year) + "." + tostring(1 + ltm.tm_mon) + "." + tostring(ltm.tm_mday) + "  " + tostring(ltm.tm_hour) + ":" + tostring(ltm.tm_min) + ":" + tostring(ltm.tm_sec));
#else
        time_t now = time(0);
        tm *ltm = localtime(&now);
        info("Now time : " + tostring(1900 + ltm->tm_year) + "." + tostring(1 + ltm->tm_mon) + "." + tostring(ltm->tm_mday) + "  " + tostring(ltm->tm_hour) + ":" + tostring(ltm->tm_min));
#endif
    }

    /**
     * ������־����ȱʡΪinfo����ѡdebug
     * @param level ��־����
     */
    static void setLevel(const char *level) {
        if (level == (char*)"debug") {
            instance().logLevel = false;
        } else {
            instance().logLevel = true;
        }
    }

    /**
     * �ر���־
     */
    static void close() {
        instance().stream.close();
    }

    /**
     * ��ӡinfo��־
     * @param log ��־����
     */
    static void info(const char *log) {
        std::string _log = "[INFO " + getTime() + "]: ";
        _log.append(log).append("\n");
        instance().stream << _log;
        std::cout << _log;
    }

    /**
     * ��ӡinfo��־
     * @param log ��־����
     */
    static void info(const std::string &log) {
        LOG::info(log.c_str());
    }

    /**
     * ��ӡdebug��־
     * @param log ��־����
     */
    static void debug(const char *log) {
        if (!instance().logLevel) {
            std::string _log = "[DEBUG " + getTime() + "]: ";
            _log.append(log).append("\n");
            instance().stream << _log;
            std::cout << _log;
        }
    }

    /**
     * ��ӡdebug��־
     * @param log ��־����
     */
    static void debug(const std::string &log) {
        LOG::debug(log.c_str());
    }

    /**
     * ��ӡerror��־
     * @param log ��־����
     */
    static void error(const char *log) {
        std::string _log = "[ERROR " + getTime() + "]: ";
        _log.append(log).append("\n");
        instance().stream << _log;
        std::cout << _log;
    }

    /**
     * ��ӡerror��־
     * @param log ��־����
     */
    static void error(const std::string &log) {
        LOG::error(log.c_str());
    }

    static std::string tostring(int i) {
        return std::to_string(i);
    }

    static std::string tostring(double i) {
        return std::to_string(i);
    }

    static std::string tostring(long i) {
        return std::to_string(i);
    }

    static std::string tostring(float i) {
        return std::to_string(i);
    }

    static std::string tostring(unsigned i) {
        return std::to_string(i);
    }

    static std::string tostring(bool i) {
        return std::to_string(i);
    }

    static std::string tostring(const char *i) {
        return std::string(i);
    }
};

#endif
