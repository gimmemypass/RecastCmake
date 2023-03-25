#include "Debug.h"

#include <string>
#include <stdio.h>
#include <stdio.h>
#include <sstream>
#include <cstring>


FuncCallBack DebugCPP::callbackInstance = nullptr;
//-------------------------------------------------------------------
void  DebugCPP::Log(const char* message) {
    if (callbackInstance != nullptr)
        callbackInstance(message, (int)strlen(message));
}

void  DebugCPP::Log(const std::string message) {
    const char* tmsg = message.c_str();
    if (callbackInstance != nullptr)
        callbackInstance(tmsg, (int)strlen(tmsg));
}

void  DebugCPP::Log(const int message) {
    std::stringstream ss;
    ss << message;
    send_log(ss);
}

void  DebugCPP::Log(const char message) {
    std::stringstream ss;
    ss << message;
    send_log(ss);
}

void  DebugCPP::Log(const float message) {
    std::stringstream ss;
    ss << message;
    send_log(ss);
}

void  DebugCPP::Log(const double message) {
    std::stringstream ss;
    ss << message;
    send_log(ss);
}

void DebugCPP::Log(const bool message) {
    std::stringstream ss;
    if (message)
        ss << "true";
    else
        ss << "false";

    send_log(ss);
}

void DebugCPP::send_log(const std::stringstream &ss) {
    const std::string tmp = ss.str();
    const char* tmsg = tmp.c_str();
    if (callbackInstance != nullptr)
        callbackInstance(tmsg, (int)strlen(tmsg));
}
