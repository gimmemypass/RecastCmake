#pragma once
#include <stdio.h>
#include <string>
#include<sstream>

typedef void(*FuncCallBack)(const char* message, int size);

class DebugCPP
{
public:
    static void Log(const char* message);
    static void Log(const std::string message);
    static void Log(const int message);
    static void Log(const char message);
    static void Log(const float message);
    static void Log(const double message);
    static void Log(const bool message);
    static FuncCallBack callbackInstance;

private:
    static void send_log(const std::stringstream &ss);
};