#pragma once
#include <Arduino.h>
#include <WebServer.h>

class HttpServer
{
public:
    HttpServer();
    void begin();
    void handleClient();

private:
    WebServer server;
    void handleRoot();
};