#include "HttpServer.h"

HttpServer::HttpServer()
{
    // Constructor implementation (if needed)
}

void HttpServer::handleRoot()
{
    server.send(200, "text/plain", "Hello, world!");
}

void HttpServer::begin()
{
    server.on("/", std::bind(&HttpServer::handleRoot, this));
    server.begin();
}

void HttpServer::handleClient()
{
    server.handleClient();
}