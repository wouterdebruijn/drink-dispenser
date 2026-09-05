#include "HttpServer.h"

HttpServer::HttpServer(Preferences *systemPrefs)
    : server(80), _systemPrefs(systemPrefs)
{
}

void HttpServer::handleRoot()
{
    String currentSsid = _systemPrefs->getString("wifiSsid", "");

    String page = F("<!doctype html><html><head><meta name=viewport "
                    "content=\"width=device-width,initial-scale=1\">"
                    "<title>Shot Machine Setup</title></head><body>"
                    "<h2>WiFi Setup</h2>"
                    "<form method=POST action=/save>"
                    "<p>SSID:<br><input name=ssid value=\"");
    page += currentSsid;
    page += F("\"></p>"
              "<p>Password:<br><input name=password type=password></p>"
              "<p><button type=submit>Save</button></p>"
              "</form></body></html>");

    server.send(200, "text/html", page);
}

void HttpServer::handleSave()
{
    String ssid = server.arg("ssid");
    String password = server.arg("password");

    _systemPrefs->putString("wifiSsid", ssid);
    _systemPrefs->putString("wifiPassword", password);

    String page = F("<!doctype html><html><head><meta name=viewport "
                    "content=\"width=device-width,initial-scale=1\">"
                    "<title>Saved</title></head><body>"
                    "<h2>Saved</h2>"
                    "<p>Network \"");
    page += ssid;
    page += F("\" stored. Switch the device to WiFi mode from the menu and "
              "reboot to connect.</p>"
              "<p><a href=/>Back</a></p></body></html>");

    server.send(200, "text/html", page);
}

void HttpServer::begin()
{
    server.on("/", HTTP_GET, std::bind(&HttpServer::handleRoot, this));
    server.on("/save", HTTP_POST, std::bind(&HttpServer::handleSave, this));
    server.begin();
}

void HttpServer::handleClient()
{
    server.handleClient();
}
