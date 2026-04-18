#pragma once

#include <Arduino.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>

#include "../common/TransportMode.h"
#include "../rfid/RfidStorage.h"

enum WifiState {
    WIFI_STATE_IDLE,
    WIFI_STATE_AP,
    WIFI_STATE_CONNECTING,
    WIFI_STATE_CONNECTED,
    WIFI_STATE_FALLBACK_AP
};

class WifiManager
{
public:
    explicit WifiManager(RfidStorage *storage);

    void begin();
    void loop();
    void trySendData();

    WifiState     getState()     const { return _state; }
    TransportMode getTransport() const { return _transport; }
    bool          isConnected()  const { return _state == WIFI_STATE_CONNECTED; }

private:
    void _startAP();
    void _stopAP();
    void _startSTA();

    void _handleRoot();
    void _handleSave();
    void _handleReset();
    void _handleNotFound();

    void _loadSettings();
    void _persistSettings();
    void _buildJson(const uint8_t *buf, uint8_t len, char *out, size_t outLen);
    void _doHttpPost();

    RfidStorage  *_storage;
    WebServer     _server{80};
    DNSServer     _dns;
    Preferences   _prefs;

    WifiState     _state        = WIFI_STATE_IDLE;
    TransportMode _transport    = TRANSPORT_BOTH;
    bool          _configured   = false;

    char _ssid[33]      = {0};
    char _pass[65]      = {0};
    char _endpoint[129] = {0};

    unsigned long _connectStartMs = 0;
    static const unsigned long CONNECT_TIMEOUT_MS = 10000;

    char _pendingJson[512] = {0};
    bool _sendPending      = false;
};
