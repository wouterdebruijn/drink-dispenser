#include "WifiManager.h"

#include <HTTPClient.h>
#include <esp_wifi.h>

// ---------------------------------------------------------------------------
// Admin page HTML (stored in flash)
// ---------------------------------------------------------------------------
static const char ADMIN_HTML[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <meta charset="utf-8">
  <title>Shot Machine Config</title>
  <style>
    body{font-family:sans-serif;max-width:440px;margin:30px auto;padding:0 16px;color:#222}
    h1{font-size:1.3em;margin-bottom:4px}
    h2{font-size:1em;border-bottom:1px solid #ddd;padding-bottom:6px;margin-top:24px}
    label{display:block;font-size:.85em;margin-bottom:2px;color:#555}
    input,select{width:100%;padding:8px;margin-bottom:14px;box-sizing:border-box;border:1px solid #ccc;border-radius:4px}
    button{width:100%;padding:10px;margin-top:4px;background:#c0392b;color:#fff;border:0;border-radius:4px;cursor:pointer;font-size:1em}
    button.secondary{background:#2980b9}
    .status{background:#f0f0f0;padding:8px;border-radius:4px;font-size:.85em;margin-bottom:16px}
  </style>
</head>
<body>
  <h1>Shot Machine v4</h1>
  <div class="status">Transport: <b>%TRANSPORT_LABEL%</b> &nbsp;|&nbsp; WiFi: <b>%WIFI_STATUS%</b></div>

  <h2>WiFi &amp; Transport</h2>
  <form method="POST" action="/save">
    <label>SSID</label>
    <input name="ssid" type="text" maxlength="32" value="%SSID%" autocomplete="off">
    <label>Password</label>
    <input name="pass" type="password" maxlength="64" placeholder="(unchanged)" autocomplete="new-password">
    <label>HTTP Endpoint URL</label>
    <input name="endpoint" type="text" maxlength="128" value="%ENDPOINT%" placeholder="http://server/api/device">
    <label>Device Secret</label>
    <input name="secret" type="password" maxlength="64" placeholder="(unchanged)" autocomplete="new-password">
    <label>Transport mode</label>
    <select name="transport">
      <option value="0"%SEL_LORA%>LoRaWAN only</option>
      <option value="1"%SEL_WIFI%>WiFi only</option>
      <option value="2"%SEL_BOTH%>Both (WiFi preferred)</option>
    </select>
    <button class="secondary" type="submit">Save &amp; Reboot</button>
  </form>

  <h2>Danger Zone</h2>
  <form method="POST" action="/reset"
        onsubmit="return confirm('Reset ALL shot counts? This cannot be undone.')">
    <button type="submit">Reset Shot Counts</button>
  </form>
</body>
</html>)rawhtml";

static const char RESET_OK_HTML[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Reset</title></head>
<body><p>Shot counts reset. <a href="/">Back</a></p></body></html>)rawhtml";

// ---------------------------------------------------------------------------

WifiManager::WifiManager(RfidStorage *storage)
    : _storage(storage)
{
}

void WifiManager::begin()
{
    _loadSettings();

    // Initialise the WiFi driver and TCP/IP stack unconditionally with
    // WIFI_STA first — this is the path that reliably calls _init() →
    // tcpip_adapter_init() on ESP32 Arduino 2.x / IDF 4.4.x.
    // Switching to AP mode afterwards (in _startAP) is safe once init is done.
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    delay(150);

    // Register web server routes using lambdas
    _server.on("/",      HTTP_GET,  [this]() { _handleRoot(); });
    _server.on("/save",  HTTP_POST, [this]() { _handleSave(); });
    _server.on("/reset", HTTP_POST, [this]() { _handleReset(); });
    _server.onNotFound(            [this]() { _handleNotFound(); });
    _server.begin();

    if (!_configured)
    {
        Serial.println("WifiManager: no credentials saved, starting AP");
        _startAP();
    }
    else
    {
        _startSTA();
        _state = WIFI_STATE_CONNECTING;
        _connectStartMs = millis();
    }
}

void WifiManager::loop()
{
    if (_state == WIFI_STATE_AP || _state == WIFI_STATE_FALLBACK_AP)
        _dns.processNextRequest();
    _server.handleClient();

    switch (_state)
    {
    case WIFI_STATE_CONNECTING:
        if (WiFi.status() == WL_CONNECTED)
        {
            Serial.printf("WifiManager: connected to %s, IP %s\n",
                          _ssid, WiFi.localIP().toString().c_str());
            _stopAP();
            _state = WIFI_STATE_CONNECTED;
        }
        else if (millis() - _connectStartMs > CONNECT_TIMEOUT_MS)
        {
            Serial.println("WifiManager: connect timeout, falling back to AP");
            _startAP();
            _state = WIFI_STATE_FALLBACK_AP;
        }
        break;

    case WIFI_STATE_CONNECTED:
        if (WiFi.status() != WL_CONNECTED)
        {
            Serial.println("WifiManager: connection lost, reconnecting");
            _state = WIFI_STATE_CONNECTING;
            _connectStartMs = millis();
            WiFi.reconnect();
        }
        break;

    default:
        break;
    }

    if (_sendPending)
        _doHttpPost();
}

void WifiManager::trySendData()
{
    if (_transport == TRANSPORT_LORA)
        return;
    if (_state != WIFI_STATE_CONNECTED)
        return;
    if (_sendPending)
        return; // previous send still in flight

    static uint8_t buffer[RFID_MAX_TAGS * 4];
    uint8_t len = _storage->dumpTagStorage(buffer);
    if (len == 0)
        return;

    _buildJson(buffer, len, _pendingJson, sizeof(_pendingJson));
    _sendPending = true;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void WifiManager::_startAP()
{
    // Stack is already initialised (WIFI_STA was set in begin()).
    // Switch to AP mode and start the captive portal.
    WiFi.mode(WIFI_AP);
    delay(100);
    WiFi.softAP("ShotMachine-Config");
    _dns.start(53, "*", IPAddress(192, 168, 4, 1));
    Serial.println("WifiManager: AP started — SSID: ShotMachine-Config, IP: 192.168.4.1");
    _state = (_state == WIFI_STATE_CONNECTING) ? WIFI_STATE_FALLBACK_AP : WIFI_STATE_AP;
}

void WifiManager::_stopAP()
{
    _dns.stop();
    WiFi.softAPdisconnect(true);
    delay(50);
    WiFi.mode(WIFI_STA);
}

void WifiManager::_startSTA()
{
    // Mode is already WIFI_STA from begin(); just start the connection.
    WiFi.begin(_ssid, _pass);
    Serial.printf("WifiManager: connecting to %s\n", _ssid);
}

void WifiManager::_doHttpPost()
{
    if (_endpoint[0] == '\0')
    {
        Serial.println("WifiManager: no endpoint configured, skipping send");
        _sendPending = false;
        return;
    }

    HTTPClient http;
    http.begin(_endpoint);
    http.addHeader("Content-Type", "application/json");
    if (_secret[0] != '\0')
    {
        char authHeader[80];
        snprintf(authHeader, sizeof(authHeader), "Bearer %s", _secret);
        http.addHeader("Authorization", authHeader);
    }
    http.setTimeout(3000);

    Serial.printf("WifiManager: POST %s\n", _endpoint);
    int code = http.POST(_pendingJson);

    if (code == 200 || code == 201 || code == 204)
    {
        Serial.printf("WifiManager: POST success (%d)\n", code);
        _storage->clearChangedTags();
    }
    else
    {
        Serial.printf("WifiManager: POST failed (%d)\n", code);
    }

    http.end();
    _sendPending = false;
}

void WifiManager::_buildJson(const uint8_t *buf, uint8_t len,
                              char *out, size_t outLen)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    char macSuffix[8];
    snprintf(macSuffix, sizeof(macSuffix), "%02X%02X%02X", mac[3], mac[4], mac[5]);

    String json = "{\"device\":\"ShotMachine-";
    json += macSuffix;
    json += "\",\"tags\":[";

    bool first = true;
    for (int i = 0; i + 3 < len; i += 4)
    {
        uint16_t id    = ((uint16_t)buf[i]   << 8) | buf[i + 1];
        uint16_t count = ((uint16_t)buf[i + 2] << 8) | buf[i + 3];
        if (!first) json += ",";
        json += "{\"id\":";
        json += id;
        json += ",\"count\":";
        json += count;
        json += "}";
        first = false;
    }
    json += "]}";

    strlcpy(out, json.c_str(), outLen);
}

// ---------------------------------------------------------------------------
// NVS
// ---------------------------------------------------------------------------

void WifiManager::_loadSettings()
{
    _prefs.begin("wifi_cfg", false); // read-write so namespace is created on first boot
    String ssid     = _prefs.isKey("ssid")      ? _prefs.getString("ssid")     : "";
    String pass     = _prefs.isKey("pass")      ? _prefs.getString("pass")     : "";
    String endpoint = _prefs.isKey("endpoint")  ? _prefs.getString("endpoint") : "";
    String secret   = _prefs.isKey("secret")    ? _prefs.getString("secret")   : "";
    _transport      = (TransportMode)_prefs.getUChar("transport", TRANSPORT_BOTH);
    _configured     = _prefs.getBool("configured", false);
    _prefs.end();

    strlcpy(_ssid,     ssid.c_str(),     sizeof(_ssid));
    strlcpy(_pass,     pass.c_str(),     sizeof(_pass));
    strlcpy(_endpoint, endpoint.c_str(), sizeof(_endpoint));
    strlcpy(_secret,   secret.c_str(),   sizeof(_secret));
}

void WifiManager::_persistSettings()
{
    _prefs.begin("wifi_cfg", false);
    _prefs.putString("ssid",       _ssid);
    _prefs.putString("pass",       _pass);
    _prefs.putString("endpoint",   _endpoint);
    _prefs.putString("secret",     _secret);
    _prefs.putUChar("transport",   (uint8_t)_transport);
    _prefs.putBool("configured",   true);
    _prefs.end();
}

// ---------------------------------------------------------------------------
// Web handlers
// ---------------------------------------------------------------------------

void WifiManager::_handleRoot()
{
    // Build the page from PROGMEM template, replacing placeholders
    String page = FPSTR(ADMIN_HTML);

    page.replace("%SSID%",      String(_ssid));
    page.replace("%ENDPOINT%",  String(_endpoint));

    const char *wifiStatus = "—";
    if (_state == WIFI_STATE_CONNECTED)          wifiStatus = "Connected";
    else if (_state == WIFI_STATE_CONNECTING)    wifiStatus = "Connecting...";
    else if (_state == WIFI_STATE_AP ||
             _state == WIFI_STATE_FALLBACK_AP)   wifiStatus = "AP mode";

    const char *transportLabel = "Both";
    if (_transport == TRANSPORT_LORA) transportLabel = "LoRaWAN only";
    else if (_transport == TRANSPORT_WIFI) transportLabel = "WiFi only";

    page.replace("%TRANSPORT_LABEL%", transportLabel);
    page.replace("%WIFI_STATUS%",     wifiStatus);
    page.replace("%SEL_LORA%", _transport == TRANSPORT_LORA ? " selected" : "");
    page.replace("%SEL_WIFI%", _transport == TRANSPORT_WIFI ? " selected" : "");
    page.replace("%SEL_BOTH%", _transport == TRANSPORT_BOTH ? " selected" : "");

    _server.send(200, "text/html", page);
}

void WifiManager::_handleSave()
{
    String ssid      = _server.arg("ssid");
    String pass      = _server.arg("pass");
    String endpoint  = _server.arg("endpoint");
    String secret    = _server.arg("secret");
    String transport = _server.arg("transport");

    strlcpy(_ssid, ssid.c_str(), sizeof(_ssid));

    // Only update password/secret if a new value was provided
    if (pass.length() > 0)
        strlcpy(_pass, pass.c_str(), sizeof(_pass));
    if (secret.length() > 0)
        strlcpy(_secret, secret.c_str(), sizeof(_secret));

    strlcpy(_endpoint, endpoint.c_str(), sizeof(_endpoint));
    _transport = (TransportMode)transport.toInt();

    _persistSettings();

    _server.send(200, "text/html",
                 "<html><body><p>Saved. Rebooting...</p></body></html>");
    delay(500);
    ESP.restart();
}

void WifiManager::_handleReset()
{
    _storage->reset();
    _server.send(200, "text/html", FPSTR(RESET_OK_HTML));
}

void WifiManager::_handleNotFound()
{
    // Captive portal redirect
    _server.sendHeader("Location", "http://192.168.4.1/", true);
    _server.send(302, "text/plain", "");
}
