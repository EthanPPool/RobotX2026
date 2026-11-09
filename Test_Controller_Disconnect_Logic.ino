#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Bluepad32.h>

// ============================================================
// ROBOTX26 ESP32-S3 MANUAL BLUEBOAT CONTROLLER
//
// CONTROLS
//
// A  = MANUAL / HOLD
// B  = SOFTWARE E-STOP TOGGLE
// X  = ARM / DISARM
// LB = DEADMAN
//
// Left Stick Y  = Forward / Reverse
// Right Stick X = Steering
//
// AUTHORITATIVE STATE:
//
// Jetson telemetry:
//   software_stop
//   armed
//
// BlueOS:
//   mode commands
//   ARM/DISARM MAVLink
//   MANUAL_CONTROL
//
// CONTROLLER FAILSAFE:
//
// Xbox disconnected / absent:
//   -> revoke manual authority
//   -> stop MANUAL_CONTROL
//   -> request HOLD
//   -> ENGAGE software E-stop
//
// E-stop remains engaged after controller reconnect.
// Operator must press B to clear it.
//
// ============================================================


// ============================================================
// WIFI
// ============================================================

const char* WIFI_SSID =
  "BaseStation - 2.4Ghz";

const char* WIFI_PASSWORD =
  "BaseStation";


IPAddress ESP32_IP(
  192,
  168,
  2,
  30
);


IPAddress NO_GATEWAY(
  0,
  0,
  0,
  0
);


IPAddress SUBNET(
  255,
  255,
  255,
  0
);


// ============================================================
// BLUEOS
// ============================================================

IPAddress BLUEOS_IP(
  192,
  168,
  2,
  2
);


const uint16_t MAVLINK2REST_PORT =
  6040;


// ============================================================
// JETSON
// ============================================================

IPAddress JETSON_IP(
  192,
  168,
  2,
  20
);


const uint16_t JETSON_COMMAND_PORT =
  8765;


const unsigned long JETSON_RESPONSE_TIMEOUT_MS =
  1500;


const unsigned long JETSON_TELEMETRY_TIMEOUT_MS =
  1500;


// ============================================================
// MAVLINK
// ============================================================

const uint8_t TARGET_SYSTEM =
  1;


const uint8_t TARGET_COMPONENT =
  1;


const uint8_t GCS_SYSTEM_ID =
  255;


const uint8_t GCS_COMPONENT_ID =
  190;


// ArduRover modes
const uint8_t ROVER_MODE_MANUAL =
  0;


const uint8_t ROVER_MODE_HOLD =
  4;


// MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
const uint8_t MAV_MODE_CUSTOM =
  1;


// ============================================================
// CONTROL SETTINGS
// ============================================================

const unsigned long MANUAL_SEND_PERIOD_MS =
  50;


const unsigned long WIFI_RECONNECT_MS =
  5000;


const unsigned long DEBUG_PERIOD_MS =
  1000;


const unsigned long ARM_VERIFY_TIMEOUT_MS =
  3000;


// ------------------------------------------------------------
// CONTROLLER FAILSAFE
//
// While controller is absent, periodically verify that
// the software E-stop remains engaged.
//
// ------------------------------------------------------------

const unsigned long CONTROLLER_FAILSAFE_CHECK_MS =
  2000;


const float AXIS_MAX =
  512.0f;


const float DEADZONE =
  0.08f;


const int16_t MAV_CONTROL_MAX =
  1000;


const int16_t MAV_AXIS_INVALID =
  32767;


// ============================================================
// GLOBALS
// ============================================================

ControllerPtr xboxController =
  nullptr;


// ------------------------------------------------------------
// MANUAL MODE
// ------------------------------------------------------------

bool manualModeEnabled =
  false;


// ------------------------------------------------------------
// AUTHORITATIVE JETSON STATE
// ------------------------------------------------------------

bool jetsonStateKnown =
  false;


bool jetsonSoftwareStopKnown =
  false;


bool jetsonSoftwareStopEngaged =
  true;


bool jetsonArmedKnown =
  false;


bool jetsonArmed =
  false;


String jetsonMode =
  "UNKNOWN";


// ------------------------------------------------------------
// BUTTON EDGES
// ------------------------------------------------------------

bool previousAButtonState =
  false;


bool previousBButtonState =
  false;


bool previousXButtonState =
  false;


// ------------------------------------------------------------
// TIMERS
// ------------------------------------------------------------

unsigned long lastManualSend =
  0;


unsigned long lastWiFiReconnect =
  0;


unsigned long lastDebug =
  0;


unsigned long lastControllerFailsafeCheck =
  0;


// ------------------------------------------------------------
// MAVLINK
// ------------------------------------------------------------

uint8_t mavSequence =
  0;


String mavlinkApiPath =
  "/v1/mavlink";


bool mavlinkApiConfirmed =
  false;


// ------------------------------------------------------------
// JETSON REQUEST IDS
// ------------------------------------------------------------

uint32_t jetsonRequestCounter =
  0;


// ============================================================
// JOYSTICK NORMALIZATION
// ============================================================

float normalizeAxis(
  int raw
) {

  float value =
    (float)raw
    /
    AXIS_MAX;


  value =
    constrain(
      value,
      -1.0f,
      1.0f
    );


  if (
    fabs(value)
    <
    DEADZONE
  ) {

    return 0.0f;
  }


  if (
    value > 0.0f
  ) {

    value =
      (value - DEADZONE)
      /
      (1.0f - DEADZONE);

  } else {

    value =
      (value + DEADZONE)
      /
      (1.0f - DEADZONE);
  }


  return constrain(
    value,
    -1.0f,
    1.0f
  );
}


// ============================================================
// XBOX CONNECTION STATUS
// ============================================================

bool xboxIsConnected() {

  return (
    xboxController != nullptr
    &&
    xboxController->isConnected()
    &&
    xboxController->isGamepad()
  );
}


// ============================================================
// JSON STRING FIELD
// ============================================================

String extractJsonStringField(
  const String& json,
  const char* fieldName
) {

  String key =
    "\"";


  key +=
    fieldName;


  key +=
    "\"";


  int keyPos =
    json.indexOf(
      key
    );


  if (
    keyPos < 0
  ) {

    return "";
  }


  int colon =
    json.indexOf(
      ':',
      keyPos + key.length()
    );


  if (
    colon < 0
  ) {

    return "";
  }


  int firstQuote =
    json.indexOf(
      '"',
      colon + 1
    );


  if (
    firstQuote < 0
  ) {

    return "";
  }


  int secondQuote =
    firstQuote + 1;


  while (
    secondQuote
    <
    json.length()
  ) {

    if (
      json[secondQuote] == '"'
      &&
      json[secondQuote - 1] != '\\'
    ) {

      break;
    }


    secondQuote++;
  }


  if (
    secondQuote
    >=
    json.length()
  ) {

    return "";
  }


  return json.substring(
    firstQuote + 1,
    secondQuote
  );
}


// ============================================================
// JSON BOOLEAN FIELD
// ============================================================

bool extractJsonBoolField(
  const String& json,
  const char* fieldName,
  bool &value
) {

  String key =
    "\"";


  key +=
    fieldName;


  key +=
    "\"";


  int keyPos =
    json.indexOf(
      key
    );


  if (
    keyPos < 0
  ) {

    return false;
  }


  int colon =
    json.indexOf(
      ':',
      keyPos + key.length()
    );


  if (
    colon < 0
  ) {

    return false;
  }


  int pos =
    colon + 1;


  while (
    pos < json.length()

    &&

    (
      json[pos] == ' '
      ||
      json[pos] == '\t'
      ||
      json[pos] == '\r'
      ||
      json[pos] == '\n'
    )
  ) {

    pos++;
  }


  if (
    json.substring(
      pos,
      pos + 4
    )
    ==
    "true"
  ) {

    value =
      true;


    return true;
  }


  if (
    json.substring(
      pos,
      pos + 5
    )
    ==
    "false"
  ) {

    value =
      false;


    return true;
  }


  return false;
}


// ============================================================
// CONNECT BASESTATION
// ============================================================

bool connectToBaseStation() {

  Serial.println();

  Serial.println(
    "===================================="
  );

  Serial.println(
    "CONNECTING TO BLUEBOAT BASESTATION"
  );

  Serial.println(
    "===================================="
  );


  WiFi.disconnect(
    false,
    false
  );


  delay(
    200
  );


  WiFi.mode(
    WIFI_STA
  );


  WiFi.setMinSecurity(
    WIFI_AUTH_WPA_PSK
  );


  WiFi.config(
    ESP32_IP,
    NO_GATEWAY,
    SUBNET
  );


  WiFi.begin(
    WIFI_SSID,
    WIFI_PASSWORD
  );


  unsigned long start =
    millis();


  while (
    WiFi.status()
      !=
      WL_CONNECTED

    &&

    millis() - start
      <
      20000
  ) {

    Serial.print(
      "."
    );


    delay(
      500
    );
  }


  Serial.println();


  if (
    WiFi.status()
    !=
    WL_CONNECTED
  ) {

    Serial.println(
      "BASESTATION CONNECTION FAILED"
    );


    return false;
  }


  Serial.println(
    "BASESTATION CONNECTED"
  );


  Serial.print(
    "ESP32 IP: "
  );


  Serial.println(
    WiFi.localIP()
  );


  Serial.print(
    "RSSI: "
  );


  Serial.print(
    WiFi.RSSI()
  );


  Serial.println(
    " dBm"
  );


  return true;
}


// ============================================================
// BLUEOS URL
// ============================================================

String makeBlueOSURL(
  const String& path
) {

  String url =
    "http://";


  url +=
    BLUEOS_IP.toString();


  url +=
    ":";


  url +=
    String(
      MAVLINK2REST_PORT
    );


  url +=
    path;


  return url;
}


// ============================================================
// BLUEOS TEST
// ============================================================

bool testBlueOSConnection() {

  if (
    WiFi.status()
    !=
    WL_CONNECTED
  ) {

    return false;
  }


  WiFiClient client;

  HTTPClient http;


  String url =
    makeBlueOSURL(
      "/info"
    );


  http.setTimeout(
    1000
  );


  if (
    !http.begin(
      client,
      url
    )
  ) {

    return false;
  }


  int code =
    http.GET();


  Serial.print(
    "BlueOS HTTP response: "
  );


  Serial.println(
    code
  );


  http.end();


  return (
    code >= 200
    &&
    code < 300
  );
}


// ============================================================
// READ AUTHORITATIVE JETSON TELEMETRY
// ============================================================

bool syncJetsonState() {

  jetsonStateKnown =
    false;


  if (
    WiFi.status()
    !=
    WL_CONNECTED
  ) {

    return false;
  }


  WiFiClient client;


  client.setTimeout(
    250
  );


  if (
    !client.connect(
      JETSON_IP,
      JETSON_COMMAND_PORT
    )
  ) {

    Serial.println(
      "Jetson telemetry connection failed"
    );


    return false;
  }


  unsigned long start =
    millis();


  while (
    millis() - start
    <
    JETSON_TELEMETRY_TIMEOUT_MS
  ) {

    BP32.update();


    if (
      !client.available()
    ) {

      delay(
        5
      );


      continue;
    }


    String line =
      client.readStringUntil(
        '\n'
      );


    line.trim();


    if (
      line.length()
      ==
      0
    ) {

      continue;
    }


    String compact =
      line;


    compact.replace(
      " ",
      ""
    );


    if (
      compact.indexOf(
        "\"type\":\"telemetry\""
      )
      <
      0
    ) {

      continue;
    }


    // --------------------------------------------------------
    // SOFTWARE STOP
    // --------------------------------------------------------

    String stopState =
      extractJsonStringField(
        line,
        "software_stop"
      );


    if (
      stopState.length()
      >
      0
    ) {

      stopState.toUpperCase();


      if (
        stopState
        ==
        "ENGAGED"
      ) {

        jetsonSoftwareStopKnown =
          true;


        jetsonSoftwareStopEngaged =
          true;

      } else if (
        stopState
        ==
        "CLEARED"
      ) {

        jetsonSoftwareStopKnown =
          true;


        jetsonSoftwareStopEngaged =
          false;

      } else {

        jetsonSoftwareStopKnown =
          false;
      }
    }


    // --------------------------------------------------------
    // ARMED
    // --------------------------------------------------------

    bool armedValue =
      false;


    if (
      extractJsonBoolField(
        line,
        "armed",
        armedValue
      )
    ) {

      jetsonArmedKnown =
        true;


      jetsonArmed =
        armedValue;

    } else {

      jetsonArmedKnown =
        false;
    }


    // --------------------------------------------------------
    // MODE
    // --------------------------------------------------------

    String modeValue =
      extractJsonStringField(
        line,
        "mode"
      );


    if (
      modeValue.length()
      >
      0
    ) {

      jetsonMode =
        modeValue;
    }


    jetsonStateKnown =
      jetsonSoftwareStopKnown
      &&
      jetsonArmedKnown;


    Serial.println();

    Serial.println(
      "----- JETSON STATE SYNC -----"
    );


    Serial.print(
      "Software stop: "
    );


    if (
      jetsonSoftwareStopKnown
    ) {

      Serial.println(
        jetsonSoftwareStopEngaged
          ?
          "ENGAGED"
          :
          "CLEARED"
      );

    } else {

      Serial.println(
        "UNKNOWN"
      );
    }


    Serial.print(
      "Armed: "
    );


    if (
      jetsonArmedKnown
    ) {

      Serial.println(
        jetsonArmed
          ?
          "YES"
          :
          "NO"
      );

    } else {

      Serial.println(
        "UNKNOWN"
      );
    }


    Serial.print(
      "Mode: "
    );


    Serial.println(
      jetsonMode
    );


    Serial.println(
      "-----------------------------"
    );


    client.stop();


    return jetsonStateKnown;
  }


  client.stop();


  Serial.println(
    "Jetson telemetry sync timed out"
  );


  return false;
}


// ============================================================
// POST MAVLINK
// ============================================================

int postMavlinkToPath(
  const String& path,
  const String& body,
  bool verbose
) {

  if (
    WiFi.status()
    !=
    WL_CONNECTED
  ) {

    return -100;
  }


  WiFiClient client;

  HTTPClient http;


  String url =
    makeBlueOSURL(
      path
    );


  http.setTimeout(
    300
  );


  if (
    !http.begin(
      client,
      url
    )
  ) {

    return -101;
  }


  http.addHeader(
    "Content-Type",
    "application/json"
  );


  int code =
    http.POST(
      body
    );


  if (
    verbose
    ||
    code < 200
    ||
    code >= 300
  ) {

    Serial.print(
      "MAVLink HTTP "
    );


    Serial.print(
      path
    );


    Serial.print(
      " -> "
    );


    Serial.println(
      code
    );


    if (
      code > 0
    ) {

      String response =
        http.getString();


      if (
        response.length()
        >
        0
      ) {

        Serial.println(
          response
        );
      }
    }
  }


  http.end();


  return code;
}


// ============================================================
// POST MAVLINK WITH FALLBACK
// ============================================================

bool postMavlink(
  const String& body,
  bool verbose
) {

  int code =
    postMavlinkToPath(
      mavlinkApiPath,
      body,
      verbose
    );


  if (
    code >= 200
    &&
    code < 300
  ) {

    mavlinkApiConfirmed =
      true;


    return true;
  }


  if (
    !mavlinkApiConfirmed
  ) {

    String alternatePath;


    if (
      mavlinkApiPath
      ==
      "/v1/mavlink"
    ) {

      alternatePath =
        "/mavlink";

    } else {

      alternatePath =
        "/v1/mavlink";
    }


    code =
      postMavlinkToPath(
        alternatePath,
        body,
        verbose
      );


    if (
      code >= 200
      &&
      code < 300
    ) {

      mavlinkApiPath =
        alternatePath;


      mavlinkApiConfirmed =
        true;


      return true;
    }
  }


  return false;
}


// ============================================================
// ROVER MODE REQUEST
// ============================================================

bool requestRoverMode(
  uint8_t roverMode
) {

  String body;


  body.reserve(
    420
  );


  body =
    "{";


  body +=
    "\"header\":{";


  body +=
    "\"system_id\":";


  body +=
    String(
      GCS_SYSTEM_ID
    );


  body +=
    ",\"component_id\":";


  body +=
    String(
      GCS_COMPONENT_ID
    );


  body +=
    ",\"sequence\":";


  body +=
    String(
      mavSequence++
    );


  body +=
    "},";


  body +=
    "\"message\":{";


  body +=
    "\"type\":\"COMMAND_LONG\",";


  body +=
    "\"param1\":";


  body +=
    String(
      MAV_MODE_CUSTOM
    );


  body +=
    ".0,";


  body +=
    "\"param2\":";


  body +=
    String(
      roverMode
    );


  body +=
    ".0,";


  body +=
    "\"param3\":0.0,";


  body +=
    "\"param4\":0.0,";


  body +=
    "\"param5\":0.0,";


  body +=
    "\"param6\":0.0,";


  body +=
    "\"param7\":0.0,";


  body +=
    "\"command\":{";


  body +=
    "\"type\":\"MAV_CMD_DO_SET_MODE\"";


  body +=
    "},";


  body +=
    "\"target_system\":";


  body +=
    String(
      TARGET_SYSTEM
    );


  body +=
    ",\"target_component\":";


  body +=
    String(
      TARGET_COMPONENT
    );


  body +=
    ",\"confirmation\":0";


  body +=
    "}";


  body +=
    "}";


  Serial.println();


  Serial.println(
    roverMode == ROVER_MODE_MANUAL
      ?
      "REQUESTING ARDUROVER MANUAL MODE"
      :
      "REQUESTING ARDUROVER HOLD MODE"
  );


  bool result =
    postMavlink(
      body,
      true
    );


  Serial.print(
    "Mode request transport: "
  );


  Serial.println(
    result
      ?
      "SUCCESS"
      :
      "FAILED"
  );


  return result;
}


// ============================================================
// MANUAL CONTROL
//
// Rover:
// Y = steering
// Z = throttle
//
// ============================================================

bool sendManualControl(
  int16_t steering,
  int16_t throttle
) {

  steering =
    constrain(
      steering,
      -MAV_CONTROL_MAX,
      MAV_CONTROL_MAX
    );


  throttle =
    constrain(
      throttle,
      -MAV_CONTROL_MAX,
      MAV_CONTROL_MAX
    );


  String body;


  body.reserve(
    520
  );


  body =
    "{";


  body +=
    "\"header\":{";


  body +=
    "\"system_id\":";


  body +=
    String(
      GCS_SYSTEM_ID
    );


  body +=
    ",\"component_id\":";


  body +=
    String(
      GCS_COMPONENT_ID
    );


  body +=
    ",\"sequence\":";


  body +=
    String(
      mavSequence++
    );


  body +=
    "},";


  body +=
    "\"message\":{";


  body +=
    "\"type\":\"MANUAL_CONTROL\",";


  body +=
    "\"target\":";


  body +=
    String(
      TARGET_SYSTEM
    );


  body +=
    ",\"x\":";


  body +=
    String(
      MAV_AXIS_INVALID
    );


  // Steering
  body +=
    ",\"y\":";


  body +=
    String(
      steering
    );


  // Throttle
  body +=
    ",\"z\":";


  body +=
    String(
      throttle
    );


  body +=
    ",\"r\":";


  body +=
    String(
      MAV_AXIS_INVALID
    );


  body +=
    ",\"buttons\":0";


  body +=
    ",\"buttons2\":0";


  body +=
    ",\"enabled_extensions\":0";


  body +=
    ",\"s\":";


  body +=
    String(
      MAV_AXIS_INVALID
    );


  body +=
    ",\"t\":";


  body +=
    String(
      MAV_AXIS_INVALID
    );


  body +=
    ",\"aux1\":";


  body +=
    String(
      MAV_AXIS_INVALID
    );


  body +=
    ",\"aux2\":";


  body +=
    String(
      MAV_AXIS_INVALID
    );


  body +=
    ",\"aux3\":";


  body +=
    String(
      MAV_AXIS_INVALID
    );


  body +=
    ",\"aux4\":";


  body +=
    String(
      MAV_AXIS_INVALID
    );


  body +=
    ",\"aux5\":";


  body +=
    String(
      MAV_AXIS_INVALID
    );


  body +=
    ",\"aux6\":";


  body +=
    String(
      MAV_AXIS_INVALID
    );


  body +=
    "}";


  body +=
    "}";


  return postMavlink(
    body,
    false
  );
}


// ============================================================
// DIRECT ARM / DISARM
// ============================================================

bool sendArmDisarmMavlink(
  bool arm
) {

  String body;


  body.reserve(
    420
  );


  body =
    "{";


  body +=
    "\"header\":{";


  body +=
    "\"system_id\":";


  body +=
    String(
      GCS_SYSTEM_ID
    );


  body +=
    ",\"component_id\":";


  body +=
    String(
      GCS_COMPONENT_ID
    );


  body +=
    ",\"sequence\":";


  body +=
    String(
      mavSequence++
    );


  body +=
    "},";


  body +=
    "\"message\":{";


  body +=
    "\"type\":\"COMMAND_LONG\",";


  body +=
    "\"param1\":";


  body +=
    arm
      ?
      "1.0"
      :
      "0.0";


  body +=
    ",";


  // Normal ArduRover arm/disarm.
  // No forced arming.
  body +=
    "\"param2\":0.0,";


  body +=
    "\"param3\":0.0,";


  body +=
    "\"param4\":0.0,";


  body +=
    "\"param5\":0.0,";


  body +=
    "\"param6\":0.0,";


  body +=
    "\"param7\":0.0,";


  body +=
    "\"command\":{";


  body +=
    "\"type\":\"MAV_CMD_COMPONENT_ARM_DISARM\"";


  body +=
    "},";


  body +=
    "\"target_system\":";


  body +=
    String(
      TARGET_SYSTEM
    );


  body +=
    ",\"target_component\":";


  body +=
    String(
      TARGET_COMPONENT
    );


  body +=
    ",\"confirmation\":0";


  body +=
    "}";


  body +=
    "}";


  Serial.println();


  Serial.println(
    arm
      ?
      "REQUESTING DIRECT MAVLINK ARM"
      :
      "REQUESTING DIRECT MAVLINK DISARM"
  );


  return postMavlink(
    body,
    true
  );
}


// ============================================================
// WAIT FOR JETSON ARM STATE
// ============================================================

bool waitForJetsonArmedState(
  bool expectedArmed
) {

  unsigned long start =
    millis();


  while (
    millis() - start
    <
    ARM_VERIFY_TIMEOUT_MS
  ) {

    BP32.update();


    if (
      syncJetsonState()
    ) {

      if (
        jetsonArmedKnown
        &&
        jetsonArmed
          ==
          expectedArmed
      ) {

        return true;
      }
    }


    delay(
      200
    );
  }


  return false;
}


// ============================================================
// DIRECT ARM / DISARM WITH CONFIRMATION
// ============================================================

bool requestArmDisarmDirect(
  bool arm
) {

  if (
    !sendArmDisarmMavlink(
      arm
    )
  ) {

    Serial.println(
      "ARM/DISARM MAVLink transport failed"
    );


    return false;
  }


  if (
    waitForJetsonArmedState(
      arm
    )
  ) {

    Serial.println();

    Serial.println(
      arm
        ?
        "*** JETSON CONFIRMS VEHICLE ARMED ***"
        :
        "*** JETSON CONFIRMS VEHICLE DISARMED ***"
    );


    return true;
  }


  Serial.println();

  Serial.println(
    arm
      ?
      "*** ARM NOT CONFIRMED BY JETSON ***"
      :
      "*** DISARM NOT CONFIRMED BY JETSON ***"
  );


  return false;
}


// ============================================================
// SEND JETSON COMMAND
// ============================================================

bool sendJetsonCommand(
  const char* command
) {

  if (
    WiFi.status()
    !=
    WL_CONNECTED
  ) {

    return false;
  }


  WiFiClient client;


  client.setTimeout(
    250
  );


  Serial.println();

  Serial.print(
    "Connecting to Jetson "
  );


  Serial.print(
    JETSON_IP
  );


  Serial.print(
    ":"
  );


  Serial.println(
    JETSON_COMMAND_PORT
  );


  if (
    !client.connect(
      JETSON_IP,
      JETSON_COMMAND_PORT
    )
  ) {

    Serial.println(
      "JETSON CONNECTION FAILED"
    );


    return false;
  }


  jetsonRequestCounter++;


  String requestId =
    "esp32_";


  requestId +=
    String(
      jetsonRequestCounter
    );


  requestId +=
    "_";


  requestId +=
    String(
      millis()
    );


  String payload;


  payload =
    "{\"type\":\"command\",\"request_id\":\"";


  payload +=
    requestId;


  payload +=
    "\",\"command\":\"";


  payload +=
    command;


  payload +=
    "\",\"data\":{}}\n";


  Serial.print(
    "Jetson command TX: "
  );


  Serial.print(
    payload
  );


  if (
    client.print(
      payload
    )
    ==
    0
  ) {

    client.stop();


    return false;
  }


  unsigned long start =
    millis();


  while (
    millis() - start
    <
    JETSON_RESPONSE_TIMEOUT_MS
  ) {

    BP32.update();


    if (
      !client.available()
    ) {

      delay(
        5
      );


      continue;
    }


    String line =
      client.readStringUntil(
        '\n'
      );


    line.trim();


    if (
      line.length()
      ==
      0
    ) {

      continue;
    }


    String compact =
      line;


    compact.replace(
      " ",
      ""
    );


    if (
      compact.indexOf(
        "\"type\":\"response\""
      )
      <
      0
    ) {

      continue;
    }


    String search =
      "\"request_id\":\"";


    search +=
      requestId;


    search +=
      "\"";


    if (
      compact.indexOf(
        search
      )
      <
      0
    ) {

      continue;
    }


    Serial.print(
      "Jetson response RX: "
    );


    Serial.println(
      line
    );


    bool success =
      compact.indexOf(
        "\"success\":true"
      )
      >=
      0;


    client.stop();


    return success;
  }


  client.stop();


  Serial.println(
    "JETSON RESPONSE TIMEOUT"
  );


  return false;
}


// ============================================================
// ENGAGE SOFTWARE E-STOP
// ============================================================

void engageSoftwareEstop() {

  Serial.println();

  Serial.println(
    "===================================="
  );

  Serial.println(
    "ENGAGING SOFTWARE E-STOP"
  );

  Serial.println(
    "===================================="
  );


  // ----------------------------------------------------------
  // IMMEDIATELY REVOKE LOCAL PROPULSION AUTHORITY
  // ----------------------------------------------------------

  manualModeEnabled =
    false;


  // ----------------------------------------------------------
  // NEUTRAL FIRST
  // ----------------------------------------------------------

  sendManualControl(
    0,
    0
  );


  // ----------------------------------------------------------
  // HOLD SECOND
  // ----------------------------------------------------------

  requestRoverMode(
    ROVER_MODE_HOLD
  );


  // ----------------------------------------------------------
  // ACTUAL JETSON SOFTWARE STOP
  // ----------------------------------------------------------

  bool result =
    sendJetsonCommand(
      "stop"
    );


  delay(
    100
  );


  syncJetsonState();


  if (
    result
    &&
    jetsonSoftwareStopKnown
    &&
    jetsonSoftwareStopEngaged
  ) {

    Serial.println(
      "*** SOFTWARE E-STOP ENGAGED ***"
    );

  } else {

    Serial.println(
      "*** SOFTWARE E-STOP ENGAGE FAILED / UNCONFIRMED ***"
    );
  }
}


// ============================================================
// CLEAR SOFTWARE E-STOP
// ============================================================

void clearSoftwareEstop() {

  Serial.println();

  Serial.println(
    "===================================="
  );

  Serial.println(
    "CLEARING SOFTWARE E-STOP"
  );

  Serial.println(
    "===================================="
  );


  manualModeEnabled =
    false;


  sendManualControl(
    0,
    0
  );


  requestRoverMode(
    ROVER_MODE_HOLD
  );


  // ----------------------------------------------------------
  // SYNC FIRST
  // ----------------------------------------------------------

  if (
    !syncJetsonState()
  ) {

    Serial.println(
      "*** CLEAR BLOCKED: JETSON STATE UNKNOWN ***"
    );


    return;
  }


  // ----------------------------------------------------------
  // ALREADY CLEARED
  // ----------------------------------------------------------

  if (
    jetsonSoftwareStopKnown
    &&
    !jetsonSoftwareStopEngaged
  ) {

    Serial.println(
      "Software E-stop is already CLEARED."
    );


    return;
  }


  // ----------------------------------------------------------
  // DISARM BEFORE CLEARING IF NEEDED
  // ----------------------------------------------------------

  if (
    jetsonArmedKnown
    &&
    jetsonArmed
  ) {

    Serial.println(
      "Vehicle is armed."
    );


    Serial.println(
      "Disarming before clearing software E-stop..."
    );


    if (
      !requestArmDisarmDirect(
        false
      )
    ) {

      Serial.println(
        "*** CLEAR ABORTED: DISARM NOT CONFIRMED ***"
      );


      return;
    }
  }


  // ----------------------------------------------------------
  // VERIFY DISARMED
  // ----------------------------------------------------------

  if (
    !syncJetsonState()
  ) {

    Serial.println(
      "*** CLEAR ABORTED: STATE SYNC FAILED ***"
    );


    return;
  }


  if (
    !jetsonArmedKnown
    ||
    jetsonArmed
  ) {

    Serial.println(
      "*** CLEAR ABORTED: VEHICLE NOT CONFIRMED DISARMED ***"
    );


    return;
  }


  // ----------------------------------------------------------
  // CLEAR STOP
  // ----------------------------------------------------------

  bool result =
    sendJetsonCommand(
      "clear_stop"
    );


  delay(
    100
  );


  syncJetsonState();


  if (
    result
    &&
    jetsonSoftwareStopKnown
    &&
    !jetsonSoftwareStopEngaged
  ) {

    Serial.println(
      "*** SOFTWARE E-STOP CLEARED ***"
    );

  } else {

    Serial.println(
      "*** SOFTWARE E-STOP CLEAR FAILED / UNCONFIRMED ***"
    );
  }
}


// ============================================================
// CONTROLLER ABSENCE E-STOP FAILSAFE
//
// If the Xbox is absent:
//
// 1. revoke manual authority immediately
// 2. verify Jetson software stop
// 3. if stop is not confirmed ENGAGED:
//      neutral
//      HOLD
//      Jetson "stop"
// 4. periodically re-check while controller remains absent
//
// ============================================================

void enforceControllerEstopFailsafe() {

  // ----------------------------------------------------------
  // CONTROLLER PRESENT
  // ----------------------------------------------------------

  if (
    xboxIsConnected()
  ) {

    return;
  }


  // ----------------------------------------------------------
  // ALWAYS REVOKE MANUAL AUTHORITY
  // ----------------------------------------------------------

  manualModeEnabled =
    false;


  // ----------------------------------------------------------
  // CAN'T CONTACT VEHICLE WITHOUT WIFI
  // ----------------------------------------------------------

  if (
    WiFi.status()
    !=
    WL_CONNECTED
  ) {

    return;
  }


  unsigned long now =
    millis();


  // ----------------------------------------------------------
  // RATE LIMIT FAILSAFE VERIFICATION
  // ----------------------------------------------------------

  if (
    lastControllerFailsafeCheck
    !=
    0

    &&

    now - lastControllerFailsafeCheck
    <
    CONTROLLER_FAILSAFE_CHECK_MS
  ) {

    return;
  }


  lastControllerFailsafeCheck =
    now;


  Serial.println();

  Serial.println(
    "===================================="
  );

  Serial.println(
    "XBOX CONTROLLER ABSENT"
  );

  Serial.println(
    "CONTROLLER FAILSAFE -> SOFTWARE E-STOP"
  );

  Serial.println(
    "===================================="
  );


  // ----------------------------------------------------------
  // CHECK CURRENT JETSON STATE
  // ----------------------------------------------------------

  bool stateOK =
    syncJetsonState();


  if (
    stateOK
    &&
    jetsonSoftwareStopKnown
    &&
    jetsonSoftwareStopEngaged
  ) {

    Serial.println(
      "Controller failsafe E-stop already ENGAGED."
    );


    return;
  }


  // ----------------------------------------------------------
  // STOP IS NOT CONFIRMED.
  //
  // Neutral first.
  // ----------------------------------------------------------

  bool neutralSent =
    sendManualControl(
      0,
      0
    );


  Serial.print(
    "Neutral command: "
  );


  Serial.println(
    neutralSent
      ?
      "SENT"
      :
      "FAILED"
  );


  // ----------------------------------------------------------
  // REQUEST HOLD
  // ----------------------------------------------------------

  bool holdSent =
    requestRoverMode(
      ROVER_MODE_HOLD
    );


  Serial.print(
    "HOLD request: "
  );


  Serial.println(
    holdSent
      ?
      "SENT"
      :
      "FAILED"
  );


  // ----------------------------------------------------------
  // ENGAGE ACTUAL SOFTWARE E-STOP
  // ----------------------------------------------------------

  bool stopSent =
    sendJetsonCommand(
      "stop"
    );


  Serial.print(
    "Jetson E-stop request: "
  );


  Serial.println(
    stopSent
      ?
      "SENT"
      :
      "FAILED"
  );


  delay(
    100
  );


  // ----------------------------------------------------------
  // VERIFY
  // ----------------------------------------------------------

  syncJetsonState();


  if (
    jetsonSoftwareStopKnown
    &&
    jetsonSoftwareStopEngaged
  ) {

    Serial.println();

    Serial.println(
      "*** CONTROLLER FAILSAFE E-STOP CONFIRMED ***"
    );

  } else {

    Serial.println();

    Serial.println(
      "*** CONTROLLER FAILSAFE E-STOP UNCONFIRMED ***"
    );
  }
}


// ============================================================
// B BUTTON
// ============================================================

bool updateSoftwareEstopToggle() {

  if (
    !xboxIsConnected()
  ) {

    previousBButtonState =
      false;


    return false;
  }


  bool currentB =
    xboxController->b();


  bool risingEdge =
    currentB
    &&
    !previousBButtonState;


  previousBButtonState =
    currentB;


  if (
    !risingEdge
  ) {

    return false;
  }


  previousAButtonState =
    xboxController->a();


  previousXButtonState =
    xboxController->x();


  Serial.println();

  Serial.println(
    "B pressed - synchronizing software stop state..."
  );


  if (
    !syncJetsonState()
  ) {

    Serial.println(
      "*** B ACTION BLOCKED: JETSON STATE UNKNOWN ***"
    );


    return true;
  }


  if (
    !jetsonSoftwareStopKnown
  ) {

    Serial.println(
      "*** B ACTION BLOCKED: SOFTWARE STOP STATE UNKNOWN ***"
    );


    return true;
  }


  if (
    jetsonSoftwareStopEngaged
  ) {

    clearSoftwareEstop();

  } else {

    engageSoftwareEstop();
  }


  return true;
}


// ============================================================
// A BUTTON - MANUAL / HOLD
// ============================================================

void updateModeToggle() {

  if (
    !xboxIsConnected()
  ) {

    previousAButtonState =
      false;


    return;
  }


  bool currentA =
    xboxController->a();


  bool risingEdge =
    currentA
    &&
    !previousAButtonState;


  previousAButtonState =
    currentA;


  if (
    !risingEdge
  ) {

    return;
  }


  if (
    !syncJetsonState()
  ) {

    Serial.println(
      "*** MANUAL BLOCKED: JETSON STATE UNKNOWN ***"
    );


    return;
  }


  if (
    !jetsonSoftwareStopKnown
    ||
    jetsonSoftwareStopEngaged
  ) {

    Serial.println(
      "*** MANUAL BLOCKED: SOFTWARE E-STOP ENGAGED ***"
    );


    return;
  }


  // ----------------------------------------------------------
  // ENTER MANUAL
  // ----------------------------------------------------------

  if (
    !manualModeEnabled
  ) {

    bool accepted =
      requestRoverMode(
        ROVER_MODE_MANUAL
      );


    if (
      accepted
    ) {

      manualModeEnabled =
        true;


      Serial.println(
        "*** MANUAL MODE REQUEST SENT ***"
      );

    } else {

      manualModeEnabled =
        false;


      Serial.println(
        "*** MANUAL MODE REQUEST FAILED ***"
      );
    }
  }

  // ----------------------------------------------------------
  // EXIT MANUAL
  // ----------------------------------------------------------

  else {

    sendManualControl(
      0,
      0
    );


    requestRoverMode(
      ROVER_MODE_HOLD
    );


    manualModeEnabled =
      false;


    Serial.println(
      "*** HOLD MODE REQUEST SENT ***"
    );
  }
}


// ============================================================
// X BUTTON - ARM / DISARM
// ============================================================

void updateArmToggle() {

  if (
    !xboxIsConnected()
  ) {

    previousXButtonState =
      false;


    return;
  }


  bool currentX =
    xboxController->x();


  bool risingEdge =
    currentX
    &&
    !previousXButtonState;


  previousXButtonState =
    currentX;


  if (
    !risingEdge
  ) {

    return;
  }


  Serial.println();

  Serial.println(
    "X pressed - synchronizing vehicle state..."
  );


  if (
    !syncJetsonState()
  ) {

    Serial.println(
      "*** ARM/DISARM BLOCKED: JETSON STATE UNKNOWN ***"
    );


    return;
  }


  // ==========================================================
  // ARMED -> DISARM
  // ==========================================================

  if (
    jetsonArmed
  ) {

    sendManualControl(
      0,
      0
    );


    requestArmDisarmDirect(
      false
    );


    return;
  }


  // ==========================================================
  // DISARMED -> ARM
  // ==========================================================

  if (
    !jetsonSoftwareStopKnown
    ||
    jetsonSoftwareStopEngaged
  ) {

    Serial.println(
      "*** ARM BLOCKED: SOFTWARE E-STOP ENGAGED ***"
    );


    return;
  }


  if (
    !manualModeEnabled
  ) {

    Serial.println(
      "*** ARM BLOCKED: ENTER MANUAL FIRST ***"
    );


    return;
  }


  requestArmDisarmDirect(
    true
  );
}


// ============================================================
// CONTROLLER CONNECTED
// ============================================================

void onControllerConnected(
  ControllerPtr controller
) {

  if (
    xboxController
    ==
    nullptr
  ) {

    xboxController =
      controller;


    // --------------------------------------------------------
    // DO NOT RESTORE MANUAL AUTHORITY AUTOMATICALLY
    // --------------------------------------------------------

    manualModeEnabled =
      false;


    previousAButtonState =
      false;


    previousBButtonState =
      false;


    previousXButtonState =
      false;


    lastControllerFailsafeCheck =
      0;


    Serial.println();

    Serial.println(
      "===================================="
    );

    Serial.println(
      "XBOX CONTROLLER CONNECTED"
    );

    Serial.println(
      "===================================="
    );


    Serial.println(
      "Controller failsafe no longer active."
    );


    Serial.println(
      "Software E-stop is NOT automatically cleared."
    );


    Serial.println(
      "Press B to clear E-stop if required."
    );


    Serial.println(
      "Then press A for MANUAL."
    );
  }
}


// ============================================================
// CONTROLLER DISCONNECTED
//
// IMPORTANT:
//
// Do not perform the entire TCP Jetson transaction inside
// the Bluepad32 callback.
//
// Instead:
//
// 1. immediately revoke local authority
// 2. immediately send neutral/HOLD if network is present
// 3. reset failsafe timer
// 4. main loop engages and verifies E-stop immediately
//
// ============================================================

void onControllerDisconnected(
  ControllerPtr controller
) {

  if (
    controller
    !=
    xboxController
  ) {

    return;
  }


  Serial.println();

  Serial.println(
    "===================================="
  );

  Serial.println(
    "*** XBOX CONTROLLER DISCONNECTED ***"
  );

  Serial.println(
    "FAILSAFE -> SOFTWARE E-STOP"
  );

  Serial.println(
    "===================================="
  );


  xboxController =
    nullptr;


  manualModeEnabled =
    false;


  previousAButtonState =
    false;


  previousBButtonState =
    false;


  previousXButtonState =
    false;


  // ----------------------------------------------------------
  // FORCE MAIN LOOP FAILSAFE TO RUN IMMEDIATELY
  // ----------------------------------------------------------

  lastControllerFailsafeCheck =
    0;


  // ----------------------------------------------------------
  // IMMEDIATE DIRECT VEHICLE NEUTRAL / HOLD
  //
  // Jetson stop will be sent by
  // enforceControllerEstopFailsafe() on the next loop.
  // ----------------------------------------------------------

  if (
    WiFi.status()
    ==
    WL_CONNECTED
  ) {

    sendManualControl(
      0,
      0
    );


    requestRoverMode(
      ROVER_MODE_HOLD
    );


    Serial.println(
      "Immediate neutral/HOLD sent."
    );


    Serial.println(
      "Software E-stop pending..."
    );

  } else {

    Serial.println(
      "Wi-Fi unavailable."
    );


    Serial.println(
      "Software E-stop will be requested after reconnect."
    );
  }
}


// ============================================================
// JOYSTICK CONTROL
// ============================================================

void updateManualControl() {

  if (
    !xboxIsConnected()
  ) {

    return;
  }


  if (
    !manualModeEnabled
  ) {

    return;
  }


  if (
    !jetsonStateKnown
    ||
    !jetsonSoftwareStopKnown
    ||
    jetsonSoftwareStopEngaged
    ||
    !jetsonArmedKnown
    ||
    !jetsonArmed
  ) {

    return;
  }


  unsigned long now =
    millis();


  if (
    now - lastManualSend
    <
    MANUAL_SEND_PERIOD_MS
  ) {

    return;
  }


  lastManualSend =
    now;


  bool deadman =
    xboxController->l1();


  int16_t steering =
    0;


  int16_t throttle =
    0;


  if (
    deadman
  ) {

    float throttleNormalized =
      -normalizeAxis(
        xboxController->axisY()
      );


    throttle =
      (int16_t)(
        throttleNormalized
        *
        MAV_CONTROL_MAX
      );


    float steeringNormalized =
      normalizeAxis(
        xboxController->axisRX()
      );


    steering =
      (int16_t)(
        steeringNormalized
        *
        MAV_CONTROL_MAX
      );
  }


  // LB released -> continuously neutral
  sendManualControl(
    steering,
    throttle
  );
}


// ============================================================
// WIFI MAINTENANCE
// ============================================================

void maintainWiFi() {

  if (
    WiFi.status()
    ==
    WL_CONNECTED
  ) {

    return;
  }


  // ----------------------------------------------------------
  // FAIL CLOSED LOCALLY
  // ----------------------------------------------------------

  manualModeEnabled =
    false;


  jetsonStateKnown =
    false;


  jetsonSoftwareStopKnown =
    false;


  jetsonArmedKnown =
    false;


  // If the Xbox is absent, make sure E-stop enforcement
  // occurs immediately once Wi-Fi comes back.
  lastControllerFailsafeCheck =
    0;


  unsigned long now =
    millis();


  if (
    now - lastWiFiReconnect
    <
    WIFI_RECONNECT_MS
  ) {

    return;
  }


  lastWiFiReconnect =
    now;


  Serial.println(
    "BaseStation connection lost"
  );


  WiFi.disconnect(
    false,
    false
  );


  delay(
    100
  );


  WiFi.setMinSecurity(
    WIFI_AUTH_WPA_PSK
  );


  WiFi.begin(
    WIFI_SSID,
    WIFI_PASSWORD
  );
}


// ============================================================
// DEBUG
// ============================================================

void printDebugInfo() {

  unsigned long now =
    millis();


  if (
    now - lastDebug
    <
    DEBUG_PERIOD_MS
  ) {

    return;
  }


  lastDebug =
    now;


  Serial.println();

  Serial.println(
    "========== ROBOTX DIRECT CONTROL =========="
  );


  // ----------------------------------------------------------
  // WIFI
  // ----------------------------------------------------------

  Serial.print(
    "BaseStation: "
  );


  Serial.println(
    WiFi.status() == WL_CONNECTED
      ?
      "CONNECTED"
      :
      "DISCONNECTED"
  );


  if (
    WiFi.status()
    ==
    WL_CONNECTED
  ) {

    Serial.print(
      "ESP32 IP: "
    );


    Serial.println(
      WiFi.localIP()
    );


    Serial.print(
      "RSSI: "
    );


    Serial.print(
      WiFi.RSSI()
    );


    Serial.println(
      " dBm"
    );
  }


  // ----------------------------------------------------------
  // NETWORK TARGETS
  // ----------------------------------------------------------

  Serial.print(
    "BlueOS: "
  );


  Serial.print(
    BLUEOS_IP
  );


  Serial.print(
    ":"
  );


  Serial.println(
    MAVLINK2REST_PORT
  );


  Serial.print(
    "Jetson: "
  );


  Serial.print(
    JETSON_IP
  );


  Serial.print(
    ":"
  );


  Serial.println(
    JETSON_COMMAND_PORT
  );


  // ----------------------------------------------------------
  // JETSON STATE
  // ----------------------------------------------------------

  Serial.print(
    "Jetson state synchronized: "
  );


  Serial.println(
    jetsonStateKnown
      ?
      "YES"
      :
      "NO"
  );


  Serial.print(
    "Jetson software stop: "
  );


  if (
    !jetsonSoftwareStopKnown
  ) {

    Serial.println(
      "UNKNOWN"
    );

  } else {

    Serial.println(
      jetsonSoftwareStopEngaged
        ?
        "ENGAGED"
        :
        "CLEARED"
    );
  }


  Serial.print(
    "Jetson vehicle armed: "
  );


  if (
    !jetsonArmedKnown
  ) {

    Serial.println(
      "UNKNOWN"
    );

  } else {

    Serial.println(
      jetsonArmed
        ?
        "YES"
        :
        "NO"
    );
  }


  Serial.print(
    "Jetson vehicle mode: "
  );


  Serial.println(
    jetsonMode
  );


  // ----------------------------------------------------------
  // CONTROLLER FAILSAFE
  // ----------------------------------------------------------

  Serial.print(
    "Controller failsafe: "
  );


  Serial.println(
    xboxIsConnected()
      ?
      "INACTIVE"
      :
      "ACTIVE -> SOFTWARE E-STOP"
  );


  // ----------------------------------------------------------
  // XBOX
  // ----------------------------------------------------------

  Serial.print(
    "Xbox: "
  );


  if (
    xboxIsConnected()
  ) {

    Serial.println(
      "CONNECTED"
    );


    bool a =
      xboxController->a();


    bool b =
      xboxController->b();


    bool x =
      xboxController->x();


    bool lb =
      xboxController->l1();


    int rawThrottle =
      xboxController->axisY();


    int rawSteering =
      xboxController->axisRX();


    float throttle =
      -normalizeAxis(
        rawThrottle
      );


    float steering =
      normalizeAxis(
        rawSteering
      );


    Serial.print(
      "A: "
    );


    Serial.println(
      a
        ?
        "PRESSED"
        :
        "released"
    );


    Serial.print(
      "B: "
    );


    Serial.println(
      b
        ?
        "PRESSED"
        :
        "released"
    );


    Serial.print(
      "X: "
    );


    Serial.println(
      x
        ?
        "PRESSED"
        :
        "released"
    );


    Serial.print(
      "LB: "
    );


    Serial.println(
      lb
        ?
        "PRESSED"
        :
        "released"
    );


    Serial.print(
      "ESP32 manual state: "
    );


    Serial.println(
      manualModeEnabled
        ?
        "MANUAL REQUESTED"
        :
        "HOLD / NOT MANUAL"
    );


    Serial.print(
      "Raw Left Y: "
    );


    Serial.println(
      rawThrottle
    );


    Serial.print(
      "Raw Right X: "
    );


    Serial.println(
      rawSteering
    );


    Serial.print(
      "Throttle: "
    );


    Serial.println(
      throttle,
      3
    );


    Serial.print(
      "Steering: "
    );


    Serial.println(
      steering,
      3
    );


    bool movementPermitted =
      jetsonStateKnown

      &&

      jetsonSoftwareStopKnown

      &&

      !jetsonSoftwareStopEngaged

      &&

      jetsonArmedKnown

      &&

      jetsonArmed

      &&

      manualModeEnabled

      &&

      lb;


    Serial.print(
      "Movement permitted: "
    );


    Serial.println(
      movementPermitted
        ?
        "YES"
        :
        "NO"
    );

  } else {

    Serial.println(
      "DISCONNECTED"
    );


    Serial.println(
      "ESP32 manual state: DISABLED"
    );


    Serial.println(
      "Required safety state: SOFTWARE E-STOP ENGAGED"
    );


    Serial.println(
      "Movement permitted: NO"
    );
  }


  Serial.println(
    "============================================"
  );
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(
    115200
  );


  delay(
    3000
  );


  Serial.println();

  Serial.println(
    "========================================"
  );

  Serial.println(
    "RobotX ESP32-S3 Direct BlueBoat Control"
  );

  Serial.println(
    "========================================"
  );


  // ----------------------------------------------------------
  // FAIL-CLOSED STARTUP
  // ----------------------------------------------------------

  manualModeEnabled =
    false;


  jetsonStateKnown =
    false;


  jetsonSoftwareStopKnown =
    false;


  jetsonArmedKnown =
    false;


  lastControllerFailsafeCheck =
    0;


  // ----------------------------------------------------------
  // WIFI
  // ----------------------------------------------------------

  WiFi.mode(
    WIFI_STA
  );


  // IMPORTANT:
  //
  // Do NOT use:
  //
  // WiFi.setSleep(false)
  //
  // Bluepad32 + Wi-Fi coexistence requires modem sleep.


  connectToBaseStation();


  // ----------------------------------------------------------
  // BLUEOS
  // ----------------------------------------------------------

  if (
    WiFi.status()
    ==
    WL_CONNECTED
  ) {

    testBlueOSConnection();
  }


  // ----------------------------------------------------------
  // BLUEPAD32
  // ----------------------------------------------------------

  BP32.setup(
    &onControllerConnected,
    &onControllerDisconnected
  );


  BP32.enableNewBluetoothConnections(
    true
  );


  // ----------------------------------------------------------
  // INITIAL JETSON STATE
  // ----------------------------------------------------------

  if (
    WiFi.status()
    ==
    WL_CONNECTED
  ) {

    Serial.println();

    Serial.println(
      "Synchronizing initial Jetson state..."
    );


    syncJetsonState();
  }


  // ----------------------------------------------------------
  // CONTROLS
  // ----------------------------------------------------------

  Serial.println();

  Serial.println(
    "CONTROLS"
  );


  Serial.println(
    "A  = MANUAL / HOLD"
  );


  Serial.println(
    "B  = SOFTWARE E-STOP TOGGLE"
  );


  Serial.println(
    "X  = ARM / DISARM"
  );


  Serial.println(
    "LB = DEADMAN"
  );


  Serial.println(
    "Left Y  = FORWARD / REVERSE"
  );


  Serial.println(
    "Right X = STEERING"
  );


  Serial.println();

  Serial.println(
    "CONTROLLER FAILSAFE:"
  );


  Serial.println(
    "Xbox disconnected = SOFTWARE E-STOP ENGAGED"
  );


  Serial.println(
    "E-stop remains engaged after reconnect."
  );


  Serial.println(
    "Press B to clear it deliberately."
  );
}


// ============================================================
// LOOP
// ============================================================

void loop() {

  // ----------------------------------------------------------
  // BLUETOOTH
  // ----------------------------------------------------------

  BP32.update();


  // ----------------------------------------------------------
  // WIFI
  // ----------------------------------------------------------

  maintainWiFi();


  // ----------------------------------------------------------
  // CONTROLLER FAILSAFE
  //
  // Xbox absent = SOFTWARE E-STOP must be ENGAGED
  // ----------------------------------------------------------

  enforceControllerEstopFailsafe();


  // ----------------------------------------------------------
  // B - SOFTWARE E-STOP
  // ----------------------------------------------------------

  bool estopEvent =
    updateSoftwareEstopToggle();


  if (
    !estopEvent
  ) {

    // --------------------------------------------------------
    // A - MANUAL / HOLD
    // --------------------------------------------------------

    updateModeToggle();


    // --------------------------------------------------------
    // X - ARM / DISARM
    // --------------------------------------------------------

    updateArmToggle();
  }


  // ----------------------------------------------------------
  // JOYSTICK
  // ----------------------------------------------------------

  updateManualControl();


  // ----------------------------------------------------------
  // STATUS
  // ----------------------------------------------------------

  printDebugInfo();


  delay(
    1
  );
}