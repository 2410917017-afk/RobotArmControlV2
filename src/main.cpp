#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <FastAccelStepper.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace {

constexpr uint8_t kAxisCount = 5;
constexpr uint32_t kBaudRate = 115200;
constexpr size_t kLineCapacity = 512;
constexpr uint8_t kQueueLength = 8;
constexpr uint8_t kTxQueueLength = 16;
constexpr size_t kOutputCapacity = 768;
constexpr uint16_t kDefaultFeed = 300;       // pasos/s del eje con mayor recorrido
constexpr uint16_t kMaximumFeed = 800;       // pasos/s
constexpr uint32_t kBaseAcceleration = 200;  // pasos/s^2 del eje con mayor recorrido
constexpr TickType_t kMotionPeriod = pdMS_TO_TICKS(5) > 0 ? pdMS_TO_TICKS(5) : 1;
constexpr uint32_t kPositionReportPeriodMs = 200;

// Pinout propuesto y confirmado por el usuario para su ESP32 estándar.
// STEP y DIR están separados por eje; confirmar etiquetas físicas de la placa.
constexpr uint8_t kStepPins[kAxisCount] = {25, 26, 27, 32, 33};
constexpr uint8_t kDirectionPins[kAxisCount] = {16, 17, 18, 19, 21};
// Se puede compartir ENABLE conectando los EN de los cinco DRV8825 juntos.
constexpr int8_t kEnablePins[kAxisCount] = {23, 23, 23, 23, 23};
constexpr bool kEnableActiveLow = true;
constexpr bool kEnableDriversAtBoot = true;

// Pin y recorrido confirmados: cerrado=0°, abierto=55°.
// Cambiar kToolPin solo si posteriormente se modifica el cableado.
constexpr int8_t kToolPin = 22;
constexpr uint8_t kToolOpenAngle = 55;
constexpr uint8_t kToolClosedAngle = 0;

struct AxisCalibration {
  uint16_t fullStepsPerMotorRevolution;
  uint8_t microsteps;
  float motorRevolutionsPerJointRevolution;
  int8_t directionSign;
  float minimumJointDegrees;
  float maximumJointDegrees;
};

// Reducciones de la tabla para las primeras cinco articulaciones.
// Los límites +/-10 grados son deliberadamente estrechos para las primeras
// pruebas; ajústelos tras verificar el montaje y el sentido de cada eje.
AxisCalibration kAxis[kAxisCount] = {
    {200, 1, 3.5f, 1, -10.0f, 10.0f},
    {200, 1, 83.3f, 1, -10.0f, 10.0f},
    {200, 1, 43.3f, 1, -10.0f, 10.0f},
    {200, 1, 4.75f, 1, -10.0f, 10.0f},
    {200, 1, 3.0f, 1, -10.0f, 10.0f},
};

enum CommandType : uint8_t {
  CMD_MOVE,
  CMD_HOME,
  CMD_STOP,
  CMD_TOOL,
  CMD_STATUS,
};

struct MotionCommand {
  uint32_t id;
  CommandType type;
  long targetSteps[kAxisCount];
  uint16_t feed;
  bool toolOpen;
};

struct OutgoingLine {
  char data[kOutputCapacity];
};

FastAccelStepperEngine stepperEngine;
FastAccelStepper* steppers[kAxisCount] = {};

QueueHandle_t commandQueue = nullptr;
QueueHandle_t txQueue = nullptr;
bool toolOpenState = false;
bool positionReferenced = true;  // origen lógico fijado en la pose al encender
uint32_t activeCommandId = 0;
uint32_t activeMoveCommandId = 0;
Servo gripperServo;

float stepsPerJointDegree(uint8_t axis) {
  const AxisCalibration& c = kAxis[axis];
  return static_cast<float>(c.fullStepsPerMotorRevolution) * c.microsteps *
         c.motorRevolutionsPerJointRevolution / 360.0f;
}

long jointDegreesToSteps(uint8_t axis, float degrees) {
  const float steps = degrees * stepsPerJointDegree(axis) * kAxis[axis].directionSign;
  return lroundf(steps);
}

float stepsToJointDegrees(uint8_t axis, long steps) {
  return static_cast<float>(steps) /
         (stepsPerJointDegree(axis) * kAxis[axis].directionSign);
}

bool anyAxisMoving() {
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    if (steppers[i] != nullptr && steppers[i]->isRunning()) return true;
  }
  return false;
}

template <typename TDocument>
void writeJsonLine(const TDocument& document) {
  if (txQueue == nullptr) return;
  OutgoingLine line = {};
  const size_t length = serializeJson(document, line.data, sizeof(line.data) - 1);
  if (length == 0 || length >= sizeof(line.data) - 1) return;
  line.data[length] = '\0';
  xQueueSend(txQueue, &line, 0);
}

void sendError(const char* command, const char* code, uint32_t id = 0) {
  StaticJsonDocument<192> response;
  response["event"] = "error";
  response["cmd"] = command;
  response["code"] = code;
  response["id"] = id;
  writeJsonLine(response);
}

void sendAck(const char* command, uint32_t id) {
  StaticJsonDocument<160> response;
  response["event"] = "ack";
  response["cmd"] = command;
  response["id"] = id;
  response["queued"] = true;
  writeJsonLine(response);
}

void sendPosition(const char* eventName, uint32_t eventId) {
  StaticJsonDocument<1024> response;
  response["event"] = eventName;
  response["id"] = eventId;
  response["moving"] = anyAxisMoving();
  response["homed"] = positionReferenced;
  response["reference"] = "power_on_pose_or_software_home";
  response["position_source"] = "commanded_open_loop_estimate";

  JsonArray axes = response.createNestedArray("axes");
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    JsonObject axis = axes.createNestedObject();
    const int32_t steps = steppers[i]->getCurrentPosition();
    const int32_t target = steppers[i]->targetPos();
    axis["joint"] = i + 1;
    axis["steps"] = steps;
    axis["joint_deg_est"] = stepsToJointDegrees(i, steps);
    axis["motor_deg_est"] = static_cast<float>(steps) * 360.0f /
                             (kAxis[i].fullStepsPerMotorRevolution * kAxis[i].microsteps);
    axis["target_joint_deg"] = stepsToJointDegrees(i, target);
  }
  response["tool_open"] = toolOpenState;
  writeJsonLine(response);
}

void sendPosition(const char* eventName) {
  sendPosition(eventName, activeCommandId);
}

void setDriversEnabled(bool enabled) {
  const uint8_t enabledLevel = kEnableActiveLow ? LOW : HIGH;
  const uint8_t disabledLevel = kEnableActiveLow ? HIGH : LOW;
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    if (kEnablePins[i] >= 0) {
      digitalWrite(kEnablePins[i], enabled ? enabledLevel : disabledLevel);
    }
  }
}

bool enqueueCommand(const MotionCommand& command, bool clearQueue = false) {
  if (clearQueue) xQueueReset(commandQueue);
  return xQueueSend(commandQueue, &command, pdMS_TO_TICKS(10)) == pdTRUE;
}

void handleLine(const char* line) {
  StaticJsonDocument<512> document;
  const DeserializationError parseError = deserializeJson(document, line);
  if (parseError) {
    sendError("", "invalid_json");
    return;
  }

  if (!document["cmd"].is<const char*>()) {
    sendError("", "missing_cmd");
    return;
  }
  const char* commandName = document["cmd"].as<const char*>();
  uint32_t id = document["id"] | 0UL;
  MotionCommand command = {};
  command.id = id;

  if (strcmp(commandName, "MOVE") == 0) {
    command.type = CMD_MOVE;
    JsonArrayConst axes = document["axes"].as<JsonArrayConst>();
    if (axes.isNull() || axes.size() != kAxisCount) {
      sendError(commandName, "axes_must_contain_5_angles", id);
      return;
    }

    if (document.containsKey("feed") && !document["feed"].is<float>() &&
        !document["feed"].is<int>() && !document["feed"].is<unsigned int>()) {
      sendError(commandName, "feed_must_be_numeric", id);
      return;
    }
    const float feed = document["feed"] | static_cast<float>(kDefaultFeed);
    if (!isfinite(feed) || feed < 1.0f || feed > kMaximumFeed) {
      sendError(commandName, "feed_out_of_range", id);
      return;
    }
    command.feed = static_cast<uint16_t>(feed);

    for (uint8_t i = 0; i < kAxisCount; ++i) {
      const JsonVariantConst value = axes[i];
      if (value.isNull() || (!value.is<float>() && !value.is<int>() &&
                             !value.is<unsigned int>())) {
        sendError(commandName, "axis_angle_must_be_numeric", id);
        return;
      }
      const float degrees = value.as<float>();
      if (!isfinite(degrees)) {
        sendError(commandName, "axis_angle_not_finite", id);
        return;
      }
      if (degrees < kAxis[i].minimumJointDegrees || degrees > kAxis[i].maximumJointDegrees) {
        sendError(commandName, "joint_limit", id);
        return;
      }
      command.targetSteps[i] = jointDegreesToSteps(i, degrees);
    }
    if (!enqueueCommand(command)) {
      sendError(commandName, "queue_full", id);
      return;
    }
    sendAck(commandName, id);
    return;
  }

  if (strcmp(commandName, "STOP") == 0) {
    command.type = CMD_STOP;
    if (!enqueueCommand(command, true)) {
      sendError(commandName, "queue_full", id);
      return;
    }
    sendAck(commandName, id);
    return;
  }

  if (strcmp(commandName, "HOME") == 0) {
    command.type = CMD_HOME;
    if (!enqueueCommand(command)) {
      sendError(commandName, "queue_full", id);
      return;
    }
    sendAck(commandName, id);
    return;
  }

  if (strcmp(commandName, "STATUS") == 0) {
    command.type = CMD_STATUS;
    if (!enqueueCommand(command)) {
      sendError(commandName, "queue_full", id);
      return;
    }
    sendAck(commandName, id);
    return;
  }

  if (strcmp(commandName, "TOOL") == 0) {
    if (!document["tool"].is<bool>()) {
      sendError(commandName, "tool_must_be_boolean", id);
      return;
    }
    command.type = CMD_TOOL;
    command.toolOpen = document["tool"].as<bool>();
    if (!enqueueCommand(command)) {
      sendError(commandName, "queue_full", id);
      return;
    }
    sendAck(commandName, id);
    return;
  }

  sendError(commandName, "unknown_command", id);
}

void serialTask(void*) {
  char line[kLineCapacity];
  size_t used = 0;
  bool overflow = false;

  for (;;) {
    while (Serial.available() > 0) {
      const char c = static_cast<char>(Serial.read());
      if (c == '\r') continue;
      if (c == '\n') {
        if (overflow) {
          sendError("", "line_too_long");
        } else if (used > 0) {
          line[used] = '\0';
          handleLine(line);
        }
        used = 0;
        overflow = false;
        continue;
      }
      if (!overflow) {
        if (used < sizeof(line) - 1) {
          line[used++] = c;
        } else {
          overflow = true;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void txTask(void*) {
  OutgoingLine line;
  for (;;) {
    if (xQueueReceive(txQueue, &line, portMAX_DELAY) == pdTRUE) {
      Serial.println(line.data);
    }
  }
}

void applyMove(const MotionCommand& command) {
  int32_t largestDistance = 0;
  int32_t targets[kAxisCount];
  int32_t distances[kAxisCount];
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    targets[i] = static_cast<int32_t>(command.targetSteps[i]);
    distances[i] = abs(targets[i] - steppers[i]->getCurrentPosition());
    if (distances[i] > largestDistance) largestDistance = distances[i];
  }

  setDriversEnabled(true);
  activeCommandId = command.id;
  activeMoveCommandId = command.id;
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    const float fraction = largestDistance == 0
                               ? 1.0f
                               : static_cast<float>(distances[i]) / largestDistance;
    uint32_t axisSpeed = static_cast<uint32_t>(command.feed * fraction);
    int32_t axisAcceleration = static_cast<int32_t>(kBaseAcceleration * fraction);
    if (axisSpeed < 1) axisSpeed = 1;
    if (axisAcceleration < 1) axisAcceleration = 1;
    if (steppers[i]->setSpeedInHz(axisSpeed) != 0 ||
        steppers[i]->setAcceleration(axisAcceleration) != 0) {
      sendError("MOVE", "stepper_speed_or_acceleration_rejected", command.id);
      return;
    }
  }
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    if (steppers[i]->moveTo(targets[i]) != MOVE_OK) {
      for (uint8_t stopIndex = 0; stopIndex < kAxisCount; ++stopIndex) {
        steppers[stopIndex]->stopMove();
      }
      sendError("MOVE", "stepper_rejected_target", command.id);
      return;
    }
  }
  sendPosition(largestDistance == 0 ? "complete" : "position");
}

void motionTask(void*) {
  MotionCommand command;
  TickType_t wakeTime = xTaskGetTickCount();
  uint32_t lastPositionReport = millis();
  bool wasMoving = false;

  for (;;) {
    if (xQueueReceive(commandQueue, &command, 0) == pdTRUE) {
      switch (command.type) {
        case CMD_MOVE:
          applyMove(command);
          wasMoving = anyAxisMoving();
          lastPositionReport = millis();
          break;

        case CMD_STOP:
          activeCommandId = command.id;
          activeMoveCommandId = command.id;
          for (uint8_t i = 0; i < kAxisCount; ++i) steppers[i]->stopMove();
          sendPosition("stopping");
          break;

        case CMD_HOME:
          activeCommandId = command.id;
          if (anyAxisMoving()) {
            sendError("HOME", "stop_before_software_home", command.id);
          } else {
            for (uint8_t i = 0; i < kAxisCount; ++i) steppers[i]->setCurrentPosition(0);
            positionReferenced = true;
            sendPosition("homed");
          }
          break;

        case CMD_TOOL:
          activeCommandId = command.id;
          toolOpenState = command.toolOpen;
          gripperServo.write(toolOpenState ? kToolOpenAngle : kToolClosedAngle);
          sendPosition("tool");
          break;

        case CMD_STATUS:
          sendPosition("status", command.id);
          break;
      }
    }

    const bool moving = anyAxisMoving();
    const uint32_t now = millis();
    if (moving && now - lastPositionReport >= kPositionReportPeriodMs) {
      sendPosition("position");
      lastPositionReport = now;
    } else if (wasMoving && !moving) {
      sendPosition("complete", activeMoveCommandId);
    }
    wasMoving = moving;

    vTaskDelayUntil(&wakeTime, kMotionPeriod);
  }
}

}  // namespace

void setup() {
  Serial.begin(kBaudRate);

  stepperEngine.init();
  for (uint8_t i = 0; i < kAxisCount; ++i) {
    steppers[i] = stepperEngine.stepperConnectToPin(kStepPins[i]);
    if (steppers[i] == nullptr) {
      Serial.printf("{\"event\":\"error\",\"code\":\"stepper_engine_connection_failed\",\"axis\":%u}\n",
                    static_cast<unsigned>(i + 1));
      for (;;) delay(1000);
    }
    steppers[i]->setDirectionPin(kDirectionPins[i]);
    if (steppers[i]->setSpeedInHz(kDefaultFeed) != 0 ||
        steppers[i]->setAcceleration(kBaseAcceleration) != 0) {
      Serial.printf("{\"event\":\"error\",\"code\":\"stepper_engine_configuration_failed\",\"axis\":%u}\n",
                    static_cast<unsigned>(i + 1));
      for (;;) delay(1000);
    }
    steppers[i]->setCurrentPosition(0);
    if (kEnablePins[i] >= 0) {
      const uint8_t disabledLevel = kEnableActiveLow ? HIGH : LOW;
      pinMode(kEnablePins[i], OUTPUT);
      digitalWrite(kEnablePins[i], disabledLevel);
    }
  }
  setDriversEnabled(kEnableDriversAtBoot);

  gripperServo.setPeriodHertz(50);
  gripperServo.attach(kToolPin, 500, 2400);
  gripperServo.write(kToolClosedAngle);

  commandQueue = xQueueCreate(kQueueLength, sizeof(MotionCommand));
  txQueue = xQueueCreate(kTxQueueLength, sizeof(OutgoingLine));
  if (commandQueue == nullptr || txQueue == nullptr) {
    Serial.println("{\"event\":\"error\",\"code\":\"freertos_resource_creation_failed\"}");
    for (;;) delay(1000);
  }

  StaticJsonDocument<256> ready;
  ready["event"] = "ready";
  ready["axes"] = kAxisCount;
  ready["stepper_engine"] = "FastAccelStepper";
  ready["homed"] = true;
  ready["reference"] = "power_on_pose";
  ready["position_source"] = "commanded_open_loop_estimate";
  writeJsonLine(ready);
  sendPosition("status");

  const BaseType_t txCreated = xTaskCreatePinnedToCore(
      txTask, "txTask", 4096, nullptr, 1, nullptr, 0);
  const BaseType_t serialCreated = xTaskCreatePinnedToCore(
      serialTask, "serialTask", 4096, nullptr, 1, nullptr, 0);
  const BaseType_t motionCreated = xTaskCreatePinnedToCore(
      motionTask, "motionTask", 4096, nullptr, 2, nullptr, 1);
  if (txCreated != pdPASS || serialCreated != pdPASS || motionCreated != pdPASS) {
    Serial.println("{\"event\":\"error\",\"code\":\"task_creation_failed\"}");
    for (;;) delay(1000);
  }
}

void loop() {
  delay(1000);
}
