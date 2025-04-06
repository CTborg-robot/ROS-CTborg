#include <Arduino.h>

#define NUM_SECTORS 12

// Настройки для управления моторами
#define MOTOR_LEFT_FORWARD_PIN 14   // Пин для управления левым двигателем (вперёд)
#define MOTOR_LEFT_BACKWARD_PIN 12  // Пин для управления левым двигателем (назад)
#define MOTOR_RIGHT_FORWARD_PIN 27  // Пин для управления правым двигателем (вперёд)
#define MOTOR_RIGHT_BACKWARD_PIN 26 // Пин для управления правым двигателем (назад)

// Настройки для лидара
#define LIDAR_RX_PIN 25
#define BAUDRATE 115200

// Структура пакета лидара: 4 байта заголовка, затем 32 байта данных
static const uint8_t LIDAR_HEADER[] = { 0x55, 0xAA, 0x03, 0x08 };
static const uint8_t LIDAR_HEADER_LEN = 4;
static const uint8_t LIDAR_BODY_LEN = 32;

enum State {
  IDLE,
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT
};

State state = IDLE;
// Пороговые расстояния (в миллиметрах)
#define ALARM_DIST 40     // Менее 400 мм -> сектор в красном цвете
#define WARNING_DIST 65   // Менее 650 мм (но >= 400 мм) -> жёлтый
#define NO_VALUE 99999.0f  // Значение, которое будет означать отсутствие данных

// Храним текущее измеренное расстояние по каждому из 12 секторов.
static float sectorDistances[NUM_SECTORS] = { 0.0f };

// Время последнего обновления данных сектора (в миллисекундах).
static uint32_t sectorUpdateTime[NUM_SECTORS] = { 0 };



/**
 * @brief Блокирующее чтение заданного количества байт из Serial с учётом таймаута.
 * 
 * @param ser Ссылка на объект Serial (например, Serial1)
 * @param buffer Указатель на буфер, куда записывать считанные байты
 * @param length Количество байт, которое необходимо прочитать
 * @param timeout_ms Максимальное время ожидания в миллисекундах (по умолчанию 500 мс)
 * @return true, если все байты успешно прочитаны, иначе false
 */
bool readBytesWithTimeout(HardwareSerial &ser, uint8_t *buffer, size_t length, uint32_t timeout_ms = 500) {
  uint32_t start = millis();
  size_t count = 0;

  while (count < length) {
    if (ser.available()) {
      buffer[count++] = ser.read();
    }
    if (millis() - start > timeout_ms) {
      return false;  // Истёк таймаут
    }
  }
  return true;
}

/**
 * @brief Ожидание появления в потоке UART специфического заголовка лидара.
 *        Заголовок определён в массиве LIDAR_HEADER.
 * 
 * @param ser Ссылка на Serial (обычно Serial1)
 * @return true, если заголовок найден, иначе false (по таймауту)
 */
bool waitForHeader(HardwareSerial &ser) {
  uint8_t matchPos = 0;
  uint32_t start = millis();

  while (true) {
    if (ser.available()) {
      uint8_t b = ser.read();
      if (b == LIDAR_HEADER[matchPos]) {
        matchPos++;
        if (matchPos == LIDAR_HEADER_LEN) {
          return true;
        }
      } else {
        matchPos = 0;
      }
    }
    if (millis() - start > 100) {
      return false;  // Таймаут
    }
  }
}

/**
 * @brief Преобразует «rawAngle» из пакета лидара в угол в градусах [0..360).
 * 
 * @param rawAngle Сырой угол (16 бит из пакета лидара)
 * @return Угол в градусах
 */
float decodeAngle(uint16_t rawAngle) {
  return (float)(rawAngle - 0xA000) / 64.0f;
}

/**
 * @brief Определяет сектор по углу.
 * 
 * @param angleDeg Угол в градусах [0..360)
 * @return Индекс сектора
 */
int angleToSector(float angleDeg) {
  float shifted = angleDeg + 15.0f;
  while (shifted < 0) shifted += 360.0f;
  while (shifted >= 360) shifted -= 360.0f;
  return (int)(shifted / 30.0f) % NUM_SECTORS;
}

/********************************************************
 *  ФУНКЦИИ УПРАВЛЕНИЯ МОТОРАМИ
 ********************************************************/

void stopMotors() {
  digitalWrite(MOTOR_LEFT_FORWARD_PIN, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD_PIN, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD_PIN, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD_PIN, LOW);
  Serial.println("Motors stopped");
}

void moveForward() {
  digitalWrite(MOTOR_LEFT_FORWARD_PIN, HIGH);
  digitalWrite(MOTOR_LEFT_BACKWARD_PIN, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD_PIN, HIGH);
  digitalWrite(MOTOR_RIGHT_BACKWARD_PIN, LOW);
  Serial.println("Moving forward");
}

void moveBackward() {
  digitalWrite(MOTOR_LEFT_FORWARD_PIN, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD_PIN, HIGH);
  digitalWrite(MOTOR_RIGHT_FORWARD_PIN, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD_PIN, HIGH);
  Serial.println("Moving backward");
}

void turnLeft() {
  digitalWrite(MOTOR_LEFT_FORWARD_PIN, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD_PIN, HIGH);
  digitalWrite(MOTOR_RIGHT_FORWARD_PIN, HIGH);
  digitalWrite(MOTOR_RIGHT_BACKWARD_PIN, LOW);
  Serial.println("Turning left");
}

void turnRight() {
  digitalWrite(MOTOR_LEFT_FORWARD_PIN, HIGH);
  digitalWrite(MOTOR_LEFT_BACKWARD_PIN, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD_PIN, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD_PIN, HIGH);
  Serial.println("Turning right");
}


/********************************************************
 *  ОСНОВНАЯ ФУНКЦИЯ ПАРСИНГА И ОБРАБОТКИ ДАННЫХ ЛИДАРА
 ********************************************************/

bool parseAndProcessPacket() {
  if (!waitForHeader(Serial1)) {
    return false;  // Заголовок не найден
  }

  uint8_t buffer[LIDAR_BODY_LEN];
  if (!readBytesWithTimeout(Serial1, buffer, LIDAR_BODY_LEN, 500)) {
    return false;  // Ошибка чтения пакета
  }

  uint16_t startAngleTmp = buffer[2] | (buffer[3] << 8);
  float startAngleDeg = decodeAngle(startAngleTmp);

  uint16_t endAngleTmp = buffer[28] | (buffer[29] << 8);
  float endAngleDeg = decodeAngle(endAngleTmp);

  float angleRange = endAngleDeg - startAngleDeg;
  float angleInc = angleRange / 8.0f;

  float packetAngles[8];
  for (int i = 0; i < 8; i++) {
    float angle = startAngleDeg + i * angleInc;
    packetAngles[i] = angle;
  }

  float tempSectorMin[NUM_SECTORS];
  for (int s = 0; s < NUM_SECTORS; s++) {
    tempSectorMin[s] = NO_VALUE;
  }

  for (int i = 0; i < 8; i++) {
    int sectorIndex = angleToSector(packetAngles[i]);
    float dist = (float)(buffer[4 + i * 3] | (buffer[5 + i * 3] << 8));
    if (dist < tempSectorMin[sectorIndex]) {
      tempSectorMin[sectorIndex] = dist;
    }
  }

  uint32_t now = millis();
  for (int s = 0; s < NUM_SECTORS; s++) {
    if (tempSectorMin[s] != NO_VALUE) {
      sectorDistances[s] = tempSectorMin[s];
      sectorUpdateTime[s] = now;
    }
  }

  // Обновление данных с проверкой на устаревшие значения
  for (int s = 0; s < NUM_SECTORS; s++) {
    if ((now - sectorUpdateTime[s]) > 500) {
      sectorDistances[s] = NO_VALUE;
    }
  }

  // Логика адаптивного движения
  bool obstacleDetected = false;
  bool canMoveForward = true;

  // Проверка на препятствия в секторах и принятие решения
  for (int s = 0; s < NUM_SECTORS; s++) {
    float dist = sectorDistances[s];

    if (dist != NO_VALUE) {
      if (dist < ALARM_DIST) {  // Слишком близко
        obstacleDetected = true;
        canMoveForward = false;

        if (s < NUM_SECTORS / 2) {
          // Препятствие слева, поворачиваем направо
          turnRight();
        } else {
          // Препятствие справа, поворачиваем налево
          turnLeft();
        }
        break;  // После одного поворота остановиться, чтобы избежать лишней обработки
      } else if (dist < WARNING_DIST) {  // Препятствие, но не слишком близко
        canMoveForward = false;

        if (s < NUM_SECTORS / 2) {
          // Препятствие справа, поворачиваем налево
          turnLeft();
        } else {
          // Препятствие слева, поворачиваем направо
          turnRight();
        }
        break;  // После одного поворота остановиться
      }
    }
  }

  // Если препятствий нет, или они были устранены, продолжаем движение вперёд
  if (!obstacleDetected && canMoveForward) {
    moveForward();
  }

  return true;
}
void sendLidarData() {
  Serial.print("LIDAR:");
  for (int i = 0; i < 12; i++) {
    Serial.print(sectorDistances[i]);
    if (i < 11) Serial.print(",");
  }
  Serial.println(";");
}

void lidarControl() {
  for (int s = 0; s < NUM_SECTORS; s++) {
  float dist = sectorDistances[s];
  if (dist != NO_VALUE && dist < ALARM_DIST) {
    moveForward();  // Если расстояние меньше порога, двигаемся вперед
  } else if (dist != NO_VALUE && dist < WARNING_DIST) {
    turnLeft();  // Если расстояние в пределах предупреждения, поворачиваем влево
  } else {
    stopMotors();  // В остальных случаях останавливаемся
  }
}
}


void updateState() {
  // Проверка сектора для движения вперёд
  if (sectorDistances[0] > 30 && sectorDistances[1] > 30 && sectorDistances[11] > 30) {
    state = FORWARD;  // Двигаться вперёд, если в передних секторах достаточно места
  } 
  // Проверка для движения назад (при блокировке передних секторов)
  else if (sectorDistances[5] < 15 && sectorDistances[6] < 15 && sectorDistances[7] < 15) {
    state = BACKWARD;  // Двигаться назад, если сзади слишком мало места
  }
  // Логика для поворота направо
  else if (sectorDistances[0] < sectorDistances[11]) {
    state = RIGHT;  // Поворачиваем направо, если впереди справа меньше расстояние
  } 
  // Логика для поворота налево
  else if (sectorDistances[0] > sectorDistances[11]) {
    state = LEFT;  // Поворачиваем налево, если слева меньше расстояние
  } 
  // Стратегия для остановки или дополнительного маневра
  else {
    state = IDLE;  // Останавливаемся, если нет явных препятствий
  }
}

void readLidarData() {
  // Тут код который читает данные из лидара
  // Например:
  if (Serial2.available()) {
    // читаем данные из лидара
  }
}

void setup() {
  Serial.begin(BAUDRATE);
  Serial1.begin(BAUDRATE, SERIAL_8N1, LIDAR_RX_PIN);

  pinMode(MOTOR_LEFT_FORWARD_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD_PIN, OUTPUT);

}


void loop() {
  // Параллельное выполнение всех шагов: обработка данных лидара, обновление состояния, управление движением
  parseAndProcessPacket(); // Обработка данных с лидара
  sendLidarData();         // Отправка данных лидара для мониторинга
  readLidarData();         // Чтение новых данных с лидара (если требуется)
  updateState();           // Обновление состояния на основе данных лидара
  lidarControl();          // Управление движением робота

  delay(100); // Пауза между циклами
}