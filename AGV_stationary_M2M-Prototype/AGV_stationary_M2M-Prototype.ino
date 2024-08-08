#include <Pixy2.h>

Pixy2 pixy;
int i;

void setup()
{
  Serial.begin(115200);   // Debugging
  Serial3.begin(9600);    // Communication with ESP32
  Serial.print("Starting...\n");

  pixy.init();
}

void loop()
{
  piXy();
}

void piXy() {
  // grab blocks!
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks)
  {
    int signature = pixy.ccc.blocks[i].m_signature;
    Serial.print("SIGNATURE:"); Serial.println(signature);

    char dataToSend;
    if (signature == 3) {
      dataToSend = 'A';
    } else {
      dataToSend = 'B';
    }

    // Send character to ESP32 via Serial3
    Serial3.print(dataToSend);
    Serial.print("Written: "); Serial.println(dataToSend);
  }
}
