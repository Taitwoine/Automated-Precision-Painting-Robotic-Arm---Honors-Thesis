#include <Pixy2.h>
Pixy2 pixy;

// Signature numbers (match PixyMon training order!)
#define SIG_BLUE   1
#define SIG_BLACK  2
#define SIG_RED    3
#define SIG_WHITE  4
#define SIG_GREEN  5

const char* sigNames[] = {
  "None",
  "Blue",
  "Yellow",
  "Red",
  "Orange",
  "Green"
};

void setup() {
  Serial.begin(115200);
  Serial.println("Pixy2 Color Detector Starting...");
  pixy.init();
}

void loop() {

  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks == 0) {
    Serial.println("No colors detected");
    delay(500);
    return;
  }

  Serial.print("Detected blocks: ");
  Serial.println(pixy.ccc.numBlocks);

  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    int sig = pixy.ccc.blocks[i].m_signature;
    int area = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;

    Serial.print("Block ");
    Serial.print(i);
    Serial.print(" → ");
    Serial.print(sigNames[sig]);
    Serial.print(" | Area = ");
    Serial.print(area);
    Serial.println(" px^2");
  }

  Serial.println("----------------------");
  delay(2000);
}
