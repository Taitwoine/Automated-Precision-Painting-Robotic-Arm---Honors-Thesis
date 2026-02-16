#include <Pixy2.h>

Pixy2 pixy;

// Define color signatures
#define TARGET_COLOR_SIG 1
#define UNDERPAINT_SIG   2
#define WRONG_COLOR_SIG  3

// Thresholds (tune experimentally)
#define MIN_TARGET_AREA  3000   // block width * height
#define GOOD_TARGET_AREA 8000   // strong coverage

void setup() {
  Serial.begin(115200);
  Serial.println("Pixy2 Paint Inspector Starting...");
  pixy.init();
}

void loop() {

  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks == 0) {
    Serial.println("No paint detected");
    return;
  }

  int bestIndex = -1;
  int maxArea = 0;

  // Find largest detected block
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    int area = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
    if (area > maxArea) {
      maxArea = area;
      bestIndex = i;
    }
  }

  if (bestIndex == -1) return;

  int sig = pixy.ccc.blocks[bestIndex].m_signature;
  int area = maxArea;

  Serial.print("Signature: ");
  Serial.print(sig);
  Serial.print(" Area: ");
  Serial.println(area);

  // ---------------- Paint Logic ----------------

  if (sig == TARGET_COLOR_SIG) {
    if (area < MIN_TARGET_AREA) {
      Serial.println("⚠ Target color detected but coverage is weak → ADD PAINT");
    }
    else if (area < GOOD_TARGET_AREA) {
      Serial.println("🟡 Almost done → ONE MORE PASS");
    }
    else {
      Serial.println("✅ TARGET COLOR REACHED → STOP PAINTING");
    }
  }
  else if (sig == UNDERPAINT_SIG) {
    Serial.println("❌ UNDERPAINTED → KEEP PAINTING");
  }
  else if (sig == WRONG_COLOR_SIG) {
    Serial.println("❌ WRONG COLOR DETECTED → ALERT");
  }
  else {
    Serial.println("Unknown color signature");
  }

  delay(1000);
}
