#include <Pixy2.h>
Pixy2 pixy;

// Signature numbers (must match PixyMon)
#define SIG_BLUE    1
#define SIG_YELLOW  2
#define SIG_RED     3
#define SIG_ORANGE  4
#define SIG_GREEN   5

#define NUM_SIGS 6   // index 0 unused

// Thresholds (tune these)
#define MIN_AREA   3000
#define GOOD_AREA  8000

const char* sigNames[NUM_SIGS] = {
  "None",
  "Blue",
  "Yellow",
  "Red",
  "Orange",
  "Green"
};

void setup() {
  Serial.begin(115200);
  Serial.println("Pixy2 Dominant Color Detector Starting...");
  pixy.init();
}

void loop() {

  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks == 0) {
    Serial.println("No colors detected");
    delay(500);
    return;
  }

  // ----------------------------------
  // Accumulate total area per color
  // ----------------------------------
  uint32_t totalArea[NUM_SIGS] = {0};

  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    int sig = pixy.ccc.blocks[i].m_signature;
    uint32_t area = (uint32_t)pixy.ccc.blocks[i].m_width * (uint32_t)pixy.ccc.blocks[i].m_height;

    if (sig >= 1 && sig <= 5) {
      totalArea[sig] += area;
    }
  }

  // ----------------------------------
  // Find most prominent color
  // ----------------------------------
  int dominantSig = 0;
  uint32_t maxArea = 0;

  for (int sig = 1; sig < NUM_SIGS; sig++) {
    if (totalArea[sig] > maxArea) {
      maxArea = totalArea[sig];
      dominantSig = sig;
    }
  }

  if (dominantSig == 0 && pixy.ccc.numBlocks == 1) {
    dominantSig = pixy.ccc.blocks[0].m_signature;
    maxArea = (uint32_t)pixy.ccc.blocks[0].m_width *
              (uint32_t)pixy.ccc.blocks[0].m_height;
  }

  if (dominantSig < 1 || dominantSig > 5) {
    Serial.println("No valid dominant color");
    delay(500);
    return;
  }
  // ----------------------------------
  // Output results
  // ----------------------------------
  Serial.print("Dominant Color: ");
  Serial.print(sigNames[dominantSig]);
  Serial.print(" | Total Area: ");
  Serial.println(maxArea);

  Serial.println("----------------------");
  delay(1000);
}
