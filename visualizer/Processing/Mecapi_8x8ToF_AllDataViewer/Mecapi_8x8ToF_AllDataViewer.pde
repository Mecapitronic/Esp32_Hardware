// VL53L8CX visualisation Processing - réception binaire rapide
import processing.serial.*;
import java.io.PrintWriter;
import java.io.ByteArrayOutputStream;
import java.util.Arrays;
import controlP5.*;

Serial myPort;
ControlP5 cp5;

String[] matrixKeys = {
  "status", "distance", "sigma", "reflectance", "nb_spads", "distance_interp", "ambient", "signal"
};

HashMap<String, float[]> matrices = new HashMap<String, float[]>();
HashMap<String, Boolean> showMatrix = new HashMap<String, Boolean>();
HashMap<String, String> units = new HashMap<String, String>();
HashMap<String, Float> manualMin = new HashMap<String, Float>();
HashMap<String, Float> manualMax = new HashMap<String, Float>();
HashMap<String, Integer> colorLow = new HashMap<String, Integer>();
HashMap<String, Integer> colorHigh = new HashMap<String, Integer>();

int cols = 8;
int rows = 8;
int cellSize = 40;
int margin = 20;

ByteArrayOutputStream incomingBuffer = new ByteArrayOutputStream();
boolean receivingData = false;
String currentKey = null;
int expectedDataLength = 0;
int currentKeyLength = 7;

boolean paused = false;
int buttonWidth = 100;
int buttonHeight = 40;
int buttonSpacing = 20;
int rightPanelX = 1380; // Position X des boutons à droite (ajuster selon la taille de la fenêtre)
int playPauseY = 20;
int screenshotY = playPauseY + buttonHeight + buttonSpacing;
int rebootY = screenshotY + buttonHeight + buttonSpacing; // Position du bouton reboot

// Mapping des status vers leur description
String[] statusDescriptions = {
  "0: Ranging data are not updated",
  "1: Signal rate too low on SPAD array",
  "2: Target phase",
  "3: Sigma estimator too high",
  "4: Target consistency failed",
  "5: Range valid",
  "6: Wrap around not performed (first range)",
  "7: Rate consistency failed",
  "8: Signal rate too low for the current target",
  "9: Range valid with large pulse (merged target)",
  "10: Range valid, but no target detected at previous range",
  "11: Measurement consistency failed",
  "12: Target blurred by another one (sharpener)",
  "13: Target detected but inconsistent data (often secondary)"
};
String status255 = "255: No target detected (only if number of targets detected is enabled)";

void setup() {
  size(1500, 740);
  surface.setResizable(true);
  surface.setLocation(10, 10);
  //myPort = new Serial(this, Serial.list()[0], 921600);
  myPort = new Serial(this, "COM7", 921600);
  cp5 = new ControlP5(this);

  for (int i = 0; i < matrixKeys.length; i++) {
    String key = matrixKeys[i];
    matrices.put(key, new float[cols * rows]);
    showMatrix.put(key, true);
    // Valeurs min/max en dur
    if (key.equals("ambient")) {
      manualMin.put(key, 0.0);
      manualMax.put(key, 1000.0);
    } else if (key.equals("nb_spads")) {
      manualMin.put(key, 0.0);
      manualMax.put(key, 4096.0);
    } else if (key.equals("signal")) {
      manualMin.put(key, 0.0);
      manualMax.put(key, 100000.0);
    } else if (key.equals("sigma")) {
      manualMin.put(key, 0.0);
      manualMax.put(key, 100.0);
    } else if (key.equals("distance")) {
      manualMin.put(key, 0.0);
      manualMax.put(key, 800.0);
    } else if (key.equals("distance_interp")) {
      manualMin.put(key, 0.0);
      manualMax.put(key, 800.0);
    } else if (key.equals("status")) {
      manualMin.put(key, 0.0);
      manualMax.put(key, 255.0);
    } else if (key.equals("reflectance")) {
      manualMin.put(key, 0.0);
      manualMax.put(key, 100.0);
    }
    colorLow.put(key, color(0, 0, 255));
    colorHigh.put(key, color(255, 0, 0));
  }

  units.put("ambient", "kcps/spad");
  units.put("nb_spads", "spads");
  units.put("signal", "kcps/spad");
  units.put("sigma", "mm");
  units.put("distance", "mm");
  units.put("distance_interp", "mm");
  units.put("status", "code");
  units.put("reflectance", "%");

  frameRate(5);
}


void draw() {
  background(30);
  // Mettre à jour la matrice interpolée avant affichage
  updateDistanceInterp();
  int leftMargin = 20;
  int topMargin = 30;
  int visibleCount = 0;
  for (String key : matrixKeys) {
    int colIndex = visibleCount % 4;
    int rowIndex = visibleCount / 4;
    int panelX = leftMargin + colIndex * (cols * cellSize + margin);
    int panelY = topMargin + rowIndex * (rows * cellSize + margin + 30);
    if (key.equals("distance_interp")) {
      drawInterpMatrix(panelX, panelY, matrices.get(key), key);
    } else {
      drawMatrix(panelX, panelY, matrices.get(key), key);
    }
    visibleCount++;
  }
  drawRightButtons();
}

// --- Bilinear interpolation for distance_interp ---
void updateDistanceInterp() {
  float[] src = matrices.get("distance");
  float[] dst = matrices.get("distance_interp");
  if (src == null) return;
  int srcW = 8, srcH = 8;
  int dstW = 32, dstH = 32;
  // (Re)allouer dst si nécessaire
  if (dst == null || dst.length != dstW * dstH) {
    dst = new float[dstW * dstH];
    matrices.put("distance_interp", dst);
  }
  // Interpolation bilinéaire 8x8 -> 32x32
  for (int y = 0; y < dstH; y++) {
    float gy = map(y, 0, dstH-1, 0, srcH-1);
    int y0 = int(floor(gy));
    int y1 = min(y0+1, srcH-1);
    float fy = gy - y0;
    for (int x = 0; x < dstW; x++) {
      float gx = map(x, 0, dstW-1, 0, srcW-1);
      int x0 = int(floor(gx));
      int x1 = min(x0+1, srcW-1);
      float fx = gx - x0;
      float v00 = src[y0*srcW + x0];
      float v10 = src[y0*srcW + x1];
      float v01 = src[y1*srcW + x0];
      float v11 = src[y1*srcW + x1];
      float interp = 
        (1-fx)*(1-fy)*v00 +
        fx*(1-fy)*v10 +
        (1-fx)*fy*v01 +
        fx*fy*v11;
      dst[y*dstW + x] = interp;
    }
  }
}

// --- Affichage matrice interpolée 32x32, même taille visuelle que 8x8 ---
void drawInterpMatrix(int x0, int y0, float[] data, String key) {
  int interpW = 32, interpH = 32;
  int displayW = cols * cellSize;
  int displayH = rows * cellSize;
  float cellW = displayW / float(interpW);
  float cellH = displayH / float(interpH);

  // Calcul du min/max sur toutes les valeurs (y compris non valides)
  float minVal = Float.MAX_VALUE;
  float maxVal = -Float.MAX_VALUE;
  for (int i = 0; i < data.length; i++) {
    float value = data[i];
    if (!Float.isNaN(value)) {
      if (value < minVal) minVal = value;
      if (value > maxVal) maxVal = value;
    }
  }
  // fallback si aucun point valide
  if (minVal == Float.MAX_VALUE || maxVal == -Float.MAX_VALUE) {
    minVal = manualMin.get("distance_interp");
    maxVal = manualMax.get("distance_interp");
  }

  int cLow = colorLow.get("distance");
  int cHigh = colorHigh.get("distance");
  pushMatrix();
  translate(x0, y0);
  fill(255);
  textAlign(CENTER, BOTTOM);
  textSize(14);
  text("distance_interp [mm]", displayW / 2, -10);

  // Affichage des couleurs sans texte
  for (int y = 0; y < interpH; y++) {
    for (int x = 0; x < interpW; x++) {
      int idx = y * interpW + x;
      float value = data[idx];
      if (Float.isNaN(value)) {
        fill(40); // gris foncé pour les points NaN
      } else {
        float norm = constrain((maxVal == minVal) ? 0 : map(value, minVal, maxVal, 0, 1), 0, 1);
        float hue = map(norm, 0, 1, 0, 170); // même palette que distance
        colorMode(HSB, 255);
        fill(hue, 255, 255);
        colorMode(RGB, 255);
      }
      noStroke();
      rect(x * cellW, y * cellH, cellW, cellH);
    }
  }
  popMatrix();
}

void drawMatrix(int x0, int y0, float[] data, String key) {
  pushMatrix();
  translate(x0, y0);
  float minVal = manualMin.get(key);
  float maxVal = manualMax.get(key);
  int cLow = colorLow.get(key);
  int cHigh = colorHigh.get(key);
  fill(255);
  textAlign(CENTER, BOTTOM);
  textSize(14);
  text(key + " [" + units.get(key) + "]", (cols * cellSize) / 2, -10);

  // Affichage info status au survol
  boolean showStatusTooltip = false;
  int statusIdx = -1;
  int statusVal = -1;
  int mouseCellX = -1, mouseCellY = -1;
  int globalCellX = -1, globalCellY = -1;
  if (key.equals("status")) {
    globalCellX = (mouseX - x0);
    globalCellY = (mouseY - y0);
    if (globalCellX >= 0 && globalCellY >= 0 && globalCellX < cols * cellSize && globalCellY < rows * cellSize) {
      mouseCellX = globalCellX / cellSize;
      mouseCellY = globalCellY / cellSize;
      statusIdx = mouseCellY * cols + mouseCellX;
      if (statusIdx >= 0 && statusIdx < data.length) {
        statusVal = int(data[statusIdx]);
        showStatusTooltip = true;
      }
    }
  }

  // Recherche du min/max relatifs de la matrice (hors NaN)
  float relMin = Float.MAX_VALUE;
  float relMax = -Float.MAX_VALUE;
  // Pour la matrice distance, ne prendre en compte que les valeurs valides (status == 5)
  if (key.equals("distance")) {
    float[] statusMatrix = matrices.get("status");
    for (int i = 0; i < data.length; i++) {
      float v = data[i];
      boolean valid = false;
      if (!Float.isNaN(v) && statusMatrix != null && i < statusMatrix.length && int(statusMatrix[i]) == 5) {
        valid = true;
      }
      if (valid) {
        if (v < relMin) relMin = v;
        if (v > relMax) relMax = v;
      }
    }
    // Si aucune valeur valide trouvée, fallback sur tout
    if (relMin == Float.MAX_VALUE || relMax == -Float.MAX_VALUE) {
      relMin = Float.MAX_VALUE;
      relMax = -Float.MAX_VALUE;
      for (int i = 0; i < data.length; i++) {
        float v = data[i];
        if (!Float.isNaN(v)) {
          if (v < relMin) relMin = v;
          if (v > relMax) relMax = v;
        }
      }
    }
  } else {
    for (int i = 0; i < data.length; i++) {
      float v = data[i];
      if (!Float.isNaN(v)) {
        if (v < relMin) relMin = v;
        if (v > relMax) relMax = v;
      }
    }
  }

  // --- Calcul des positions min/max par colonne, uniquement pour les valeurs encadrées ---
  int[] minRowPerCol = new int[cols];
  int[] maxRowPerCol = new int[cols];
  int minCount = 0, maxCount = 0;
  float minSum = 0, maxSum = 0;
  for (int x = 0; x < cols; x++) {
    int minRow = -1, maxRow = -1;
    for (int y = 0; y < rows; y++) {
      int idx = y * cols + x;
      float v = data[idx];
      // Vérifie si la valeur est encadrée (min ou max relatifs, donc encadrée)
      boolean isRelMin = abs(v - relMin) < 1e-6;
      boolean isRelMax = abs(v - relMax) < 1e-6;
      if (isRelMin) { minRow = y; }
      if (isRelMax) { maxRow = y; }
    }
    minRowPerCol[x] = minRow;
    maxRowPerCol[x] = maxRow;
    if (minRow != -1) { minSum += minRow; minCount++; }
    if (maxRow != -1) { maxSum += maxRow; maxCount++; }
  }
  float meanMin = (minCount > 0) ? minSum / minCount : 0;
  float meanMax = (maxCount > 0) ? maxSum / maxCount : 0;

  for (int y = 0; y < rows; y++) {
    for (int x = 0; x < cols; x++) {
      int idx = y * cols + x;
      float value = data[idx];
      if (Float.isNaN(value)) value = 0;
      int cellStatusVal = (key.equals("status")) ? int(value) : -1;

      // Couleur de fond selon la confiance pour la matrice status
      if (key.equals("status")) {
        if (cellStatusVal == 5) {
          fill(40, 180, 40); // vert
        } else if (cellStatusVal == 6 || cellStatusVal == 9) {
          fill(230, 160, 30); // orange
        } else if (cellStatusVal == 255) {
          fill(0, 0, 0); // noir
        } else {
          fill(200, 40, 40); // rouge
        }
      } else if (key.equals("distance")) {
        // Fond gris foncé si status correspondant n'est pas valide (vert)
        int statusValDist = 5; // valeur par défaut valide
        if (matrices.containsKey("status")) {
          float[] statusMatrix = matrices.get("status");
          if (statusMatrix != null && idx >= 0 && idx < statusMatrix.length) {
            statusValDist = int(statusMatrix[idx]);
          }
        }
        if (statusValDist == 5) {
          // Rainbow normal inversé : proche = rouge (0), loin = violet (170)
          float norm = constrain((maxVal == minVal) ? 0 : map(value, minVal, maxVal, 0, 1), 0, 1);
          float hue = map(norm, 0, 1, 0, 170); // 0=proche=rouge, 1=loin=violet
          colorMode(HSB, 255);
          fill(hue, 255, 255);
          colorMode(RGB, 255);
        } else {
          fill(40); // gris foncé
        }
      } else if (key.equals("sigma")) {
        // Rainbow normal : faible sigma = vert (bon), fort sigma = rouge (mauvais)
        float norm = constrain((maxVal == minVal) ? 0 : map(value, minVal, maxVal, 0, 1), 0, 1);
        float hue = map(norm, 0, 1, 85, 0); // 0=faible=vert, 1=fort=rouge
        colorMode(HSB, 255);
        fill(hue, 255, 255);
        colorMode(RGB, 255);
      } else {
        float normVal = constrain((maxVal == minVal) ? 0 : map(value, minVal, maxVal, 0, 1), 0, 1);
        int c = lerpColor(cLow, cHigh, normVal);
        fill(c);
      }
      stroke(50);
      rect(x * cellSize, y * cellSize, cellSize, cellSize);

      // Encadrement si valeur == min/max relatifs de la matrice
      boolean isRelMin = abs(value - relMin) < 1e-6;
      boolean isRelMax = abs(value - relMax) < 1e-6;
      if (isRelMin || isRelMax) {
        strokeWeight(3);
        if (isRelMin && isRelMax) {
          stroke(255, 0, 255); // violet si min==max
        } else if (isRelMin) {
          stroke(0, 255, 255); // cyan pour min relatif
        } else {
          stroke(255, 255, 0); // jaune pour max relatif
        }
        noFill();
        rect(x * cellSize + 2, y * cellSize + 2, cellSize - 4, cellSize - 4);
        strokeWeight(1);
      }

      fill(255);
      textSize(14);
      textAlign(CENTER, CENTER);
      text(nf(value, 0, 0), x * cellSize + cellSize / 2, y * cellSize + cellSize / 2);
    }
  }

  // --- Afficher un trait horizontal sous chaque colonne qui contient un min ou un max encadré ---
  int yBase = rows * cellSize + 2;
  for (int x = 0; x < cols; x++) {
    // Trait min (cyan) si la colonne contient un min encadré
    if (minRowPerCol[x] != -1) {
      int x0min = x * cellSize + 4;
      int x1min = (x + 1) * cellSize - 4;
      int yLine = yBase + 2;
      stroke(0, 255, 255);
      line(x0min, yLine, x1min, yLine);
    }
    // Trait max (jaune) si la colonne contient un max encadré
    if (maxRowPerCol[x] != -1) {
      int x0max = x * cellSize + 4;
      int x1max = (x + 1) * cellSize - 4;
      int yLine = yBase + 8;
      stroke(255, 255, 0);
      line(x0max, yLine, x1max, yLine);
    }
  }
  stroke(50);

  popMatrix();

  // Affichage du tooltip status simplifié (en dehors du push/popMatrix pour coordonnées globales)
  if (showStatusTooltip) {
    String desc = "";
    if (statusVal >= 0 && statusVal < statusDescriptions.length) {
      desc = statusDescriptions[statusVal];
    } else if (statusVal == 255) {
      desc = status255;
    } else {
      desc = statusVal + ": Unknown";
    }

    // Déterminer la couleur de fond selon la confiance
    color bg;
    if (statusVal == 5) {
      bg = color(40, 180, 40, 230); // vert
    } else if (statusVal == 6 || statusVal == 9) {
      bg = color(230, 160, 30, 230); // orange
    } else if (statusVal == 255) {
      bg = color(0, 0, 0, 230); // noir
    } else {
      bg = color(200, 40, 40, 230); // rouge
    }

    int tooltipX = constrain(mouseX + 10, 0, width - 120);
    int tooltipY = constrain(mouseY + 10, 0, height - 85);
    int boxW = 120;
    int boxH = 85;
    fill(bg);
    stroke(200);
    rect(tooltipX, tooltipY, boxW, boxH, 8);
    fill(255);
    textAlign(LEFT, TOP);
    textSize(15);
    text(desc, tooltipX + 8, tooltipY + 8, boxW - 16, boxH - 16);
  }
}

void drawRightButtons() {
  // Play/Pause button
  fill(paused ? color(200, 60, 60) : color(60, 200, 60));
  stroke(255);
  rect(rightPanelX, playPauseY, buttonWidth, buttonHeight, 10);
  fill(255);
  textAlign(CENTER, CENTER);
  textSize(16);
  text(paused ? "Play" : "Pause", rightPanelX + buttonWidth / 2, playPauseY + buttonHeight / 2);

  // Screenshot button
  fill(80, 120, 220);
  stroke(255);
  rect(rightPanelX, screenshotY, buttonWidth, buttonHeight, 10);
  fill(255);
  textAlign(CENTER, CENTER);
  textSize(16);
  text("Screenshot", rightPanelX + buttonWidth / 2, screenshotY + buttonHeight / 2);

  // Reboot button
  fill(220, 120, 80);
  stroke(255);
  rect(rightPanelX, rebootY, buttonWidth, buttonHeight, 10);
  fill(255);
  textAlign(CENTER, CENTER);
  textSize(16);
  text("Reboot", rightPanelX + buttonWidth / 2, rebootY + buttonHeight / 2);
}

void mousePressed() {
  // Play/Pause button
  if (mouseX >= rightPanelX && mouseX <= rightPanelX + buttonWidth &&
      mouseY >= playPauseY && mouseY <= playPauseY + buttonHeight) {
    paused = !paused;
    return;
  }
  // Screenshot button
  if (mouseX >= rightPanelX && mouseX <= rightPanelX + buttonWidth &&
      mouseY >= screenshotY && mouseY <= screenshotY + buttonHeight) {
    saveFrame("Test-###.png");
    return;
  }
  // Reboot button
  if (mouseX >= rightPanelX && mouseX <= rightPanelX + buttonWidth &&
      mouseY >= rebootY && mouseY <= rebootY + buttonHeight) {
    // Relancer la fenêtre Processing (cross-platform)
    String javaBin = System.getProperty("java.home") + "/bin/java";
    String jarPath = System.getProperty("java.class.path");
    String className = this.getClass().getName();
    try {
      Runtime.getRuntime().exec(new String[] { javaBin, "-cp", jarPath, className });
    } catch (Exception e) {
      println("Erreur reboot: " + e);
    }
    exit();
    return;
  }
  // ...existing code if you want to keep other mousePressed logic...
}

void serialEvent(Serial port) {
  if (paused) return; // Ignore serial data when paused
  while (port.available() > 0) {
    byte b = (byte) port.read();

    if (!receivingData) {
      incomingBuffer.write(b);
      if (incomingBuffer.size() == currentKeyLength) {
        String prefix = new String(incomingBuffer.toByteArray());
        incomingBuffer.reset();

        if (prefix.startsWith("VL53")) {
          String suffix = prefix.substring(4);
          switch (suffix) {
            case "amb":  currentKey = "ambient";     expectedDataLength = 64 * 4; break;
            case "tar":  currentKey = "nb_targets";  expectedDataLength = 64;     break;
            case "spa":  currentKey = "nb_spads";    expectedDataLength = 64 * 4; break;
            case "sps":  currentKey = "signal";      expectedDataLength = 64 * 4; break;
            case "sig":  currentKey = "sigma";       expectedDataLength = 64 * 2; break;
            case "dis":  currentKey = "distance";    expectedDataLength = 64 * 2; break;
            case "sta":  currentKey = "status";      expectedDataLength = 64;     break;
            case "ref":  currentKey = "reflectance"; expectedDataLength = 64;     break;
            default:
              currentKey = null;
              expectedDataLength = 0;
          }
          if (expectedDataLength > 0) {
            receivingData = true;
          }
        }
      }
    } else {
      incomingBuffer.write(b);
      if (incomingBuffer.size() >= expectedDataLength) {
        byte[] buf = incomingBuffer.toByteArray();
        try {
          parseBlock(currentKey, buf);
        } catch (Exception e) {
          println("Erreur parseBlock " + currentKey + ": " + e);
        }
        incomingBuffer.reset();
        receivingData = false;
        currentKey = null;
        expectedDataLength = 0;
      }
    }
  }
}

void parseBlock(String key, byte[] buf) {
  float[] result = new float[64];
  switch (key) {
    case "ambient":
    case "nb_spads":
    case "signal":
      for (int i = 0; i < 64; i++) {
        int idx = i * 4;
        long val = ((long)(buf[idx + 3] & 0xFF) << 24) |
                   ((long)(buf[idx + 2] & 0xFF) << 16) |
                   ((long)(buf[idx + 1] & 0xFF) << 8) |
                   (long)(buf[idx] & 0xFF);
        result[i] = (float) val;
      }
      break;
    case "sigma":
      for (int i = 0; i < 64; i++) {
        int idx = i * 2;
        int val = ((buf[idx + 1] & 0xFF) << 8) | (buf[idx] & 0xFF);
        result[i] = val;
      }
      break;
    case "distance":
      for (int i = 0; i < 64; i++) {
        int idx = i * 2;
        short val = (short)(((buf[idx + 1] & 0xFF) << 8) | (buf[idx] & 0xFF));
        result[i] = val;
      }
      break;
    case "nb_targets":
    case "status":
    case "reflectance":
      for (int i = 0; i < 64; i++) {
        result[i] = buf[i] & 0xFF;
      }
      break;
  }
  if (matrices.containsKey(key)) matrices.put(key, result);
}
