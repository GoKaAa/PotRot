const int potPin = A3;       // Analog pin connected to potentiometer
const int buttonPin = 2;     // Digital pin connected to button
const int encoderResolution = 1024;  // Resolution of the rotary encoder
const float alpha = 0.1;     // Smoothing factor for the low-pass filter
const int deadZone = 5;      // Dead zone range to ignore small fluctuations
const int debounceDelay = 50;// Debounce delay for button press
const int clickThreshold = 500; // Time threshold to differentiate between clicks

int lastPosition = 0;
float filteredValue = 0.0;
int minValue = 0;
int maxValue = 1023;
bool calibrateMode = false;
unsigned long lastDebounceTime = 0;
bool buttonState = HIGH;
bool lastButtonState = HIGH;
int clickCount = 0;
unsigned long lastClickTime = 0;

int maxSelections = 5;

void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT_PULLUP);  // Set button pin as input with pull-up resistor
  lastPosition = analogRead(potPin);
  filteredValue = lastPosition;
  maxSelections++;
}

void loop() {
  int rawValue = analogRead(potPin);

  // Apply low-pass filter to reduce noise
  filteredValue = alpha * rawValue + (1 - alpha) * filteredValue;

  // Apply dead zone around the central value
  if (abs(filteredValue - lastPosition) < deadZone) {
    return;
  }

  // Non-linear mapping example (using quadratic mapping)
  float normalizedValue = (filteredValue - minValue) / (float)(maxValue - minValue);
  int position = pow(normalizedValue, 2) * encoderResolution;

  if (position != lastPosition) {
    int mappedPosition = map(position, 0, 1024, 1, maxSelections);
    int mappedLastPosition = map(lastPosition, 0, 1024, 1, maxSelections);

    if (mappedPosition != mappedLastPosition) {
      if (mappedPosition <= (maxSelections-1)) {
        Serial.print("Position: ");
        Serial.println(mappedPosition);
        lastPosition = position;
      }
    }
  }

  // Button handling for clicks and calibration mode
  handleButton();

  delay(10);  // Small delay to stabilize readings
}

void handleButton() {
  bool reading = digitalRead(buttonPin);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {
        // Register a click
        unsigned long currentTime = millis();
        if (currentTime - lastClickTime > clickThreshold) {
          clickCount = 0;
        }
        lastClickTime = currentTime;
        clickCount++;
      }
    }
  }

  // Handle clicks
  if (clickCount > 0 && millis() - lastClickTime > clickThreshold) {
    if (clickCount == 1) {
      Serial.println("Single click detected");
    } else if (clickCount == 2) {
      Serial.println("Double click detected");
    } else if (clickCount == 3) {
      // Enter calibration mode
      if (!calibrateMode) {
        calibrateMode = true;
        minValue = analogRead(potPin);
        Serial.println("Calibration started. Rotate to maximum position and press button 3 times.");
      } else {
        // Exit calibration mode and set min and max values
        maxValue = analogRead(potPin);
        calibrateMode = false;
        Serial.println("Calibration completed.");
      }
    }
    clickCount = 0;
  }

  lastButtonState = reading;
}
