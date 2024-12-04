#define BUTTON_PIN 13
#define LED_PIN 14

bool ledState = LOW;                     //track the state of led (on or off)
bool lastButtonState = HIGH;              // store the previous state of the button to detect changes
unsigned long lastDebounceTime = 0;      // button by tracking the last time its state changed
const unsigned long debounceDealy = 50;  // delay or interval

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLDOWN);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, ledState);
}
void loop() {
  int buttonState = digitalRead(BUTTON_PIN);  //when the button pressed it will be HIGH cuz pull down connnection
  Serial.print(buttonState);

  if (buttonState != lastButtonState) {  //initial state will be LOW after
    lastDebounceTime = millis();         // Reset the debounce timer
    Serial.print("It's not equal ");
      Serial.println(lastDebounceTime);
  }
  if ((millis() - lastDebounceTime) > debounceDealy) {
    if(buttonState == LOW){
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
    }
  }

  lastButtonState = buttonState; // Update the last button state
}
