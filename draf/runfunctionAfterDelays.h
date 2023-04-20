
unsigned long timer = 0;
void runFunctionAfterDelay(unsigned long delay, void (*function)()) {
  // Check if the delay has passed
  if (millis() - timer > delay) {
    // Run the function
    function();

    // Reset the timer
    timer = millis();
  }
}