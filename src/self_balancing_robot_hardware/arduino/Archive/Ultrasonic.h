float getDistance() {
    // Clear trigger pin
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    
    // Send 10μs pulse
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // Measure the response
    long duration = pulseIn(ECHO_PIN, HIGH);
    
    // Calculate distance in centimeters
    // Using speed of sound = 340 m/s = 0.034 cm/μs
    float distance = duration * 0.034 / 2;
    
    return distance;
}