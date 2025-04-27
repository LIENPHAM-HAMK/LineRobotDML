package today22;

import lejos.hardware.motor.Motor;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.Color;

/**
 * LineFollowerRobot demonstrates a simple line following robot with object detection.
 */

public class 22 {
    // Define sensor and motor settings
    private static final float CRITICAL_DISTANCE = 0.10f; // 15 cm
    private static final float SLOW_DISTANCE = 0.25f;
    private static final int BASE_SPEED = 200;
    private static final int TURN_SPEED = 100;
    private static final int SLOW_SPEED = 100;
    private static final int ROTATE_ANGLE= 180;

    private EV3LargeRegulatedMotor leftMotor;
    private EV3LargeRegulatedMotor rightMotor;
    private EV3ColorSensor colorSensor;

    // PID parameters
    private double Kp = 1.0; // Proportional gain
    private double Ki = 0.0; // Integral gain
    private double Kd = 0.5; // Derivative gain
    
    private double previousError = 0;
    private double integral = 0;

    public PIDLineFollower() {
        leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
        rightMotor = new EV3LargeRegulatedMotor(MotorPort.B);
        colorSensor = new EV3ColorSensor(BrickFinder.getDefault().getPort("S1"));
    }

    public static void main(String[] args)  {

        // Initialize color and ultrasonic sensors
        final EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);
        final EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S2);

        // Fetch color sensor in black mode (for line intensity)
        final SampleProvider colorProvider = colorSensor.getRedMode();
        final float[] colorSample = new float[colorProvider.sampleSize()];

        // Fetch ultrasonic sensor in distance mode
        final SampleProvider distanceProvider = ultrasonicSensor.getDistanceMode();
        final float[] distanceSample = new float[distanceProvider.sampleSize()];

        // PID calculations
        integral += error; // Accumulate the integral
        double derivative = error - previousError; // Calculate derivative
        double output = Kp * error + Ki * integral + Kd * derivative; // Compute output


        // Set initial motor speed
        Motor.A.setSpeed(BASE_SPEED);
        Motor.B.setSpeed(BASE_SPEED);

        // Start forward movement
        Motor.A.forward();
        Motor.B.forward();

        // Create a thread to handle obstacle detection
        Thread obstacleThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!Button.ESCAPE.isDown()) {
                    distanceProvider.fetchSample(distanceSample, 0);
                    float distance = distanceSample[0];

                    // Display distance on screen
                    LCD.clear();
                    LCD.drawString("Dist: " + distance, 0, 0);

                    // If obstacle detected, stop and spin to avoid
                    if (distance < CRITICAL_DISTANCE) {
                        Motor.A.stop(true);
                        Motor.B.stop();

                        // Simple avoidance: spin in place
                        Motor.A.rotate(-ROTATE_ANGLE, true);
                        Motor.B.rotate(ROTATE_ANGLE);
                        Delay.msDelay(1000); // spin for 1 second

                        // Resume normal speed and forward motion
                        Motor.A.setSpeed(BASE_SPEED);
                        Motor.B.setSpeed(BASE_SPEED);
                        Motor.A.forward();
                        Motor.B.forward();
                    }
                    else if (distance < SLOW_DISTANCE) {
                        // Obstacle nearby, slow down
                        Motor.A.setSpeed(SLOW_SPEED);
                        Motor.B.setSpeed(SLOW_SPEED);
                    }
                    else {
                        // No obstacle, move at normal speed
                        Motor.A.setSpeed(BASE_SPEED);
                        Motor.B.setSpeed(BASE_SPEED);
                    }

                    Delay.msDelay(100);
                }
            }
        });

        // Start the thread
        obstacleThread.setDaemon(true);
        obstacleThread.start();

        // Main loop: Line following
        while (!Button.ESCAPE.isDown()) {
            colorProvider.fetchSample(colorSample, 0);
            float lightIntensity = colorSample[0]; // 0 = black, 1 = white

            // Simple logic: adjust direction based on line
            if (lightIntensity < 0.3) {
                // On the line (dark), go straight
                Motor.A.setSpeed(BASE_SPEED);
                Motor.B.setSpeed(BASE_SPEED);
                Motor.A.forward();
                Motor.B.forward();
            } else {
                // Off the line (light), correct course
                Motor.A.setSpeed(TURN_SPEED); // slow left motor
                Motor.B.setSpeed(BASE_SPEED); // normal right motor
                Motor.A.forward();
                Motor.B.forward();
            }

            Delay.msDelay(50); // small delay for stability
        }

        // Cleanup after loop
        colorSensor.close();
        ultrasonicSensor.close();
        Motor.A.stop();
        Motor.B.stop();

        Button.waitForAnyPress(); // wait before exiting
    }
    
}
