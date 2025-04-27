package last;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class FinalExam {

    static EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.A);
    static EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.B);

    static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);
    static EV3UltrasonicSensor ultraSensor = new EV3UltrasonicSensor(SensorPort.S2);

    static volatile boolean foundObstacle = false;
    static volatile boolean rotatedOnce = false;
    static volatile boolean rotatedTwice = false;
    static volatile boolean slowingDown = false;
    static volatile boolean searchingLine = false;
    static volatile boolean adjustSpeed = false;

    static int baseSpeed = 180;
    static int minSpeed = 120;
    static int currentBaseSpeed = baseSpeed;

    static float kp = 250f;
    static float kd = 500f;
    static float target = 0.35f;
    static float filteredLight = target;
    static float lastError = 0f;

    public static void main(String[] args) {

        Thread distanceThread = new Thread(new Runnable() {
            public void run() {
                SampleProvider distSP = ultraSensor.getDistanceMode();
                float[] distSample = new float[distSP.sampleSize()];
                while (true) {
                    distSP.fetchSample(distSample, 0);
                    float distance = distSample[0];

                    if (!foundObstacle && distance <= 0.25f && distance > 0.20f) {
                        slowingDown = true;
                    }

                    if (!foundObstacle && distance <= 0.20f) {
                        foundObstacle = true;
                        leftMotor.setSpeed(100);
                        rightMotor.setSpeed(100);
                        leftMotor.backward();
                        rightMotor.forward();
                    }

                    if (foundObstacle && !rotatedOnce && distance > 0.5f) {
                        Delay.msDelay(240);
                        int rightSpeed = 150;
                        int leftSpeed = (int)(rightSpeed * 1.24);
                        leftMotor.setSpeed(leftSpeed);
                        rightMotor.setSpeed(rightSpeed);
                        leftMotor.forward();
                        rightMotor.forward();
                        rotatedOnce = true;
                        rotatedTwice = true;
                        searchingLine = true;
                        slowingDown = false;
                        currentBaseSpeed = baseSpeed;

                        Thread adjustThread = new Thread(new Runnable() {
                            public void run() {
                                Delay.msDelay(6000);
                                adjustSpeed = true;
                            }
                        });
                        adjustThread.start();
                    }
                    Delay.msDelay(10);
                }
            }
        });

        Thread lightThread = new Thread(new Runnable() {
            public void run() {
                SampleProvider lightSP = colorSensor.getRedMode();
                float[] lightSample = new float[lightSP.sampleSize()];
                while (true) {
                    if (searchingLine) {
                        lightSP.fetchSample(lightSample, 0);
                        float light = lightSample[0];

                        if (Math.abs(light - target) < 0.05f) {
                            searchingLine = false;
                            foundObstacle = false;
                            rotatedOnce = false;
                            rotatedTwice = false;
                            slowingDown = false;
                            adjustSpeed = false;
                        }
                        Delay.msDelay(10);
                    } else {
                        Delay.msDelay(5);
                    }
                }
            }
        });

        Thread driveThread = new Thread(new Runnable() {
            public void run() {
                SampleProvider lightSP = colorSensor.getRedMode();
                float[] lightSample = new float[lightSP.sampleSize()];
                while (true) {
                    if (!foundObstacle && !searchingLine) {
                        lightSP.fetchSample(lightSample, 0);
                        float rawLight = lightSample[0];

                        filteredLight = 0.5f * filteredLight + 0.5f * rawLight;

                        float error = filteredLight - target;
                        float derivative = error - lastError;

                        int correction = (int)(kp * error + kd * derivative);
                        lastError = error;

                        if (slowingDown && currentBaseSpeed > minSpeed) {
                            currentBaseSpeed -= 1;
                        }
                        if (!slowingDown && currentBaseSpeed < baseSpeed) {
                            currentBaseSpeed += 1;
                        }

                        int leftSpeed = currentBaseSpeed - correction;
                        int rightSpeed = currentBaseSpeed + correction;

                        leftSpeed = Math.max(80, Math.min(450, leftSpeed));
                        rightSpeed = Math.max(80, Math.min(450, rightSpeed));

                        leftMotor.setSpeed(leftSpeed);
                        rightMotor.setSpeed(rightSpeed);

                        leftMotor.forward();
                        rightMotor.forward();
                    }

                    if (searchingLine) {
                        int rightSpeed = 150;
                        int leftSpeed = adjustSpeed ? (int)(rightSpeed * 1.3) : (int)(rightSpeed * 1.24);

                        leftMotor.setSpeed(leftSpeed);
                        rightMotor.setSpeed(rightSpeed);

                        leftMotor.forward();
                        rightMotor.forward();
                    }

                    Delay.msDelay(5);
                }
            }
        });

        distanceThread.start();
        lightThread.start();
        driveThread.start();
    }
}