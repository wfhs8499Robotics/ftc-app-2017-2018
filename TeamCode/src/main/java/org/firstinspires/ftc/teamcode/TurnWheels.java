package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;

/**
 * TurnWheels.
 *
 * couple of methods to move the robot around..  one for inches and one for MM.
 *
 */

public class TurnWheels {

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     WHEEL_DIAMETER_MM       = 101.6 ;   // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1416);
    static final double     COUNTS_PER_MM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * 3.1416);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     HALF_SPEED              = 0.25;
    static final double     TURN_SPEED              = 0.25;
    // Robot configuration and turning distances
    static final double     wheelWidth              = 13.8125;
    static final double     completeCircle          = wheelWidth * 3.14159;
    static final double     turn180degrees          = completeCircle / 2;
    static final double     turn90degrees           = completeCircle / 4;
    static final double     turn35degrees           = completeCircle / 10.2857;

    static final double INCREMENT   = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_FWD     =  1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV     = -1.0;     // Maximum REV power applied to motor

    private DcMotor leftmotor = null; // Hardware Device Object
    private DcMotor rightmotor = null; // Hardware Device Object
    private IntegratingGyroscope gyro;
    private ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    // Define class members
    double  power   = 0;
    boolean rampUp  = true;

    private ElapsedTime runtime = new ElapsedTime();  // used for timing of the encoder run



    /* Constructor */
    public TurnWheels(){
    }

    public void init(HardwareMap hwMap){
        leftmotor = hwMap.dcMotor.get("left motor");
        rightmotor = hwMap.dcMotor.get("right motor");
        leftmotor.setDirection(DcMotor.Direction.REVERSE);
        // Get a reference to a Modern Robotics gyro object. We use several interfaces
        // on this object to illustrate which interfaces support which functionality.
        modernRoboticsI2cGyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;
        // If you're only interested int the IntegratingGyroscope interface, the following will suffice.
        // gyro = hardwareMap.get(IntegratingGyroscope.class, "gyro");
        // A similar approach will work for the Gyroscope interface, if that's all you need.
        // Start calibrating the gyro. This takes a few seconds and is worth performing
        // during the initialization phase at the start of each opMode.
        modernRoboticsI2cGyro.calibrate();

        // Wait until the gyro calibration is complete

        while (modernRoboticsI2cGyro.isCalibrating())  {
            try {
                sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        modernRoboticsI2cGyro.resetZAxisIntegrator();
    }
    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     *
     *  Note: Reverse movement is obtained by setting a negative distance (not speed)
     *      encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 48 Inches with 5 Sec timeout
     *      encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
     *      encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
     *
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS){
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        newLeftTarget = leftmotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newRightTarget = rightmotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        leftmotor.setTargetPosition(newLeftTarget);
        rightmotor.setTargetPosition(newRightTarget);

        leftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        leftmotor.setPower(Math.abs(speed));
        rightmotor.setPower(Math.abs(speed));


        // keep looping while we are still active, and there is time left, and both motors are running.
        while ((runtime.seconds() < timeoutS) &&
                (leftmotor.isBusy() && rightmotor.isBusy())) {

            // Allow time for other processes to run.
            try {
                sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        // Stop all motion;
        leftmotor.setPower(0);
        rightmotor.setPower(0);

        // Turn off RUN_TO_POSITION
        leftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        try {
            sleep(250);   // optional pause after each move
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }
    /*
*  Method to perform a relative move, based on encoder counts.
*  Encoders are not reset as the move is based on the current position.
*  Move will stop if any of three conditions occur:
*  1) Move gets to the desired position
*  2) Move runs out of time
*  3) Driver stops the opmode running.
*
*  Note: Reverse movement is obtained by setting a negative distance (not speed)
*      encoderDrive(DRIVE_SPEED,  480,  480, 5.0);  // S1: Forward 48 MMs with 5 Sec timeout
*      encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 MMs with 4 Sec timeout
*      encoderDrive(DRIVE_SPEED, -240, -240, 4.0);  // S3: Reverse 24 MMs with 4 Sec timeout
*

    public void encoderDriveMM(double speed,
                               double leftMMs, double rightMMs,
                               double timeoutS)  {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftmotor.getCurrentPosition() + (int)(leftMMs * COUNTS_PER_MM);
            newRightTarget = rightmotor.getCurrentPosition() + (int)(rightMMs * COUNTS_PER_MM);
            leftmotor.setTargetPosition(newLeftTarget);
            rightmotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftmotor.setPower(Math.abs(speed));
            rightmotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while ((runtime.seconds() < timeoutS) &&
                    (leftmotor.isBusy() && rightmotor.isBusy())) {

                // Allow time for other processes to run.
                try {
                    sleep(50);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            // Stop all motion;
            leftmotor.setPower(0);
            rightmotor.setPower(0);
            // Turn off RUN_TO_POSITION
            leftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
*/
    /*
     *  Helper methods to perform a relative turn, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     */
    public void right90 () {
        encoderDrive(TURN_SPEED,turn90degrees,-turn90degrees,10);
    }

    public void left90 () {
        encoderDrive(TURN_SPEED,-turn90degrees,turn90degrees,10);
    }

    public void right180(){
        encoderDrive(TURN_SPEED,turn180degrees,-turn180degrees, 10);
    }

    public void left180(){
        encoderDrive(TURN_SPEED, -turn180degrees,turn180degrees, 10);
    }

    public void right33(){
        encoderDrive(TURN_SPEED, turn35degrees, -turn35degrees, 10);
    }

    public void left33(){
        encoderDrive(TURN_SPEED, -turn35degrees, turn35degrees, 10);
    }





    public void rampMotor(){
        // Ramp the motors, according to the rampUp variable.
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                power += INCREMENT ;
                if (power >= MAX_FWD ) {
                    power = MAX_FWD;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                power -= INCREMENT ;
                if (power <= MAX_REV ) {
                    power = MAX_REV;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }

            // Set the motor to the new power and pause;
            leftmotor.setPower(power);
            rightmotor.setPower(power);
            try {
                sleep(CYCLE_MS);   // optional pause after each move
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        // Turn off motor and signal done;
//        leftmotor.setPower(0);
//        rightmotor.setPower(0);

    }
}
