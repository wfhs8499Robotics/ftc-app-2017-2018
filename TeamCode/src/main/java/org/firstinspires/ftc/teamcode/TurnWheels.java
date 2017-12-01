package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    static final double     DRIVE_SPEED             = 0.6;
    static final double     HALF_SPEED              = 0.3;
    static final double     TURN_SPEED              = 0.5;
    // Robot configuration and turning distances
    static final double     wheelWidth              = 13.0;
    static final double     completeCircle          = 13.0 * 3.14159;
    static final double     turn90degrees           = completeCircle / 4;
    static final double     turn45degrees           = completeCircle / 8;

    DcMotor leftmotor = null; // Hardware Device Object
    DcMotor rightmotor = null; // Hardware Device Object

    private ElapsedTime runtime = new ElapsedTime();  // used for timing of the encoder run


    /* Constructor */
    public TurnWheels(){
    }

    public void init(HardwareMap hwMap){
        leftmotor = hwMap.dcMotor.get("left motor");
        leftmotor.setDirection(DcMotor.Direction.REVERSE);
        rightmotor = hwMap.dcMotor.get("right motor");
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
*/
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


}
