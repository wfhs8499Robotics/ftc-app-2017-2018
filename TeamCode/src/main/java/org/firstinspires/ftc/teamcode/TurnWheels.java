package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    private DcMotor leftmotor = null; // Hardware Device Object
    private DcMotor rightmotor = null; // Hardware Device Object
    private IntegratingGyroscope gyro;
    private ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    private ModernRoboticsI2cRangeSensor rangeSensor;
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
        //*rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
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
    public int getRobotHeading(){

        int heading = modernRoboticsI2cGyro.getHeading();
        return heading;
    }
    public void getRangeDistance(){

        rangeSensor.rawUltrasonic();
    }

/*    // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        leftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

    // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating())  {
        sleep(50);
        idle();
    }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        leftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
        telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
        telemetry.update();
    }

        gyro.resetZAxisIntegrator();

    // Step through each leg of the path,
    // Note: Reverse movement is obtained by setting a negative distance (not speed)
    // Put a hold after each turn
    gyroDrive(DRIVE_SPEED, 48.0, 0.0);    // Drive FWD 48 inches
    gyroTurn( TURN_SPEED, -45.0);         // Turn  CCW to -45 Degrees
    gyroHold( TURN_SPEED, -45.0, 0.5);    // Hold -45 Deg heading for a 1/2 second
    gyroDrive(DRIVE_SPEED, 12.0, -45.0);  // Drive FWD 12 inches at 45 degrees
    gyroTurn( TURN_SPEED,  45.0);         // Turn  CW  to  45 Degrees
    gyroHold( TURN_SPEED,  45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second
    gyroTurn( TURN_SPEED,   0.0);         // Turn  CW  to   0 Degrees
    gyroHold( TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for a 1 second
    gyroDrive(DRIVE_SPEED,-48.0, 0.0);    // Drive REV 48 inches

        telemetry.addData("Path", "Complete");
        telemetry.update();
}
*/

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still activ

        // Determine new target position, and pass to motor controller
        moveCounts = (int)(distance * COUNTS_PER_INCH);
        newLeftTarget = leftmotor.getCurrentPosition() + moveCounts;
        newRightTarget = rightmotor.getCurrentPosition() + moveCounts;

        // Set Target and Turn On RUN_TO_POSITION
        leftmotor.setTargetPosition(newLeftTarget);
        rightmotor.setTargetPosition(newRightTarget);

        leftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        leftmotor.setPower(speed);
        rightmotor.setPower(speed);

        // keep looping while we are still active, and BOTH motors are running.
        while ((leftmotor.isBusy() && rightmotor.isBusy())) {

            // adjust relative speed based on heading error.
            error = getError(angle);
            steer = getSteer(error, P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                steer *= -1.0;

            leftSpeed = speed - steer;
            rightSpeed = speed + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            leftmotor.setPower(leftSpeed);
            rightmotor.setPower(rightSpeed);

            // Display drive status for the driver.
            // telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
            // telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
            // telemetry.addData("Actual",  "%7d:%7d",      leftmotor.getCurrentPosition(),
                 //   rightmotor.getCurrentPosition());
            // telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
            // telemetry.update();
        }

        // Stop all motion;
        leftmotor.setPower(0);
        rightmotor.setPower(0);

        // Turn off RUN_TO_POSITION
        leftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (!onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            try {
                sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while ((holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            try {
                sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }

        // Stop all motion;
        leftmotor.setPower(0);
        rightmotor.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        leftmotor.setPower(leftSpeed);
        rightmotor.setPower(rightSpeed);

        // Display it for the driver.
        // telemetry.addData("Target", "%5.2f", angle);
        // telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        // telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - modernRoboticsI2cGyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}
