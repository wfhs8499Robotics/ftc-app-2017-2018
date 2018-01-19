package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*
 * The code is structured as an Iterative OpMode
 */

@TeleOp(name="8499: Driver Mode", group="TeleOp")

public class DriverMode extends OpMode {

    DcMotor leftmotor = null;   // Hardware Device Object
    DcMotor rightmotor = null;  // Hardware Device Object
    DcMotor liftmotor = null;   // Hardware Device Object
    DcMotor relicExtensionMotor = null;
    Servo leftgrabber = null;         // Hardware Device Object
    Servo rightgrabber = null;         // Hardware Device Object
    Servo jewelpusher = null;         // Hardware Device Object
    Servo relicGrabberServo = null;
    CRServo relicRotatorCRServo = null;
    ColorSensor colorSensor = null;


    float LiftPercent = 0.40f;  // Lift Motor:: only use 50 percent power as the default speed at full throttle
    float LIFT_LOWER_PERCENT = 0.5f;
    float StickPercent = 0.5f;  // only use 50 percent power as the default speed at full throttle
    // settings for the Servo
    static final double RIGHT_MAX_POS = 0.60;     // Maximum rotational position
    static final double RIGHT_MIN_POS = 0.39;     // Minimum rotational position    static final double MAX_POS = 0.70;     // Maximum rotational position
    static final double LEFT_MAX_POS = 0.60;     // Maximum rotational position
    static final double LEFT_MIN_POS = 0.42;     // Minimum rotational position

    // settings for the lift release servo
    static final double LIFT_MAX_POS     =  0.50;     // Maximum rotational position
    static final double LIFT_MIN_POS     =  0.05;     // Minimum rotational position

    static final double RELIC_GRABBER_OPEN = 0.33;
    static final double RELIC_GRABBER_CLOSED = 0.00;
    static final double RELIC_ROTATOR_BACK = 0.02;
    static final double RELIC_ROTATOR_DOWN = -0.02;
    static final double RELIC_ROTATOR_OUT = -0.12;
    static final double RELIC_ROTATOR_UP = -0.17;

    // all the variables we need
    double leftpower;
    double rightpower;
    double lift;
    double relicExtension;
    float hypermode;
    float seanmode;
    float hyperliftmode;
    float seanliftmode;
    float driveadjustment;
    float liftadjustment;
    boolean opengrabber = false;
//    float squeezegrabberleft = 0;
    boolean centerservo = false;
    boolean extendbothservo = false;
    boolean jewelpusherpushed = false;
    boolean bSeanMode = false;
    boolean bFastMode = false;
    boolean bSeanButtonPushed = false;
    boolean bFastButtonPushed = false;
    boolean bSeanLiftMode = false;
    boolean bFastLiftMode = false;
    boolean bSeanLiftButtonPushed = false;
    boolean bFastLiftButtonPushed = false;
    boolean bLedOff = false;
    boolean bRelicGrabber = false;
    boolean bRelicRotatorUp = false;
    boolean bRelicRotatorReverse = false;
    boolean bRelicRotatorZero = false;
    boolean bRelicRotatorOut = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        // get the motor objects created
        leftmotor = hardwareMap.dcMotor.get("left motor");

        rightmotor = hardwareMap.dcMotor.get("right motor");
        leftmotor.setDirection(DcMotor.Direction.REVERSE);

        liftmotor = hardwareMap.dcMotor.get("lift");

        relicExtensionMotor = hardwareMap.dcMotor.get("relic extension motor");
        // Get the servo object created
        leftgrabber = hardwareMap.servo.get("left grabber");
        rightgrabber = hardwareMap.servo.get("right grabber");
        leftgrabber.setDirection(Servo.Direction.REVERSE);
        //position the servo to the minimum position
        leftgrabber.setPosition(LEFT_MIN_POS);
        rightgrabber.setPosition(RIGHT_MIN_POS);
        // Get the lift release servo object created
        jewelpusher = hardwareMap.servo.get("jewel pusher");
//        jewelpusher.setDirection(Servo.Direction.REVERSE);
        //position the servo to Minimum position
        jewelpusher.setPosition(LIFT_MIN_POS);
        relicGrabberServo = hardwareMap.servo.get("relic grabber");
        relicRotatorCRServo = hardwareMap.crservo.get("relic rotator");
        relicRotatorCRServo.setPower(RELIC_ROTATOR_BACK);
        bRelicRotatorReverse  = true;
        relicGrabberServo.setPosition(RELIC_GRABBER_CLOSED);

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.colorSensor.get("color sensor");
        colorSensor.enableLed(bLedOff);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver - I am ready");    //
        updateTelemetry(telemetry);
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
// nothing to do here
    }
    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        // nothing to do here
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        // get all the gamepad variables
        leftpower = -gamepad1.left_stick_y;
        rightpower = -gamepad1.right_stick_y;
        hypermode = gamepad1.right_trigger;
        seanmode = gamepad1.left_trigger;
        opengrabber = gamepad2.right_bumper;
        bRelicGrabber = gamepad2.left_bumper;
//        squeezegrabberleft = gamepad2.left_trigger;
        lift = -gamepad2.left_stick_y;
//        hyperliftmode = gamepad2.right_trigger;
//        seanliftmode = gamepad2.left_trigger;
        jewelpusherpushed = gamepad2.y;

        relicExtension = gamepad2.right_stick_y;
        bRelicRotatorUp = gamepad2.dpad_up;
        bRelicRotatorZero = gamepad2.dpad_down;
        bRelicRotatorReverse = gamepad2.dpad_right;
        bRelicRotatorOut = gamepad2.dpad_left;
        // if either trigger has started to be pushed, wait til it goes to 0 to toggle modes
        if (hypermode > 0){
            bFastButtonPushed = true;
        }
        if (hypermode == 0 && bFastButtonPushed){
            bFastButtonPushed = false;
            bFastMode = !bFastMode;
            if (bFastMode){
                bSeanMode = false;
            }
        }
        if (seanmode > 0){
            bSeanButtonPushed = true;
        }
        if (seanmode == 0 && bSeanButtonPushed){
            bSeanButtonPushed = false;
            bSeanMode = !bSeanMode;
            if (bSeanMode){
                bFastMode = false;
            }
        }
        // Lift Motor Controls:: if either trigger has started to be pushed, wait til it goes to 0 to toggle modes
        if (hyperliftmode > 0){
            bFastLiftButtonPushed = true;
        }
        if (hyperliftmode == 0 && bFastLiftButtonPushed){
            bFastLiftButtonPushed = false;
            bFastLiftMode = !bFastLiftMode;
            if (bFastLiftMode){
                bSeanLiftMode = false;
            }
        }
        if (seanliftmode > 0){
            bSeanLiftButtonPushed = true;
        }
        if (seanliftmode == 0 && bSeanLiftButtonPushed){
            bSeanLiftButtonPushed = false;
            bSeanLiftMode = !bSeanLiftMode;
            if (bSeanLiftMode){
                bFastLiftMode = false;
            }
        }
        // move the servo forward on the right
        if (opengrabber == true){
            rightgrabber.setPosition(RIGHT_MAX_POS);
            leftgrabber.setPosition(LEFT_MAX_POS);
//            leftgrabber.setPosition(MIN_POS);
        } else {
            rightgrabber.setPosition(RIGHT_MIN_POS);
            leftgrabber.setPosition(LEFT_MIN_POS);
        }

        // center the servo
        if (centerservo){
            leftgrabber.setPosition(LEFT_MIN_POS);
            rightgrabber.setPosition(RIGHT_MIN_POS);
        }
        // Extend both servos
        if (extendbothservo){
            leftgrabber.setPosition(LEFT_MAX_POS);
            rightgrabber.setPosition(LEFT_MAX_POS);
        }
        if (jewelpusherpushed) {
            jewelpusher.setPosition(LIFT_MIN_POS);
        }
        if (bRelicGrabber){
            relicGrabberServo.setPosition(RELIC_GRABBER_OPEN);
        } else {
            relicGrabberServo.setPosition(RELIC_GRABBER_CLOSED);
        }
        if (bRelicRotatorZero) {
            relicRotatorCRServo.setPower(RELIC_ROTATOR_DOWN);
//            bRelicRotatorZero = false;
        }
        if (bRelicRotatorReverse) {
            relicRotatorCRServo.setPower(RELIC_ROTATOR_BACK);
        }
        if (bRelicRotatorOut){
            relicRotatorCRServo.setPower(RELIC_ROTATOR_OUT);
        }
        if (bRelicRotatorUp){
            relicRotatorCRServo.setPower(RELIC_ROTATOR_UP);
        }

        // set drive adjustment to the default stick percent
        driveadjustment = StickPercent;
        // change the drive adjustment for hypermode
        if (bFastMode){
            driveadjustment = StickPercent * 2.0f;
        }
        // change the drive adjustment to slow mode
        if (bSeanMode){
            driveadjustment = StickPercent * 0.5f;
        }
        // Lift Motor::  set drive adjustment to the default stick percent
        liftadjustment = LiftPercent;
        // change the drive adjustment for hypermode
        if (bFastLiftMode){
            liftadjustment = LiftPercent * 2.0f;
        }
        // change the drive adjustment to slow mode
        if (bSeanLiftMode){
            liftadjustment = LiftPercent * 0.5f;
        }
        if (lift < 0){
            liftadjustment = liftadjustment * LIFT_LOWER_PERCENT;
        }

        // set the power of the motor to the stick value multiplied by the adjustment
        leftmotor.setPower(leftpower * driveadjustment);
        rightmotor.setPower(rightpower * driveadjustment);
        liftmotor.setPower(lift * liftadjustment);
        relicExtensionMotor.setPower(relicExtension);

        // Tell the driver
        telemetry.addData("Fast Mode", bFastMode);
        telemetry.addData("Sean Mode", bSeanMode);
        telemetry.addData("left",  "%.2f", leftpower * driveadjustment);
        telemetry.addData("right", "%.2f", rightpower * driveadjustment);
        telemetry.addData("Lift Fast Mode", bFastLiftMode);
        telemetry.addData("Lift Sean Mode", bSeanLiftMode);
        telemetry.addData("Lift",  "%.2f", lift * liftadjustment);
        telemetry.addData("relic extension", "%2f", relicExtension);

        if (opengrabber == true){
            telemetry.addData("servo", "servo open pushed %.2f", RIGHT_MAX_POS);
        }
        if (centerservo){
            telemetry.addData("servo", "servo center pushed %.2f", RIGHT_MIN_POS);
        }
        if (extendbothservo){
            telemetry.addData("servo", "servo extend pushed %.2f", RIGHT_MAX_POS);
        }
        updateTelemetry(telemetry);
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // nothing to do here
    }
}
