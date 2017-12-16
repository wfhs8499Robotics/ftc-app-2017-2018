package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    DcMotor frontleftmotor = null;   // Hardware Device Object
    DcMotor frontrightmotor = null;  // Hardware Device Object
    DcMotor liftmotor = null;   // Hardware Device Object
    Servo leftgrabber = null;         // Hardware Device Object
    Servo rightgrabber = null;         // Hardware Device Object
    Servo jewelpusher = null;         // Hardware Device Object
    ColorSensor colorSensor = null;


    float LiftPercent = 0.25f;  // Lift Motor:: only use 50 percent power as the default speed at full throttle
    float LIFT_LOWER_PERCENT = 0.5f;
    float StickPercent = 0.5f;  // only use 50 percent power as the default speed at full throttle
    // settings for the Servo
    static final double RIGHT_MAX_POS = 0.80;     // Maximum rotational position
    static final double RIGHT_MIN_POS = 0.60;     // Minimum rotational position    static final double MAX_POS = 0.70;     // Maximum rotational position
    static final double LEFT_MAX_POS = 0.60;     // Maximum rotational position
    static final double LEFT_MIN_POS = 0.50;     // Minimum rotational position

    // settings for the lift release servo
    static final double LIFT_MAX_POS     =  0.50;     // Maximum rotational position
    static final double LIFT_MIN_POS     =  0.05;     // Minimum rotational position

    // all the variables we need
    double leftpower;
    double rightpower;
    double lift;
    float hypermode;
    float seanmode;
    float hyperliftmode;
    float seanliftmode;
    float driveadjustment;
    float liftadjustment;
    boolean squeezegrabberright = false;
    float squeezegrabberleft = 0;
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

        frontleftmotor = hardwareMap.dcMotor.get("front left motor");

        frontrightmotor = hardwareMap.dcMotor.get("front right motor");
        frontleftmotor.setDirection(DcMotor.Direction.REVERSE);
        // get the motor objects created
        liftmotor = hardwareMap.dcMotor.get("lift");
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
        squeezegrabberright = gamepad2.left_bumper;
        squeezegrabberleft = gamepad2.left_trigger;
        lift = -gamepad2.left_stick_y;
//        hyperliftmode = gamepad2.right_trigger;
//        seanliftmode = gamepad2.left_trigger;
        jewelpusherpushed = gamepad2.y;
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
        if (squeezegrabberright == true){
            rightgrabber.setPosition(RIGHT_MAX_POS);
//            leftgrabber.setPosition(MIN_POS);
        } else {
            rightgrabber.setPosition(RIGHT_MIN_POS);
        }
        // move the servo forward on the left
        if (squeezegrabberleft != 0){
            leftgrabber.setPosition(LEFT_MAX_POS);
//            rightgrabber.setPosition(MIN_POS);
        } else {
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
            jewelpusher.setPosition(LIFT_MAX_POS);
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
        frontleftmotor.setPower(leftpower * driveadjustment);
        frontrightmotor.setPower(rightpower * driveadjustment);
        liftmotor.setPower(lift * liftadjustment);

        // Tell the driver
        telemetry.addData("Fast Mode", bFastMode);
        telemetry.addData("Sean Mode", bSeanMode);
        telemetry.addData("left",  "%.2f", leftpower * driveadjustment);
        telemetry.addData("right", "%.2f", rightpower * driveadjustment);
        telemetry.addData("Lift Fast Mode", bFastLiftMode);
        telemetry.addData("Lift Sean Mode", bSeanLiftMode);
        telemetry.addData("Lift",  "%.2f", lift * liftadjustment);

        if (squeezegrabberright == true){
            telemetry.addData("servo", "servo right pushed %.2f", RIGHT_MAX_POS);
        }
        if (squeezegrabberleft != 0){
            telemetry.addData("servo", "servo left pushed %.2f", LEFT_MIN_POS);
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
