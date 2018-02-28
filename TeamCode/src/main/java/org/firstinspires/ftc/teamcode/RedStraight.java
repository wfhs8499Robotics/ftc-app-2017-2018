
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


@Autonomous(name="Red Straight", group="Autonomous")

public class RedStraight extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private TurnWheels turnWheels = new TurnWheels();

    private CameraOn cameraOn = new CameraOn();
    private JewelMover jewelMover = new JewelMover();
    private PlaceGlyph placeGlyph = new PlaceGlyph();

    private static final double columnWidth = 7.63;
    private static final double wheelOffest = 4.25;
    private static final double centerOffest = 1.25;

    private RelicRecoveryVuMark vuMark;
    @Override
    public void runOpMode() {

        cameraOn.init(hardwareMap);
        turnWheels.init(hardwareMap);

        jewelMover.init(hardwareMap, "RED");
        placeGlyph.preinit(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        vuMark = cameraOn.run();
        telemetry.addData("VuMark", "%s visible", vuMark);
        telemetry.update();

        placeGlyph.init(hardwareMap);

        jewelMover.run();

        turnWheels.gyroDrive(.3,24 - wheelOffest,0);
                //encoderDrive(.3,18.25,18.25, 10);
        turnWheels.left90();
        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            turnWheels.gyroDrive(.3,12 + centerOffest - columnWidth,turnWheels.getRobotHeading());
        }
        if (vuMark == RelicRecoveryVuMark.CENTER || vuMark == RelicRecoveryVuMark.UNKNOWN) {
            turnWheels.gyroDrive(.3,12 + centerOffest,turnWheels.getRobotHeading());
        }
        // encoderDrive(.3,30.25,30.25, 10);
        if (vuMark == RelicRecoveryVuMark.LEFT) {
            turnWheels.gyroDrive(.3,12 + centerOffest + columnWidth,turnWheels.getRobotHeading());
        }
                //encoderDrive(.3,12,12, 10);
        turnWheels.heading0();

        placeGlyph.run(vuMark);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
}
