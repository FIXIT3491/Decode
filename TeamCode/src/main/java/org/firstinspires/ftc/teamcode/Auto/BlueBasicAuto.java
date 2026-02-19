package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Commands.BasicMecanumDrive;
import org.firstinspires.ftc.teamcode.Commands.Launcher;
import org.firstinspires.ftc.teamcode.Commands.WheelRotation;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(group = "Front", name="FrontBlueAuto")
public class BlueBasicAuto extends LinearOpMode {

    private BasicMecanumDrive drive;
    private Launcher launcher;

    private DcMotor intake;
    private Servo kick;
    private Servo kick2;
    private Servo hood;

    private ColorSensor intakeColor;
    private ColorSensor outtakeColor;

    private int kickState = 0;
    private int kickWheelCounter = 0;

    private final double KICK_UP = 0.6;
    private final double KICK_DOWN = 0.2;
    private final double KICK2_UP = 1.0;
    private final double KICK2_DOWN = 0.5;
    private int intakeCounter = 0;
    private int outtakeCounter = 0;
    private int kickRun = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        BasicMecanumDrive drive = new BasicMecanumDrive(hardwareMap);
        Launcher launcher = new Launcher();
        WheelRotation wheel = new WheelRotation();
        intake = hardwareMap.get(DcMotor.class, "intake");

        kick = hardwareMap.get(Servo.class, "kick");
        kick2 = hardwareMap.get(Servo.class, "kick2");

        intakeColor = hardwareMap.get(ColorSensor.class, "color2");
        outtakeColor = hardwareMap.get(ColorSensor.class, "color1");

        kick2.setDirection(Servo.Direction.REVERSE);

        kick.setPosition(KICK_DOWN);
        kick2.setPosition(KICK2_DOWN);

        hood = hardwareMap.get(Servo.class, "hood");

        waitForStart();

        launcher.init(hardwareMap);
        wheel.init(hardwareMap, telemetry);
        launcher.setTrackedTagId(20);

        launcher.updateTurretFromAprilTag();

        //move back and shoot
        drive.resetHeading();
        sleep(100);
        drive.drive(-0.5,0,0);
        sleep(1000);
        drive.stopMotors();
        launcher.setFlywheelRPM(3200);
        launcher.updateFlywheel();
        hood.setPosition(0.0);
        sleep(200);

        while (opModeIsActive() && kickRun < 3){
            if (kickState == 0) {
                kick.setPosition(KICK_UP);
                sleep(100);
                kickState = 1;
            }

            if (kickState == 1) {
                kick2.setPosition(KICK2_UP);
                sleep(200);
                kickState = 2;
            }

            if (kickState == 2) {
                kick.setPosition(KICK_DOWN);
                kick2.setPosition(KICK2_DOWN);
                sleep(400);
                kickState = 3;
            }

            if (kickState == 3) {

                switch (kickWheelCounter) {
                    case 0: wheel.rotateToAngle((120), 0.3); break;
                    case 1: wheel.rotateToAngle((240), 0.3); break;
                    case 2: wheel.rotateToAngle((4), 0.3); break;
                }

                kickWheelCounter = (kickWheelCounter + 1) % 3;
                kickState = 0;
                kickRun++;
            }
        }

    }
}
