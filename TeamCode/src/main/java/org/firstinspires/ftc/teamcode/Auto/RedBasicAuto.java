package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Commands.BasicMecanumDrive;
import org.firstinspires.ftc.teamcode.Commands.Launcher;
import org.firstinspires.ftc.teamcode.Commands.WheelRotation;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(group = "Front", name="FrontRedAuto")
public class RedBasicAuto extends LinearOpMode {

    private BasicMecanumDrive drive;
    private Launcher launcher;

    private DcMotor intake;
    private Servo kick;
    private Servo kick2;
    private Servo hood;

    private ColorSensor intakeColor;
    private ColorSensor outtakeColor;

    private ElapsedTime timer = new ElapsedTime();

    private int kickState = 0;
    private int kickWheelCounter = 0;

    private final double KICK_UP = 0.6;
    private final double KICK_DOWN = 0.2;
    private final double KICK2_UP = 1.0;
    private final double KICK2_DOWN = 0.5;
    private int intakeCounter = 0;
    private int outtakeCounter = 0;
    private int kickRun = 0;
    int ballCount = 0;
    boolean lastDetected = false;
    boolean thing = false;

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
        timer.reset();

        launcher.updateTurretFromAprilTag();

        //move back and shoot
        drive.resetHeading();
        sleep(100);
        launcher.setFlywheelRPM(3400);
        launcher.updateTurretFromAprilTag();
        launcher.updateFlywheel();
        hood.setPosition(0.1);
        drive.drive(-0.5,0,0);
        sleep(1500);
        drive.brake();
        sleep(200);
        launcher.updateTurretFromAprilTag();
        launcher.updateFlywheel();

        while (opModeIsActive() && kickRun <= 5){
            if (kickState == 0) {
                launcher.updateFlywheel();
                launcher.updateTurretFromAprilTag();
                kick.setPosition(KICK_UP);
                sleep(100);
                kickState = 1;
            }

            if (kickState == 1) {
                launcher.updateFlywheel();
                launcher.updateTurretFromAprilTag();
                kick2.setPosition(KICK2_UP);
                sleep(200);
                kickState = 2;
            }

            if (kickState == 2) {
                launcher.updateFlywheel();
                launcher.updateTurretFromAprilTag();
                kick.setPosition(KICK_DOWN);
                kick2.setPosition(KICK2_DOWN);
                sleep(400);
                kickState = 3;
            }

            if (kickState == 3) {
                launcher.updateFlywheel();
                launcher.updateTurretFromAprilTag();
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

        launcher.stopFlywheel();
        kickRun = 0;
        sleep(500);
        drive.drive(0.1,0.47, 0);
        sleep(700);
        drive.brake();
        drive.drive(0,0,0.3);
        sleep(730);
        drive.brake();
        drive.resetHeading();
        wheel.rotateToAngle(98, 0.3);
        drive.drive(0.35,0,0);
        sleep(500);
        intake.setPower(-0.8);
        drive.drive(0.15,0,0);
        sleep(200);

        while (opModeIsActive() && ballCount < 3 && timer.seconds() <= 17.5) {

            boolean detected = isGreenOrPurple(outtakeColor);

            if (detected && !lastDetected) {

                switch (ballCount) {
                    case 0:
                        wheel.rotateToAngle(200, 0.4);
                        break;
                    case 1:
                        wheel.rotateToAngle(312, 0.4);
                        break;
                }

                ballCount++;
            }

            lastDetected = detected;
            sleep(20);
        }

        intake.setPower(0);
        drive.brake();

        wheel.rotateToAngle(357, 0.3);

        sleep(100);
        thing = true;

        if (thing) {
            launcher.setFlywheelRPM(3400);
            launcher.updateFlywheel();
            hood.setPosition(0.1);
            drive.drive(-0.5,0,0);
            sleep(1300);
            drive.brake();
            drive.drive(0,0,-0.3);
            sleep(900);
            drive.brake();
            sleep(100);
            launcher.updateTurretFromAprilTag();
            launcher.updateFlywheel();

            while (opModeIsActive() && kickRun <= 3 && thing) {
                if (kickState == 0) {
                    launcher.updateFlywheel();
                    launcher.updateTurretFromAprilTag();
                    kick.setPosition(KICK_UP);
                    sleep(100);
                    kickState = 1;
                }

                if (kickState == 1) {
                    launcher.updateFlywheel();
                    launcher.updateTurretFromAprilTag();
                    kick2.setPosition(KICK2_UP);
                    sleep(200);
                    kickState = 2;
                }

                if (kickState == 2) {
                    launcher.updateFlywheel();
                    launcher.updateTurretFromAprilTag();
                    kick.setPosition(KICK_DOWN);
                    kick2.setPosition(KICK2_DOWN);
                    sleep(400);
                    kickState = 3;
                }

                if (kickState == 3) {
                    launcher.updateFlywheel();
                    launcher.updateTurretFromAprilTag();
                    switch (kickWheelCounter) {
                        case 0:
                            wheel.rotateToAngle((120), 0.3);
                            break;
                        case 1:
                            wheel.rotateToAngle((240), 0.3);
                            break;
                        case 2:
                            wheel.rotateToAngle((4), 0.3);
                            break;
                    }

                    kickWheelCounter = (kickWheelCounter + 1) % 3;
                    kickState = 0;
                    kickRun++;
                }
            }

            sleep(100);
            drive.drive(0,0,0.3);
            sleep(730);
            drive.brake();
            drive.drive(0.4,0,0);
            sleep(1500);
            drive.brake();
        }
    }

    private boolean isGreenOrPurple(ColorSensor sensor) {

        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        int total = r + g + b;
        if (total < 350) return false;

        boolean isGreen = g > r + 40 && g > b + 40;
        boolean isPurple = b > g + 30 && r > g + 10;

        return isGreen || isPurple;
    }

}
