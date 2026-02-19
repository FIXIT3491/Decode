package org.firstinspires.ftc.teamcode.AutoTest;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import org.firstinspires.ftc.teamcode.Commands.BasicAutoDriveSubsystem;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Commands.BasicMecanumDrive;
import org.firstinspires.ftc.teamcode.Commands.Launcher;
import org.firstinspires.ftc.teamcode.Commands.WheelRotation;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(group = "Test", name="Basic Test")
public class BasicAutoTest extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        BasicAutoDriveSubsystem autoDrive =
                new BasicAutoDriveSubsystem(hardwareMap);

        waitForStart();

        autoDrive.driveForward(12, 0.3);
        wait(250);
        autoDrive.stop();
        wait(50);
        autoDrive.strafeRight(12, 0.3);
        wait(250);
        autoDrive.stop();

    }


}
