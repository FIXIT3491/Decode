package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Launcher {

    private DcMotor leftFlywheel;
    private DcMotor rightFlywheel;

    // Initialize motors
    public void init(HardwareMap hardwareMap) {
        leftFlywheel = hardwareMap.get(DcMotor.class, "flyLeft");
        rightFlywheel = hardwareMap.get(DcMotor.class, "flyRight");

        // Make sure both spin in the same direction
        leftFlywheel.setDirection(DcMotor.Direction.FORWARD);
        rightFlywheel.setDirection(DcMotor.Direction.REVERSE);
    }

    // Simple method to spin both flywheels
    public void setFlywheelPower(double power) {
        leftFlywheel.setPower(power);
        rightFlywheel.setPower(power);
    }

    // Optional stop method
    public void stop() {
        leftFlywheel.setPower(0);
        rightFlywheel.setPower(0);
    }
}
