package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.drivetrain.CustomDrivetrain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.DiffyDriveSubsystem;

/**
 * Pedro Pathing CustomDrivetrain that routes autonomous driving commands
 * to DiffyDriveSubsystem using the exact differential logic from diffyCode.
 */
public class DiffyPedroDrivetrain extends CustomDrivetrain {

    private final DiffyDriveSubsystem diffyDrive;

    public DiffyPedroDrivetrain(HardwareMap hardwareMap) {
        this.diffyDrive = new DiffyDriveSubsystem(hardwareMap);
    }

    public DiffyPedroDrivetrain(DiffyDriveSubsystem diffyDrive) {
        this.diffyDrive = diffyDrive;
    }

    public DiffyDriveSubsystem getDiffyDriveSubsystem() {
        return diffyDrive;
    }

    @Override
    public void arcadeDrive(double forward, double strafe, double rotation) {
        // Pedro Pathing: forward = -gamepad1.left_stick_y, strafe = gamepad1.left_stick_x
        // diffyCode: x = gamepad1.left_stick_y, y = gamepad1.left_stick_x
        diffyDrive.drive(forward, strafe);
    }

    @Override
    public void updateConstants() {
        // No-op for custom diffy drivetrain
    }

    @Override
    public void breakFollowing() {
        diffyDrive.stop();
    }

    @Override
    public void startTeleopDrive() {
        // Handled by SolversLib CommandOpMode in TeleOp
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        // Handled by SolversLib CommandOpMode in TeleOp
    }

    @Override
    public double xVelocity() {
        return 60.0;
    }

    @Override
    public double yVelocity() {
        return 60.0;
    }

    @Override
    public void setXVelocity(double xMovement) {}

    @Override
    public void setYVelocity(double yMovement) {}

    @Override
    public double getVoltage() {
        return 12.0;
    }

    @Override
    public String debugString() {
        return "DiffyPedroDrivetrain";
    }
}
