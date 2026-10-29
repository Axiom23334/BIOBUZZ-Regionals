package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

public class MechanismSubsystem extends SubsystemBase {
    private final DcMotorEx turretMotor;
    private final DcMotorEx intakeMotor;
    private final DcMotorEx transferMotor;
    private final DcMotorEx shooterMotor;

    // PID Controller for 3500 RPM shooter control
    private final PIDFController shooterPid = new PIDFController(0.0008, 0.00005, 0.00001, 0.00025);
    public static double SHOOTER_TICKS_PER_REV = 28.0;

    public MechanismSubsystem(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
    }

    public void runIntakeOnly() {
        if (intakeMotor != null) intakeMotor.setPower(1.0);
        if (transferMotor != null) transferMotor.setPower(0.0);
    }

    public void runTransfer() {
        if (intakeMotor != null) intakeMotor.setPower(1.0);
        if (transferMotor != null) transferMotor.setPower(1.0);
    }

    public void stopIntakeAndTransfer() {
        if (intakeMotor != null) intakeMotor.setPower(0.0);
        if (transferMotor != null) transferMotor.setPower(0.0);
    }

    public void runShooterPID(double targetRPM) {
        if (shooterMotor != null) {
            double currentTicksPerSec = shooterMotor.getVelocity();
            double currentRPM = (currentTicksPerSec * 60.0) / SHOOTER_TICKS_PER_REV;
            shooterPid.setSetPoint(targetRPM);
            double power = shooterPid.calculate(currentRPM);
            shooterMotor.setPower(Range.clip(power, -1.0, 1.0));
        }
    }

    public void setTurretPower(double power) {
        if (turretMotor != null) turretMotor.setPower(power);
    }

    public void stopAll() {
        if (intakeMotor != null) intakeMotor.setPower(0.0);
        if (transferMotor != null) transferMotor.setPower(0.0);
        if (shooterMotor != null) shooterMotor.setPower(0.0);
        if (turretMotor != null) turretMotor.setPower(0.0);
        shooterPid.reset();
    }

    public DcMotorEx getTurretMotor() { return turretMotor; }
    public DcMotorEx getIntakeMotor() { return intakeMotor; }
    public DcMotorEx getTransferMotor() { return transferMotor; }
    public DcMotorEx getShooterMotor() { return shooterMotor; }
}
