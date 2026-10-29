package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class FlipperSubsystem extends SubsystemBase {
    private Servo flipperServo;

    public static double INTAKE_POSITION = 1.0;
    public static double REST_POSITION = 0.0;

    public FlipperSubsystem(HardwareMap hardwareMap) {
        try {
            flipperServo = hardwareMap.get(Servo.class, "flipper");
        } catch (Exception e) {
            flipperServo = null;
        }
    }

    public void setPosition(double position) {
        if (flipperServo != null) {
            flipperServo.setPosition(position);
        }
    }

    public void flipperIntake() {
        setPosition(INTAKE_POSITION);
    }

    public void flipperRest() {
        setPosition(REST_POSITION);
    }
}
