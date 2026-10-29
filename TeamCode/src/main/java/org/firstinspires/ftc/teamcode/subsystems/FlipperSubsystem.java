package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class FlipperSubsystem extends SubsystemBase {
    public static final double INTAKE_POSITION = 0.5;
    private final Servo flipperServo;

    public static double DOWN_POSITION = 0.0;
    public static double UP_POSITION = 1.0;

    public FlipperSubsystem(HardwareMap hardwareMap) {
        flipperServo = hardwareMap.get(Servo.class, "flipper");
    }

    public void setPosition(double position) {
        if (flipperServo != null) {
            flipperServo.setPosition(position);
        }
    }

    public void flipperDown() {
        setPosition(DOWN_POSITION);
    }

    public void flipperUp() {
        setPosition(UP_POSITION);
    }
}
