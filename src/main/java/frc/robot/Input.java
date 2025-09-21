package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Input {
    private static final XboxController driveController = new XboxController(0);

    public static double getTranslationX() {
        return driveController.getLeftX();
    }

    public static double getTranslationY() {
        return driveController.getLeftY();
    }

    public static double getRotation() {
        return driveController.getRightY();
    }
    
}
