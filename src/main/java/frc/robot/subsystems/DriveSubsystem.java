package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

  public class DriveSubsystem extends SubsystemBase
  {
    SwerveDrive swerveDrive;
    public DriveSubsystem(File directory) {
        try
        {
          this.swerveDrive = new SwerveParser(directory).createSwerveDrive(5);
          // Alternative method if you don't want to supply the conversion factor via JSON files.
          // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
        } catch (Exception e)
        {
          throw new RuntimeException(e);
        }
      } 
    
    /**
     * Command to drive the robot using translative values and heading as a setpoint.
     *
     * @param translationX Translation in the X direction. Cubed for smoother controls.
     * @param translationY Translation in the Y direction. Cubed for smoother controls.
     * @param headingX     Heading X to calculate angle of the joystick.
     * @param headingY     Heading Y to calculate angle of the joystick.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                                DoubleSupplier headingY)
    {
      ChassisSpeeds speeds = new ChassisSpeeds();
      // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
      return run(() -> {
  
        Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                   translationY.getAsDouble()), 3.0);
  

        speeds.vxMetersPerSecond = scaledInputs.getX();
        speeds.vyMetersPerSecond = scaledInputs.getY();
        speeds.omegaRadiansPerSecond = MathUtil.applyDeadband(headingX.getAsDouble(), 0.05) * 5;
        // Make the robot move
        driveFieldOriented(speeds);
        // driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
        //                                                                 headingX.getAsDouble(),
        //                                                                 headingY.getAsDouble(),
        //                                                                 swerveDrive.getOdometryHeading().getRadians(),
        //                                                                 swerveDrive.getMaximumChassisVelocity()));
      });
    }

    public void seedForwards() {
      var pose = swerveDrive.getPose();
      swerveDrive.resetOdometry(new Pose2d(pose.getX(), pose.getY(), Rotation2d.kZero));
    }
    
    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public void driveFieldOriented(ChassisSpeeds velocity)
    {
      swerveDrive.driveFieldOriented(velocity);
    }

    @Override 
    public void periodic() {
      SmartDashboard.putNumber("Adjusted Front Left", swerveDrive.getModules()[0].getAbsolutePosition());
      SmartDashboard.putNumber("Adjusted Front Right", swerveDrive.getModules()[1].getAbsolutePosition());
      SmartDashboard.putNumber("Adjusted Back Left", swerveDrive.getModules()[2].getAbsolutePosition());
      SmartDashboard.putNumber("Adjusted Back Right", swerveDrive.getModules()[3].getAbsolutePosition());
      SmartDashboard.putNumber("heading", swerveDrive.getOdometryHeading().getDegrees());
    }
}
