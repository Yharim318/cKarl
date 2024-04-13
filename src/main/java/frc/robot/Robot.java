// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.Motors.DriveMotors;
import frc.robot.Constants.Motors.Translations;
import frc.robot.Constants.Motors.kMotors;
import frc.robot.Constants.Motors.Translations.CenterTranslations;
import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Declares a variable for each of the four swerve modules
  SwerveWheel wheelFL;
  SwerveWheel wheelFR;
  SwerveWheel wheelBR;
  SwerveWheel wheelBL;

  // Declares the SwerveDriveKinematics variable to house the swerve modules
  SwerveDriveKinematics swerveDrive;
  
  // Xbox controller
  XboxController xbox;
  SlewRateLimiter xSlew;
  SlewRateLimiter ySlew;
  
  // Declares empty variables for the swerve goal speed and goal states of each swerve module
  ChassisSpeeds goalSpeed;
  SwerveModuleState[] goalStates;
  Translation2d centerRotation;

  // Adds gyro functionality
   AHRS gyro;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Initializes an instance of the SwerveWheel class for each swerve module
    wheelFL = new SwerveWheel(kMotors.FL);
    wheelFR = new SwerveWheel(kMotors.FR);
    wheelBR = new SwerveWheel(kMotors.BR);
    wheelBL = new SwerveWheel(kMotors.BL);

    // Initializes a new object that holds the location of each swerve module
    // This is used when calculating the required angle and speed of each wheel of the module
    swerveDrive = new SwerveDriveKinematics(
      wheelFL.getWheelTranslation(),
      wheelFR.getWheelTranslation(),
      wheelBR.getWheelTranslation(),
      wheelBL.getWheelTranslation()
    );

    // Xbox controller
    xbox = new XboxController(0);

    // Slew Rate Limiters; these keep inputs from changing too rapidly
    xSlew = new SlewRateLimiter(DriveMotors.slewLimit);
    ySlew = new SlewRateLimiter(DriveMotors.slewLimit);
    centerRotation = new Translation2d(0, 0);

    // Initializes the gyro
    gyro = new AHRS(I2C.Port.kOnboard);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    goalSpeed = ChassisSpeeds.fromFieldRelativeSpeeds( // This static method generates a new ChassisSpeeds object based on given velocities
      ySlew.calculate(xbox.getRawAxis(1)), // Xbox controller
      xSlew.calculate(xbox.getRawAxis(0)), // Xbox controller
      // While the input speeds are technically supposed to be m/s, it is easier to assume max joystick is 1 m/s

      // Because of the above assumption, the rotation joystick needs to be scaled to balance movement with rotation
      // For example, without the scalar, the robot would rotate at 1 rad/s at maximum rotate (and thus take the same amount of time to rotate once as to travel six meters)
      // The scalar exists to customize this to fit user need
      xbox.getRawAxis(4) * DriveMotors.rotationScalar, // Xbox controller

      // This is needed because of the field relative nature of this object; if a gyro is used, this should be the gyro
      // If used with a gyro, this allows for the joystick to operate the robot in the same directions regardless of robot orientation
      // i.e. Up on the left joystick always moves the robot away from the user regardless of its rotation
      Rotation2d.fromDegrees(-gyro.getYaw()));
    switch (xbox.getPOV()) {
      case -1: {
        centerRotation = CenterTranslations.C;
        break;
      }
      case 0:
      {
        centerRotation = CenterTranslations.F;
        break;
      }
      case 45:
      {
        centerRotation = CenterTranslations.FR;
        break;
      }
      case 90:
      {
        centerRotation = CenterTranslations.R;
        break;
      }
      case 135:
      {
        centerRotation = CenterTranslations.BR;
        break;
      }
      case 180:
      {
        centerRotation = CenterTranslations.B;
        break;
      }
      case 225:
      {
        centerRotation = CenterTranslations.BL;
        break;
      }
      case 270:
      {
        centerRotation = CenterTranslations.L;
        break;
      }
      case 315:
      {
        centerRotation = CenterTranslations.FL;
        break;
      }
    }
    if (xbox.getYButton()) {
      centerRotation = CenterTranslations.FF;
    }
    if (xbox.getXButton()) {
      gyro.zeroYaw();
    }
    
    // This converts the goal velocity defined above to the speed and rotation of each wheel on the robot (it's pretty cool)
    goalStates = swerveDrive.toSwerveModuleStates(goalSpeed, centerRotation);

    // This scales the velocity of each wheel such that none exceed the maximum speed
    // For example, if one wheel were going at a speed of 2, it would scale it down to 1 and scale each other wheel down by a half
    // A max speed of 1 is used here as well, as it represents the percent of maximum speed as opposed to any actual fixed speed
    SwerveDriveKinematics.desaturateWheelSpeeds(goalStates, 1);

    // System.out.print(wheelFL.getRotation().getDegrees());
    // System.out.print(" - ");
    // System.out.print(wheelFR.getRotation().getDegrees());
    // System.out.print(" - ");
    // System.out.print(wheelBR.getRotation().getDegrees());
    // System.out.print(" - ");
    // System.out.print(wheelBL.getRotation().getDegrees());
    // System.out.println();

    // Tells each SwerveWheel to update its PID loop with its assigned state
    // The index numbers on goalStates are due to the order in which the modules were added above
    wheelFL.updatePID(goalStates[0], xbox);
    wheelFR.updatePID(goalStates[1], xbox);
    wheelBR.updatePID(goalStates[2], xbox);
    wheelBL.updatePID(goalStates[3], xbox);
  }
}
