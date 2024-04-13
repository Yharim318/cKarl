package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.PIDNumbers;
import frc.robot.Constants.Motors.DriveMotors;
import frc.robot.Constants.Motors.SwerveEncoders;
import frc.robot.Constants.Motors.SwerveMotors;
import frc.robot.Constants.Motors.Translations;
import frc.robot.Constants.Motors.kMotors;

public class SwerveWheel {
    private final WPI_TalonSRX driveMotor; // Motor controller object for the drive motor
    public final WPI_TalonSRX swerveMotor; // Motor controller object fo the turning motor
    public final Encoder swerveEncoder; // Encoder object for the turning encoder
    private final Translation2d wheelPos; // Translation object that holds the position of the motor relative to the center of the robot
    private final PIDController swervePID; // PID controller object for error correction
    public SwerveWheel(kMotors pos) {
        swervePID = new PIDController(PIDNumbers.kP, PIDNumbers.kI, PIDNumbers.kD); // Creates a new PIDController object and passes the P, I, and D values to it

        // This takes the input position from the object definition and uses it to set the appropriate values for each object mentioned above
        // Only FL is commented; the rest are identical except for the exact values passed
        switch (pos) {
            case FL:
                driveMotor = new WPI_TalonSRX(DriveMotors.FL); // Creates a new WPI_VictorSPX object at the given CAN ID for the drive motor (WPI version of the VictorSPX object)
                swerveMotor = new WPI_TalonSRX(SwerveMotors.FL); // Same as previous, but fo the turning motor
                swerveMotor.setInverted(SwerveMotors.FLr);
                swerveEncoder = new Encoder(SwerveEncoders.FLa, SwerveEncoders.FLb, // Creates a new Encoder object using the numbered DIO ports (FLa and FLb)
                SwerveEncoders.FLr); // and reverses it if necessary
                wheelPos = new Translation2d(Translations.FLx, Translations.FLy); // Creates a new Translation2d object that hold the position of this swerve module relative to the center of the robot
                swerveEncoder.setDistancePerPulse(SwerveEncoders.encoderRatio);
                break;

            case FR:
                driveMotor = new WPI_TalonSRX(DriveMotors.FR);
                swerveMotor = new WPI_TalonSRX(SwerveMotors.FR);
                swerveMotor.setInverted(SwerveMotors.FRr);
                swerveEncoder = new Encoder(SwerveEncoders.FRa, SwerveEncoders.FRb, SwerveEncoders.FRr);
                wheelPos = new Translation2d(Translations.FRx, Translations.FRy);
                swerveEncoder.setDistancePerPulse(SwerveEncoders.encoderRatio);
                break;

            case BR:
                driveMotor = new WPI_TalonSRX(DriveMotors.BR);
                swerveMotor = new WPI_TalonSRX(SwerveMotors.BR);
                swerveMotor.setInverted(SwerveMotors.BRr);
                swerveEncoder = new Encoder(SwerveEncoders.BRa, SwerveEncoders.BRb, SwerveEncoders.BRr);
                wheelPos = new Translation2d(Translations.BRx, Translations.BRy);
                swerveEncoder.setDistancePerPulse(SwerveEncoders.encoderRatio);
                break;

            case BL:
                driveMotor = new WPI_TalonSRX(DriveMotors.BL);
                swerveMotor = new WPI_TalonSRX(SwerveMotors.BL);
                swerveMotor.setInverted(SwerveMotors.BLr);
                swerveEncoder = new Encoder(SwerveEncoders.BLa, SwerveEncoders.BLb, SwerveEncoders.BLr);
                wheelPos = new Translation2d(Translations.BLx, Translations.BLy);
                swerveEncoder.setDistancePerPulse(SwerveEncoders.encoderRatio);
                break;

            default:
                driveMotor = null;
                swerveMotor = null;
                swerveEncoder = null;
                wheelPos = null;
                break;
        }
        // This sets the distance per pulse on the encoder, using the ration in the constants file to make one full rotation a distance of 360
        

        // This resets the encoder just in case
        swerveEncoder.reset();

        // This enales continuous input (woah) on the PID loop; if not enabled, points directly adjacent on the circle may not be seen as such
        // For example, if the wheel was at 350 degrees and needed to go to 10 degrees, despite being 20 degrees apart, would still go all the way around if this was not enabled
        // (Values used are from -180 degrees to 180 degrees)
        swervePID.enableContinuousInput(-180, 180);

        swervePID.setSetpoint(0);
    }

    // This allows for an outside class to access the location of this swerve module
    public Translation2d getWheelTranslation() {
        return wheelPos;
    }

    public Rotation2d normalize(Rotation2d angle) {
        return angle.minus(Rotation2d.fromDegrees(0));
    }

    // This returns a Rotation2d object with angle equal to the current angle of the wheel
    public Rotation2d getRotation() {
        return normalize(Rotation2d.fromDegrees(swerveMotor.getSelectedSensorPosition()));
    }

    public void setDriveSpeed(double speed) {
        driveMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setSwerveSpeed(double speed) {
        swerveMotor.set(ControlMode.PercentOutput, speed);
    }

    public void updatePID(SwerveModuleState wheelState, XboxController xbox) {
        // This takes the given SwerveModuleState and optimizes it
        // For example, if the wheel were facing at 90 degrees, and had to be moving forward at -75 degrees, it would rotate the wheel to 105 degrees and reverse its direction
        // It limits the rotational distance the wheel needs to cover
        SwerveModuleState optimizedWheelState;
        if (xbox.getAButton()) {
            optimizedWheelState = wheelState;
        }
        else {
            optimizedWheelState = SwerveModuleState.optimize(wheelState, getRotation());
        }
        // This sets the setpoint of the PID loop to the angle determined above
        swervePID.setSetpoint(optimizedWheelState.angle.getDegrees());

        swerveMotor.set(ControlMode.PercentOutput, // Sets the output of the rotation motor
            MathUtil.clamp( // Clamps PID output to between -1 and 1 to keep it in bounds
                swervePID.calculate(getRotation().getDegrees()), -1, 1)); // The calculate command calculates the next iteration of the PID loop given the current angle of the wheel

        driveMotor.set(ControlMode.PercentOutput, // This sets the speed of the wheel to the speed assigned by the optimized SwerveModuleState
        optimizedWheelState.speedMetersPerSecond * DriveMotors.maxSpeed); // and multiplies it by a global maxSpeed multiplier
    }
}
