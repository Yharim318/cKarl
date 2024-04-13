// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final static class PIDNumbers {
        // These are the constants used by the PID loop
        public final static double kP = 0.085; // Proportion
        public final static double kI = 0; // Integral
        public final static double kD = 0.001; // Derivative
    }
    public final static class Motors {
        public final static class Translations { // These are the x and y locations of each module of the swerve drive (relative to the robot center)
            // 8.125in x 14.75in
            public final static double width = 8.125; // In inches, but units don't matter
            public final static double length = 14.75;

            public final static double FLx = length / 2;
            public final static double FLy = width / 2;
            
            public final static double FRx = length / 2;
            public final static double FRy = width / -2;

            public final static double BRx = length / -2;
            public final static double BRy = width / -2;

            public final static double BLx = length / -2;
            public final static double BLy = width / 2;
            public final static class CenterTranslations {
                public final static Translation2d C = new Translation2d(0, 0);
                public final static Translation2d F = new Translation2d(length / 2, 0);
                public final static Translation2d L = new Translation2d(0, width / 2);
                public final static Translation2d B = new Translation2d(-length / 2, 0);
                public final static Translation2d R = new Translation2d(0, -width / 2);
                public final static Translation2d FL = new Translation2d(FLx, FLy);
                public final static Translation2d FR = new Translation2d(FRx, FRy);
                public final static Translation2d BR = new Translation2d(BRx, BRy);
                public final static Translation2d BL = new Translation2d(BLx, BLy); 
                public final static Translation2d FF = new Translation2d(30, 0);
            }
        }

        public final static class DriveMotors {
            public final static double maxSpeed = 0.3; // The maximum output power of the drive motors
            public final static double rotationScalar = 0.1; // The scaling of rotation speed relative to default
            public final static double slewLimit = 1000; // Maximum input rate of change

            // These are the CAN IDs of each drive motor
            public final static int FL = 2;
            public final static int FR = 9;
            public final static int BR = 7;
            public final static int BL = 5;
        }

        public final static class SwerveMotors {
            // These are the CAN IDs of each turning motor
            public final static int FL = 1;
            public final static Boolean FLr = false;

            public final static int FR = 8;
            public final static Boolean FRr = false;

            public final static int BR = 6;
            public final static Boolean BRr = false;

            public final static int BL = 4;
            public final static Boolean BLr = false;
        }

        public final static class SwerveEncoders {
            public final static double encoderRatio = ((360.0 / 71) * (48.0 / 40)) / 7; // The ratio applied to the encoder counts such that one full rotation = 360 distance
            
            // These are the DIO ports used by each encoder, along with if any encoder direction should be reversed
            public final static int FLa = 2;
            public final static int FLb = 3;
            public final static Boolean FLr = false;

            public final static int FRa = 0;
            public final static int FRb = 1;
            public final static Boolean FRr = false;

            public final static int BRa = 5;
            public final static int BRb = 6;
            public final static Boolean BRr = false;

            public final static int BLa = 8;
            public final static int BLb = 9;
            public final static Boolean BLr = false;
        }
        
        // This enumerable is used to shorten the definition of each SwerveWheel object in Robot.java
        public enum kMotors {
            FL,
            FR,
            BR,
            BL
        }
    }
}
