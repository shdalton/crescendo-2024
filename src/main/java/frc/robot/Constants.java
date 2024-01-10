// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/* ****************************************
 * TODO: remove unnecessary stuff
 * ****************************************/

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.WCPSwerveModule.WCPSwerveModuleConfig;

public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class WCPSwerveModule {

    public static final WCPSwerveModuleConfig[] kConfigs = {
      new WCPSwerveModuleConfig(2, 1, 3, 9650),
      new WCPSwerveModuleConfig(4, 3, 4, 20756.0),
      new WCPSwerveModuleConfig(6, 5, 5, 1072.0),
      new WCPSwerveModuleConfig(8, 7, 6, 13555.0)
    };
    public static final Translation2d[] kLocations = {
      new Translation2d(0.3525, 0.275),
      new Translation2d(0.3525, -0.275),
      new Translation2d(-0.3525, -0.275),
      new Translation2d(-0.3525, 0.275)
    };

    public static final double kAnalogToDeg = 360.0 / 28000;
    public static final double kDegToAnalog = 1.0 / kAnalogToDeg;

    public static final double kTickToMeter = 2.1 / 102260;
    public static final double kTickToMeterPerS = 10.0 * kTickToMeter;

    public static final double kMeterPerSToTick = 1.0 / kTickToMeterPerS;

    public static final double kTurnKp = 0.15;
    public static final double kTurnKi = 0.0;
    public static final double kTurnKd = 0.05;
    public static final double kTurnIZone = 0.0;

    public static final double kDriveKp = 0.02;
    public static final double kDriveKi = 0.0;
    public static final double kDriveKd = 0.08;
    public static final double kDriveKf = 0.04625;
    public static final double kDriveIZone = 0.0;
  }

  public static class HyperionSwerveModule {

    // public static final HyperionSwerveModuleConfig[] kConfigs = {
    //   new HyperionSwerveModuleConfig(7, 6, 0, 455, InvertType.InvertMotorOutput, false),
    //   new HyperionSwerveModuleConfig(8, 5, 1, 871, InvertType.InvertMotorOutput, true),
    //   new HyperionSwerveModuleConfig(10, 9, 2, 721, InvertType.InvertMotorOutput, false)
    // };

    public static final Translation2d[] kLocations = {
      new Translation2d(0.28, 0.0), new Translation2d(-0.28, 0.28), new Translation2d(-0.28, -0.28)
    };

    public static final double kAnalogToDeg = 360.0 / 1024;
    public static final double kDegToAnalog = 1.0 / kAnalogToDeg;

    public static final double kTickToMeter = 1.0 / 3243;
    public static final double kTickToMeterPerS = 10.0 * kTickToMeter;

    public static final double kMeterPerSToTick = 1.0 / kTickToMeterPerS;

    public static final double kTurnKp = 10.0;
    public static final double kTurnKi = 0.0;
    public static final double kTurnKd = 0.0;
    public static final double kTurnIZone = 0.0;

    public static final double kDriveKp = 3.0;
    public static final double kDriveKi = 0.0;
    public static final double kDriveKd = 0.0;
    public static final double kDriveKf = 0.0; // 0.5;
    public static final double kDriveIZone = 0.0;
  }
}
