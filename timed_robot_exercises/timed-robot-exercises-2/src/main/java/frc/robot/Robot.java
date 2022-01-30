// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private final PWMVictorSPX left_front = new PWMVictorSPX(0);
  private final PWMVictorSPX left_middle = new PWMVictorSPX(1);
  private final PWMVictorSPX left_back = new PWMVictorSPX(2);

  private final PWMVictorSPX right_front = new PWMVictorSPX(3);
  private final PWMVictorSPX right_middle = new PWMVictorSPX(4);
  private final PWMVictorSPX right_back = new PWMVictorSPX(5);

  private final SpeedControllerGroup left = new SpeedControllerGroup(left_front, left_middle, left_back);
  private final SpeedControllerGroup right = new SpeedControllerGroup(left_front, left_middle, left_back);

  private final DifferentialDrive drive = new DifferentialDrive(left, right);

  private final PWMVictorSPX elevator1 = new PWMVictorSPX(6);
  private final PWMVictorSPX elevator2 = new PWMVictorSPX(7);

  private final double elevatorLevelZero = 15.0;
  private final double elevatorLevelOne = 40.0;
  private final double elevatorLevelTwo = 60.0;

  private boolean isElevatorZero = true;
  private boolean isElevatorOne = false;
  private boolean isElevatorTwo = false;

  private final SpeedControllerGroup elevator = new SpeedControllerGroup(elevator1, elevator2);

  private final PWMVictorSPX intake1 = new PWMVictorSPX(8);
  private final PWMVictorSPX intake2 = new PWMVictorSPX(9);

  private boolean isIntakeOpened = false;

  private final SpeedControllerGroup intake = new SpeedControllerGroup(intake1, intake2);

  private final Encoder encoder = new Encoder(10, 11);

  private final Joystick joystick = new Joystick(0);

  private final JoystickButton buttonA = new JoystickButton(joystick, 1);
  private final JoystickButton buttonB = new JoystickButton(joystick, 2);
  private final JoystickButton buttonX = new JoystickButton(joystick, 0);
  private final JoystickButton buttonY = new JoystickButton(joystick, 3);

  public Timer timer = new Timer();

  public double getElevatorRaising() {
    encoder.setDistancePerPulse(10 / 1000);
    return encoder.getDistance();

  }

  public void encoderReset() {
    encoder.reset();
  }

  public void elevatorToZero() {
    encoderReset();
    if (isElevatorZero) {
      elevator.set(0);
    }

    else if (isElevatorOne) {
      if (getElevatorRaising() < 25) {
        elevator.set(-0.7);
      } else if (getElevatorRaising() >= 25) {
        elevator.set(0);
        isElevatorOne = false;
        isElevatorZero = true;
      }
    }

    else if (isElevatorTwo) {
      if (getElevatorRaising() < 45) {
        elevator.set(-0.7);
      } else if (getElevatorRaising() >= 45) {
        elevator.set(0);
        isElevatorTwo = false;
        isElevatorZero = true;
      }
    }

  }

  public void elevatorToOne() {
    encoderReset();
    if (isElevatorOne) {
      elevator.set(0);
    }

    else if (isElevatorZero) {
      if (getElevatorRaising() < 25) {
        elevator.set(0.7);
      } else if (getElevatorRaising() >= 25) {
        elevator.set(0);
        isElevatorOne = true;
        isElevatorZero = false;
      }
    }

    else if (isElevatorTwo) {
      if (getElevatorRaising() < 20) {
        elevator.set(-0.7);
      } else if (getElevatorRaising() >= 20) {
        elevator.set(0);
        isElevatorOne = true;
        isElevatorTwo = false;

      }
    }

  }

  public void elevatorToTwo() {
    encoderReset();
    if (isElevatorTwo) {
      elevator.set(0);
    }

    else if (isElevatorZero) {
      if (getElevatorRaising() < 45) {
        elevator.set(0.7);
      } else if (getElevatorRaising() >= 45) {
        elevator.set(0);
        isElevatorTwo = true;
        isElevatorZero = false;
      }
    }

    else if (isElevatorOne) {
      if (getElevatorRaising() < 20) {
        elevator.set(0.7);
      } else if (getElevatorRaising() >= 20) {
        elevator.set(0);
        isElevatorTwo = true;
        isElevatorOne = false;

      }
    }
  }

  public void intakeMove() {
    if (isIntakeOpened) {
      intake.set(-0.7);
      isIntakeOpened = false;
    } else if (!isIntakeOpened) {
      intake.set(0.7);
      isIntakeOpened = true;
    }
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (timer.get() < 5.0) {
      drive.tankDrive(0.5, 0.5);
    } else if (timer.get() < 8.0 & timer.get() >= 5.0) {
      drive.tankDrive(0.3, 0.3);
    } else {
      drive.tankDrive(0, 0);
    }

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    drive.tankDrive(joystick.getRawAxis(1), joystick.getRawAxis(3));

    left.set(joystick.getRawAxis(1));
    right.set(joystick.getRawAxis(3));

    if (buttonA.get()) {
      elevatorToZero();
    } else if (buttonX.get()) {
      elevatorToOne();
    } else if (buttonY.get()) {
      elevatorToTwo();
    } else if (buttonB.get()) {
      intakeMove();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
