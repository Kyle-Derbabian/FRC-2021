/*

  This is the cleaned-up version of last year's final competition code. Minimal changes were made starting November 2020.

  NOTES:

  fix drive train inversion and drive train code so left stick controls left wheels and right stick controls right wheels.

  MAKE A SINGLE BUTTON TO BREAK ALL WHILE LOOPS (or just don't use while loops)

  fix 44 and 55

  here's the temporary shooter code:     //leftShooterMotorController.set((-rightJoystick.getRawAxis(3) + 1) / 2);      //    it's not very good.

*/

package frc.robot;

// Imports WPI classes
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

// Imports CTRE classes.
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

// Imports REV Robotics classes.
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;

public class Robot extends TimedRobot {

  // Autonomous
  Timer autonomousTimer = new Timer();
  boolean autonomousStopper = false;

  // Joysticks
  Joystick leftJoystick = new Joystick(0);
  Joystick rightJoystick = new Joystick(1);
  Joystick thirdJoystick = new Joystick(2);

  // Intake and Hopper
  TalonSRX intakeMotorController = new TalonSRX(0);
  TalonSRX leftHopperMotorController = new TalonSRX(1);
  TalonSRX rightHopperMotorController = new TalonSRX(2);

  // Spinner
  TalonSRX spinnerMotorController = new TalonSRX(3);

  // Lift
  TalonSRX leftLiftMotorController = new TalonSRX(4);
  TalonSRX rightLiftMotorController = new TalonSRX(5);

  // Drive Train
  CANSparkMax frontLeftDriveTrainMotorController = new CANSparkMax(6, MotorType.kBrushless);
  CANSparkMax backLeftDriveTrainMotorController = new CANSparkMax(7, MotorType.kBrushless);
  CANSparkMax frontRightDriveTrainMotorController = new CANSparkMax(8, MotorType.kBrushless);
  CANSparkMax backRightDriveTrainMotorController = new CANSparkMax(9, MotorType.kBrushless);
  CANEncoder frontLeftDriveTrainMotorEncoder = frontLeftDriveTrainMotorController.getEncoder();
  CANEncoder backLeftDriveTrainMotorEncoder = backLeftDriveTrainMotorController.getEncoder();
  CANEncoder frontRightDriveTrainMotorEncoder = frontRightDriveTrainMotorController.getEncoder();
  CANEncoder backRightDriveTrainMotorEncoder = backRightDriveTrainMotorController.getEncoder();

  // Shooter
  CANSparkMax leftShooterMotorController = new CANSparkMax(44, MotorType.kBrushless);
  CANSparkMax rightShooterMotorController = new CANSparkMax(55, MotorType.kBrushless);
  CANEncoder leftShooterMotorEncoder = leftShooterMotorController.getEncoder();
  CANEncoder rightShooterMotorEncoder = rightShooterMotorController.getEncoder();
  double autonomousShooterPower;

  // Limelight
  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry horizontalOffsetEntry = limelightTable.getEntry("tx");
  NetworkTableEntry verticalOffsetEntry = limelightTable.getEntry("ty");
  NetworkTableEntry targetAreaEntry = limelightTable.getEntry("ta");
  NetworkTableEntry targetValidityEntry = limelightTable.getEntry("tv");
  double horizontalOffset;
  double verticalOffset;
  double targetArea;
  double targetValidity;
  final double limelightHeight = 33.5;    // Declares and initializes a new double variable to represent the height of the Limelight from the ground in inches.
  final double targetHeight = 97;         // Declares and initializes a new double variable to represent the height of the target from the ground in inches.
  final double limelightAngle = 3;        // Declares and initializes a new double variable to represent the angle of the Limelight with respect to the horizontal degrees.
  double targetDistance;                  // Declares a new double variable to represent the horizontal distance from the Limelight to the base of the target in inches.

  // Color Sensor
  I2C.Port i2cPort = I2C.Port.kOnboard;
  ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  ColorMatch colorMatcher = new ColorMatch(); 
  Color detectedColor;
  ColorMatchResult match;
  String colorString;
  String wantedColorString;
  String gameDataString;
  final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  // Distance Sensor
  Port distanceSensorPort = Port.kOnboard;
  Rev2mDistanceSensor distanceSensor = new Rev2mDistanceSensor(distanceSensorPort);

  // Compressor
  Compressor compressor = new Compressor();
  Solenoid intakeArmSolenoid = new Solenoid(2);
  DoubleSolenoid ratchetSolenoid = new DoubleSolenoid(0, 1);

  @Override
  /**
   * Control the robot upon enabling the robot.
   */
  public void robotInit() {

    // Resets the autonomous stopper boolean.
    autonomousStopper = false;

    // Resets the state of the compressor.
    compressor.clearAllPCMStickyFaults();

    // Starts the compressor.
    compressor.start();

    // Inverts and links motors if necessary.
    intakeMotorController.setInverted(true);
    leftHopperMotorController.setInverted(false);
    rightHopperMotorController.setInverted(true);
    spinnerMotorController.setInverted(false);
    leftLiftMotorController.setInverted(true);
    rightLiftMotorController.setInverted(false);
    frontLeftDriveTrainMotorController.setInverted(false);
    backLeftDriveTrainMotorController.setInverted(false);
    frontRightDriveTrainMotorController.setInverted(true);
    backRightDriveTrainMotorController.setInverted(true);
    rightShooterMotorController.follow(leftShooterMotorController, true);

    // Adds colors to the color matcher
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kYellowTarget);

  }

  /**
   * Control the robot upon enabling autonomous mode.
   */
  @Override
  public void autonomousInit() {

    // Starts the autonomous timer.
    autonomousTimer.start();

    // Resets the autonomous stopper boolean.
    autonomousStopper = false;

    // Resets drive-train encoders.
    frontLeftDriveTrainMotorEncoder.setPosition(0);
    backLeftDriveTrainMotorEncoder.setPosition(0);
    frontRightDriveTrainMotorEncoder.setPosition(0);
    backRightDriveTrainMotorEncoder.setPosition(0);

  }

  /**
   * Control the robot after enabling autonomous mode.
   */
  @Override
  public void autonomousPeriodic() {

    // Drives to a reasonable shooting distance.
    while ((frontLeftDriveTrainMotorEncoder.getPosition() < 35) && !autonomousStopper) {

      // Adds the encoder values to SmartDashboard
      SmartDashboard.putNumber("ENCODER", frontLeftDriveTrainMotorEncoder.getPosition());

      // Drives away from the target.
      frontLeftDriveTrainMotorController.set(0.1);
      backLeftDriveTrainMotorController.set(0.1);
      frontRightDriveTrainMotorController.set(0.1);
      backRightDriveTrainMotorController.set(0.1);

    }

    // Aims and stops moving the robot.
    if (!autonomousStopper) {

      prepareToShoot();

      // Ensures this code block executes exactly once.
      autonomousStopper = true;

      // Stops driving.
      frontLeftDriveTrainMotorController.set(0);
      backLeftDriveTrainMotorController.set(0);
      frontRightDriveTrainMotorController.set(0);
      backRightDriveTrainMotorController.set(0);

    }

    // Shoots once the robot has stopped moving and is aligned with the target.
    if (Math.abs(horizontalOffset) < 0.5) {

      // Runs hopper motors forward.
      leftHopperMotorController.set(ControlMode.PercentOutput, 0.3);
      rightHopperMotorController.set(ControlMode.PercentOutput, 0.3);

    }

  }

  /**
   * Control the robot upon enabling tele-operated mode.
   */
  @Override
  public void teleopInit() {

    // Stops the robot.
    frontLeftDriveTrainMotorController.set(0);
    backLeftDriveTrainMotorController.set(0);
    frontRightDriveTrainMotorController.set(0);
    backRightDriveTrainMotorController.set(0);

    // Starts the camera streams.
    CameraServer.getInstance().startAutomaticCapture(0);
    CameraServer.getInstance().startAutomaticCapture(1);

    // Resets drive-train encoders.
    frontLeftDriveTrainMotorEncoder.setPosition(0);
    backLeftDriveTrainMotorEncoder.setPosition(0);
    frontRightDriveTrainMotorEncoder.setPosition(0);
    backRightDriveTrainMotorEncoder.setPosition(0);

    // Allows the distance sensor to read measurements and store values.
    distanceSensor.setAutomaticMode(true);

  }

  /**
   * Control the robot after enabling tele-operated mode.
   */
  @Override
  public void teleopPeriodic() {

    SmartDashboard.putNumber("RPM", leftShooterMotorEncoder.getVelocity());

    if (thirdJoystick.getRawButton(4)) {
      leftHopperMotorController.set(ControlMode.PercentOutput, 0.6);
      rightHopperMotorController.set(ControlMode.PercentOutput, 0.6);
    }

    if (leftJoystick.getRawButton(1)) {
      prepareToShoot();
    }

    if (leftJoystick.getRawButton(2)) {
      leftShooterMotorController.set(0);
    }

    // Stops the intake, hopper, spinner, and lift motors.
    intakeMotorController.set(ControlMode.PercentOutput, 0);
    leftHopperMotorController.set(ControlMode.PercentOutput, 0);
    rightHopperMotorController.set(ControlMode.PercentOutput, 0);
    spinnerMotorController.set(ControlMode.PercentOutput, 0);
    leftLiftMotorController.set(ControlMode.PercentOutput, 0);
    rightLiftMotorController.set(ControlMode.PercentOutput, 0);

    driveRobot();

    if (leftJoystick.getRawButtonPressed(1)) {
      prepareToShoot();
    }

    if (rightJoystick.getRawButton(1)) {
      leftHopperMotorController.set(ControlMode.PercentOutput, 0.32);
      rightHopperMotorController.set(ControlMode.PercentOutput, 0.32);
    }

    if (rightJoystick.getRawButton(2)) {
      freeSpin();
    }

    if (rightJoystick.getRawButton(3)) {
      beginRotationControl();
    }

    if (rightJoystick.getRawButton(4)) {
      beginPositionControl();
    }

    if (thirdJoystick.getRawAxis(1) > 0.9) {
      intakeArmSolenoid.set(true);
    }

    if (thirdJoystick.getRawAxis(1) < -0.9) {
      intakeArmSolenoid.set(false);
    }

    if (thirdJoystick.getRawAxis(2) > 0.9) {
      outtake();
    }

    if (thirdJoystick.getRawAxis(3) > 0.9) {
      intake();
    }

    if (thirdJoystick.getRawButton(1)) {
      movePowerCellUp();
    }

    if (thirdJoystick.getRawButton(5)) {
      moveHopperDown();
    }

    if (thirdJoystick.getRawButton(6)) {
      moveHopperUp();
    }

    if (thirdJoystick.getPOV() == 270) {
      freeRatchet();
    }

    if (thirdJoystick.getPOV() == 90) {
      lockRatchet();
    }

    if (thirdJoystick.getPOV() == 0) {
      moveLiftUp();
    }

    if (thirdJoystick.getPOV() == 180) {
      moveLiftDown();
    }

  }

  /**
   * Control the robot after enabling test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * Control the robot upon disabling the robot.
   */
  @Override
  public void disabledInit() {
    distanceSensor.setAutomaticMode(false);
    leftShooterMotorController.set(0);  
  }

  /**
   * Drive the robot using tank-drive controls.
   */
  public void driveRobot() {

    // Maps each motor controller to the cube of its respective joystick axis value.
    frontLeftDriveTrainMotorController.set(Math.pow(rightJoystick.getRawAxis(1), 3));
    backLeftDriveTrainMotorController.set(Math.pow(rightJoystick.getRawAxis(1), 3));
    frontRightDriveTrainMotorController.set(Math.pow(leftJoystick.getRawAxis(1), 3));
    backRightDriveTrainMotorController.set(Math.pow(leftJoystick.getRawAxis(1), 3));

  }

  /**
   * Freely spin the CONTROL PANEL.
   */
  public void freeSpin() {
    spinnerMotorController.set(ControlMode.PercentOutput, 0.6);
  }

  /**
   * Update color variables and displays them on Driver Station and SmartDashboard.
   */
  public void updateColorVariables() {

    // Initializes the detected color.
    detectedColor = colorSensor.getColor();

    // Initializes the closest match to the detected color.
    match = colorMatcher.matchClosestColor(detectedColor);

    // Checks if the detected color is blue to create a new String object to represent the color string as "B".
    if (match.color == kBlueTarget) {

      // Creates a new String object to represent the color string as "B".
      colorString = new String("B");

    }

    // Checks if the detected color is red to create a new String object to represent the color string as "R".
    else if (match.color == kRedTarget) {

      // Creates a new String object to represent the color string as "R".
      colorString = new String("R");

    }

    // Checks if the detected color is green to create a new String object to represent the color string as "G".
    else if (match.color == kGreenTarget) {

      // Creates a new String object to represent the color string as "G".
      colorString = new String("G");

    }

    // Checks if the detected color is yellow to create a new String object to represent the color string as "Y".
    else if (match.color == kYellowTarget) {

      // Creates a new String object to represent the color string as "Y".
      colorString = new String("Y");

    }

    // Checks if the detected color is unknown to create a new String object to represent the color string as "U".
    else {

      // Creates a new String object to represent the color string as "U".
      colorString = new String("U");

    }

    // Initializes the Game Data string, which represents what color the ARENA's color sensor should look for.
    gameDataString = DriverStation.getInstance().getGameSpecificMessage();

    // Checks if the value of the Game Data string is "B" in order to create a new String object to represent the wanted color string as "R".
    if (gameDataString.equals("B")) {

      // Creates a new String object to represent the wanted color string as "R".
      wantedColorString = new String("R");

    }

    // Checks if the value of the Game Data string is "R" in order to create a new String object to represent the wanted color string as "B".
    else if (gameDataString.equals("R")) {

      // Creates a new String object to represent the wanted color string as "B".
      wantedColorString = new String("B");
    
    }

    // Checks if the value of the Game Data string is "G" in order to create a new String object to represent the wanted color string as "Y".
    else if (gameDataString.equals("G")) {

      // Creates a new String object to represent the wanted color string as "Y".
      wantedColorString = new String("Y");

    }

    // Checks if the value of the Game Data string is "Y" in order to create a new String object to reepresent the wanted color string as "G".
    else if (gameDataString.equals("Y")) {

      // Creates a new String object to represent the wanted color string as "G".
      wantedColorString = new String("G");

    }

    // Checks if the value of the Game Data string is "" in order to create a new String object to represent the wanted color string as "".
    else {

      // Creates a new String object to represent the wanted color string as "".
      wantedColorString = new String("");

    }

    // Prints the detected color string to FRC Driver Station.
    DriverStation.reportWarning(colorString, false);

    // Prints the detected color string to FRC SmartDashboard.
    //SmartDashboard.putString("Detected Color", colorString);

    // Prints the wanted color string to FRC SmartDashboard.
    //SmartDashboard.putString("Wanted Color", wantedColorString);

    // Prints the Game Data string to FRC SmartDashboard.
    //SmartDashboard.putString("Game Data", gameDataString);
  
  }

  /**
   * Spin the CONTROL PANEL for more than 3 revolutions but fewer than 5 revolutions.
   */
  public void beginRotationControl() {

    // Loops through 16 iterations in order to spin the CONTROL PANEL approximately 4 times.
    for (int i = 0; i < 16; i++) {

      // Updates the color variables.
      updateColorVariables();

      // Checks if button 2 on the left joystick is pressed in order to stop the spinner motor.
      if (leftJoystick.getRawButton(2)) {

        // Stops the spinner motor.
        spinnerMotorController.set(ControlMode.PercentOutput, 0);

        // Breaks the for loop.
        break;

      }

      // Checks if the detected color is red in order to spin the CONTROL PANEL to blue.
      if (match.color == kRedTarget) {

        // Loops as long as the detected color is not blue in order to spin the CONTROL PANEL to blue.
        while (match.color != kBlueTarget) {

          // Updates the color variables.
          updateColorVariables();

          // Activates the spinner motor.
          spinnerMotorController.set(ControlMode.PercentOutput, 0.6);

          // Checks if button 2 on the left joystick is pressed in order to stop the spinner motor.
          if (leftJoystick.getRawButton(2)) {

            // Stops the spinner motor.
            spinnerMotorController.set(ControlMode.PercentOutput, 0);

            // Breaks the while loop.
            break;

          }

        }

      }

      // Checks if the detected color is blue in order to spin the CONTROL PANEL to red.
      else if (match.color == kBlueTarget) {

        // Loops as long as the detected color is not red in order to spin the CONTROL PANEL to red.
        while (match.color != kRedTarget) {

          // Updates the color variables.
          updateColorVariables();

          // Activates the spinner motor.
          spinnerMotorController.set(ControlMode.PercentOutput, 0.6);

          // Checks if button 2 on the left joystick is pressed in order to stop the spinner motor.
          if (leftJoystick.getRawButton(2)) {

            // Stops the spinner motor motor.
            spinnerMotorController.set(ControlMode.PercentOutput, 0);

            // Breaks the while loop.
            break;

          }

        }

      }

      // Checks if the detected color is neither blue nor red in order to spin the CONTROL PANEL to either red.
      else {

        // Loops as long as the detected color is neither red nor blue in order to spin the CONTROL PANEL to red.
        while (match.color != kRedTarget) {

          // Updates the color variables.
          updateColorVariables();

          // Activates the spinner motor.
          spinnerMotorController.set(ControlMode.PercentOutput, 0.6);

          // Checks if button 2 on the left joystick is pressed in order to stop the spinner motor.
          if (leftJoystick.getRawButton(2)) {

            // Stops the spinner motor.
            spinnerMotorController.set(ControlMode.PercentOutput, 0);

            // Breaks the while loop.
            break;

          }

        }

      }

    }

    // Stops the spinner motor.
    spinnerMotorController.set(ControlMode.PercentOutput, 0);

  }

  /**
   * Spin the CONTROL PANEL unitl the specified color is aligned with the ARENA's color sensor.
   */
  public void beginPositionControl() {

    // Updates the color variables.
    updateColorVariables();

    // Loops as long as the detected color is not the wanted color in order to align the specified color with the ARENA's color sensor.
    while (!colorString.equals(wantedColorString)) {

      // Updates the color variables.
      updateColorVariables();

      // Activates the spinner motor.
      spinnerMotorController.set(ControlMode.PercentOutput, 0.6);

      // Checks if button 2 on the left joystick is pressed in order to stop the spinner.
      if (leftJoystick.getRawButton(2)) {

        // Stops the spinner motor.
        spinnerMotorController.set(ControlMode.PercentOutput, 0);

        // Breaks the while loop.
        break;

      }

    }

    // Stops the spinner motor.
    spinnerMotorController.set(ControlMode.PercentOutput, 0);

  }

  /**
   * Move a POWER CELL up the intake and hopper system.
   */
  public void movePowerCellUp() {
    
    // Checks if the distance sensor's range is less than 0.3 to move the POWER CELL up.
    while (distanceSensor.isRangeValid() && distanceSensor.getRange() < 4)  {

      // Spins the intake motor forward.
      intakeMotorController.set(ControlMode.PercentOutput, 0.3);

      // Spins the left hopper motor forward.
      leftHopperMotorController.set(ControlMode.PercentOutput, 0.3);

      // Spins the right hopper motor forward.
      rightHopperMotorController.set(ControlMode.PercentOutput, 0.3);

      // Checks if the distance sensor yields an invalid range to stop the intake, left hopper, and right hopper motors.
      if (distanceSensor.getRange() == -1) {

        // Stops the intake motor.
        intakeMotorController.set(ControlMode.PercentOutput, 0);

        // Stops the left hopper motor.
        leftHopperMotorController.set(ControlMode.PercentOutput, 0);

        // Stops the right hopper motor.
        rightHopperMotorController.set(ControlMode.PercentOutput, 0);

        // Breaks the while loop.
        break;
        
      }

      // Checks if button 2 on the left joystick is pressed in order to stop the intake, left hopper, and right hopper motors.
      if (leftJoystick.getRawButton(2)) {

        // Stops the intake motor.
        intakeMotorController.set(ControlMode.PercentOutput, 0);

        // Stops the left hopper motor.
        leftHopperMotorController.set(ControlMode.PercentOutput, 0);

        // Stops the right hopper motor.
        rightHopperMotorController.set(ControlMode.PercentOutput, 0);

        // Breaks the while loop.
        break;

      }

    }
  
  }

  /**
   * Move the intake arm up.
   */
  public void moveIntakeArmUp() {
    intakeArmSolenoid.set(true);
  }

  /**
   * Move the intake arm down.
   */
  public void moveIntakeArmDown() {
    intakeArmSolenoid.set(false);
  }

  /**
   * Intake a POWER CELL.
   */
  public void intake() {
    intakeMotorController.set(ControlMode.PercentOutput, 0.5);
  }

  /**
   * Outtake a POWER CELL.
   */
  public void outtake() {
    intakeMotorController.set(ControlMode.PercentOutput, -0.5);
  }

  /**
   * Move a POWER CELL up the hopper.
   */
  public void moveHopperUp() {

    // Spins the left hopper motor forward.
    leftHopperMotorController.set(ControlMode.PercentOutput, 0.4);

    // Spins the right hopper motor forward.
    rightHopperMotorController.set(ControlMode.PercentOutput, 0.4);

  }

  /**
   * Move a POWER CELL down the hopper.
   */
  public void moveHopperDown() {

    // Spins the left hopper motor backward.
    leftHopperMotorController.set(ControlMode.PercentOutput, -0.4);

    // Spins the right hopper motor backward.
    rightHopperMotorController.set(ControlMode.PercentOutput, -0.4);

  } 

  /**
   * Update Limelight variables and display them on SmartDashboard.
   */
  public void updateLimelightVariables() {

    // Initializes the horizontal offset.
    horizontalOffset = horizontalOffsetEntry.getDouble(0.0);

    // Initializes the vertical offset.
    verticalOffset = verticalOffsetEntry.getDouble(0.0);

    // Initializes the target area.
    targetArea = targetAreaEntry.getDouble(0.0);

    // Initializes the target validity.
    targetValidity = targetValidityEntry.getDouble(0.0);    

    // Initializes the target distance.
    targetDistance = (targetHeight - limelightHeight) / Math.tan((limelightAngle + verticalOffset) * Math.PI / 180);

    // Prints the horizontal offset double to FRC SmartDashboard.
    SmartDashboard.putNumber("Horizontal Offset", horizontalOffset);
    
    // Prints the vertical offset double to FRC SmartDashboard.
    //SmartDashboard.putNumber("Vertical Offset", verticalOffset);

    // Prints the target area double to FRC SmartDashboard.
    //SmartDashboard.putNumber("Target Area", targetArea);

    // Prints the target validity double to FRC SmartDashboard.
    //SmartDashboard.putNumber("Target Validity", targetValidity);

    // Prints the target distance double to FRC SmartDashboard.
    SmartDashboard.putNumber("Target Distance", targetDistance);

  }

  /**
   * Convert the distance from the target to an autonomous shooter power.
   * @param targetDistance The distance from the target, in inches.
   * @return The power to be applied to the autonomous shooter.
   */
  public double convertTargetDistanceToAutonomousShooterPower(double targetDistance) {

    // Updates the Limelight variables.
    updateLimelightVariables();

    // Returns the autonomous shooter power based on the horizontal distance from the Limelight to the base of the target.
    return 1.5 / ((targetDistance / 12) - 10) + 0.016 * (targetDistance / 12) + 0.20;

  }

  public void autonomousAiming() {

    // Updates the Limelight variables.
    updateLimelightVariables();

    leftShooterMotorController.set(0.95);

    if (leftJoystick.getRawButton(2)) {

      leftShooterMotorController.set(0);

    }

    // Loops as long as the absolute horizontal offset from the Limelight to the target is less than 0.3.
    while (Math.abs(horizontalOffset) > 0.13) {

      // Updates the Limelight variables.
      updateLimelightVariables();

      //eftShooterMotorController.set(0.95);

      // Checks if the horizontal offset is greater than 0, meaning the robot is to the right of the target, to align the robot by turning it left.
      if (horizontalOffset > 0) {

        frontLeftDriveTrainMotorController.set(0.1);
        backLeftDriveTrainMotorController.set(0.1);
        frontRightDriveTrainMotorController.set(-0.1);
        backRightDriveTrainMotorController.set(-0.1);

      }
      
      // Checks if the horizontal offset is less than 0, meaning the robot is to the left of the target, to align the robot by turning it right.
      if (horizontalOffset < 0) {

        frontLeftDriveTrainMotorController.set(-0.1);
        backLeftDriveTrainMotorController.set(-0.1);
        frontRightDriveTrainMotorController.set(0.1);
        backRightDriveTrainMotorController.set(0.1);

      }

      if (leftJoystick.getRawButton(2)) {

        frontLeftDriveTrainMotorController.set(0);
        backLeftDriveTrainMotorController.set(0);
        frontRightDriveTrainMotorController.set(0);
        backRightDriveTrainMotorController.set(0);

        break;

      }

    }

    frontLeftDriveTrainMotorController.set(0);
    backLeftDriveTrainMotorController.set(0);
    frontRightDriveTrainMotorController.set(0);
    backRightDriveTrainMotorController.set(0);

  }

  /**
   * Aim the robot at the target and turns on shooter motors.
   */
  public void prepareToShoot() {

    // Updates the Limelight variables.
    updateLimelightVariables();

    if (targetValidity == 1) {
      leftShooterMotorController.set(convertTargetDistanceToAutonomousShooterPower(targetDistance));
    } else {
      leftShooterMotorController.set(1);
    }

    SmartDashboard.putNumber("Shooter Power", convertTargetDistanceToAutonomousShooterPower(targetDistance));

    if (leftJoystick.getRawButton(2)) {

      leftShooterMotorController.set(0);

    }

    // Loops as long as the absolute horizontal offset from the Limelight to the target is less than 0.3.
    while (Math.abs(horizontalOffset) > 0.1) {

      // Updates the Limelight variables.
      updateLimelightVariables();

      leftShooterMotorController.set(convertTargetDistanceToAutonomousShooterPower(targetDistance));

      // Checks if the horizontal offset is greater than 0, meaning the robot is to the right of the target, to align the robot by turning it left.
      if (horizontalOffset > 0) {

        frontLeftDriveTrainMotorController.set(0.09);
        backLeftDriveTrainMotorController.set(0.09);
        frontRightDriveTrainMotorController.set(-0.09);
        backRightDriveTrainMotorController.set(-0.09);

      }
      
      // Checks if the horizontal offset is less than 0, meaning the robot is to the left of the target, to align the robot by turning it right.
      if (horizontalOffset < 0) {

        frontLeftDriveTrainMotorController.set(-0.09);
        backLeftDriveTrainMotorController.set(-0.09);
        frontRightDriveTrainMotorController.set(0.09);
        backRightDriveTrainMotorController.set(0.09);

      }

      if (leftJoystick.getRawButton(2)) {

        frontLeftDriveTrainMotorController.set(0);
        backLeftDriveTrainMotorController.set(0);
        frontRightDriveTrainMotorController.set(0);
        backRightDriveTrainMotorController.set(0);

        break;

      }

    }

    frontLeftDriveTrainMotorController.set(0);
    backLeftDriveTrainMotorController.set(0);
    frontRightDriveTrainMotorController.set(0);
    backRightDriveTrainMotorController.set(0);

  }

  /**
   * Free the ratchet.
   */
  public void freeRatchet() {

    ratchetSolenoid.set(Value.kReverse);

    thirdJoystick.setRumble(RumbleType.kLeftRumble, 0);
    thirdJoystick.setRumble(RumbleType.kRightRumble, 0);

  }

  /**
   * Lock the ratchet.
   */
  public void lockRatchet() {

    ratchetSolenoid.set(Value.kForward);
    
    thirdJoystick.setRumble(RumbleType.kLeftRumble, 1);
    thirdJoystick.setRumble(RumbleType.kRightRumble, 1);

  }

  /**
   * Move the lift up.
   */
  public void moveLiftUp() {

    // Spins the left lift motor forward.
    leftLiftMotorController.set(ControlMode.PercentOutput, 0.5);

    // Spins the right lift motor forward.
    rightLiftMotorController.set(ControlMode.PercentOutput, 0.5);

  }

  /**
   * Move the lift down.
   */
  public void moveLiftDown() {

    // Spins the left lift motor backward.
    leftLiftMotorController.set(ControlMode.PercentOutput, -0.7);

    // Spins the right lift motor backward.
    rightLiftMotorController.set(ControlMode.PercentOutput, -0.7);

  }

}