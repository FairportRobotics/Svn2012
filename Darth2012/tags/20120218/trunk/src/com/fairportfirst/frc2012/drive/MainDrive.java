package com.fairportfirst.frc2012.drive;

import com.fairportfirst.frc2012.structures.*;
import com.fairportfirst.frc2012.vision.Camera;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * The supreme central class whence all other classes are called
 * @author brennan
 */
public class MainDrive extends IterativeRobot {

    // Statics for CANJaguar
    private static int JAG_LEFT_A = 2;
    private static int JAG_LEFT_B = 4;
    private static int JAG_RIGHT_A = 3;
    private static int JAG_RIGHT_B = 5;
    private static int JAG_SHOOTER_WHEELS_A = 6;
    private static int JAG_SHOOTER_WHEELS_B = 7;
    private static int JAG_TURRET = 9;

    // Statics for PWM (Victors)
    private static int VIC_WEDGE = 1;
    private static int VIC_LIFT = 2;
    private static int VIC_BALLPICKUP = 3;

    // Statics for Digital I/O (Limit Switches)
//    private static int DIO_TURRET_CW_LIMIT = 1;
//    private static int DIO_TURRET_CCW_LIMIT = 2;
    private static int DIO_WEDGE_UP_LIMIT = 3;
    private static int DIO_WEDGE_DOWN_LIMIT = 4;

    // Statics for Drive
    private static int TANK_DRIVE = 1;
    private static int TILT_DRIVE = 2;
    private static int TWIST_DRIVE = 3;
    DriveType driveType;
    private double left = 0;
    private double right = 0;
    int counter = 0;
    AutoSpeed autoSpeed;
    TurretMode turretMode;

    // Statics for Shooter Wheel Speed
    private static double PERCENT_100 = 1;
    private static double PERCENT_66 = 2/3.0;
    private static double PERCENT_33 = 1/3.0;
    private static double PERCENT_0 = 0;

    private static boolean LIFTMANUAL;

    DriverStationLCD dslcd;

    DriveTrain driveTrain;

    Lift mrLift;
    Shooter mrShooter;
    Wedge mrWedge;
    BallPickup mrPickup;
    Relay mrRelay;
    AxisCamera camera;
    Camera spyCam;
    
    boolean relayBool = false;
    boolean isProjectAimbot = true;
//    private DigitalInput turretCwLimit;
//    private DigitalInput turretCcwLimit;
    private DigitalInput wedgeUpLimit;
    private DigitalInput wedgeDownLimit;

    private SendableChooser autoSpeedChooser;
    private SendableChooser cameraAlign;
    private SendableChooser controlChooser;
    private SendableChooser liftChooser;
    private SendableChooser shooterChooser;
    private SendableChooser wedgeChooser;

    private Joystick driveStick1;
    private Joystick driveStick2;
    private Joystick gamepad;  // always the gamepad even with one joystick

    SendableGyro mrGyro;

    /**
     * This method is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        NetworkTable.initialize();
        dslcd = DriverStationLCD.getInstance();
        dslcd.println(DriverStationLCD.Line.kMain6, 1, "instantiating Bot2012");

        // Limit switches
        wedgeUpLimit = new DigitalInput(DIO_WEDGE_UP_LIMIT);
        wedgeDownLimit = new DigitalInput(DIO_WEDGE_DOWN_LIMIT);

        // CAN IDs  left: 2,4  right: 3,5
        driveTrain = new DriveTrain(JAG_LEFT_A, JAG_RIGHT_A, JAG_LEFT_B, JAG_RIGHT_B);

        // CAN IDs shooter wheels: 6,7  turret: 8
        mrShooter = new Shooter(JAG_SHOOTER_WHEELS_A, JAG_SHOOTER_WHEELS_B, JAG_TURRET);

        // Victors - PWM slots on Digital Sidecar
        mrWedge = new Wedge(VIC_WEDGE, wedgeUpLimit, wedgeDownLimit);
        mrLift = new Lift(VIC_LIFT);
        mrPickup = new BallPickup(VIC_BALLPICKUP);


        mrRelay = new Relay(1, 1);

        // UI devices
        driveStick1 = new Joystick(1);
        driveStick2 = new Joystick(2);
        try {
            gamepad = new Joystick(3);
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
        mrGyro = new SendableGyro(1);
        mrGyro.reset();

        spyCam = new Camera();


        //adds the drive options to the sendable chooser, with TankDrive as the default
        controlChooser = new SendableChooser();
        controlChooser.addDefault("Default Tank Drive", new DriveType(TANK_DRIVE));
        controlChooser.addObject("X-Y Tilt Drive", new DriveType(TILT_DRIVE));
        controlChooser.addObject("Twist Stick Drive", new DriveType(TWIST_DRIVE));
        SmartDashboard.putData("Drive Chooser", controlChooser);

        //adds the speed options to the sendable chooser, with 66% as the default
        autoSpeedChooser = new SendableChooser();
        autoSpeedChooser.addDefault("Default 66%", new AutoSpeed(PERCENT_66));
        autoSpeedChooser.addObject("100%", new AutoSpeed(PERCENT_100));
        autoSpeedChooser.addObject("33%", new AutoSpeed(PERCENT_33));
        autoSpeedChooser.addObject("0%", new AutoSpeed(PERCENT_0));
        SmartDashboard.putData("Autonomous Speed", autoSpeedChooser);

        //adds the option of camera alignment, with manual control as the default
        cameraAlign = new SendableChooser();
        cameraAlign.addDefault("Manual turret control", new TurretMode(true));
        cameraAlign.addObject("Camera Alignment", new TurretMode(false));
        SmartDashboard.putData("Turret Mode", cameraAlign);

        liftChooser = new SendableChooser();
        SmartDashboard.putData("Lift Chooser", liftChooser);

        double angle = roundTwoPlaces(mrGyro.getAngle());
        //SmartDashboard.putDouble("Gyro", angle);

        mrRelay.set(Relay.Value.kForward);
    }

    public void autonomousInit()
    {
        autoSpeed = (AutoSpeed) autoSpeedChooser.getSelected();
    }

    /**
     * This method is called periodically when the robot is in autonomous
     */
    public void autonomousPeriodic() {
        //implement the camera tracking code in here before you shoot

        //prepare for the shot (line up the drive train)

        // Start the shooter wheels at 66% power
        mrShooter.setWheelSpeed(autoSpeed.getSpeed());
//        mrShooter.wheelsOn();

        // delay for wheel spinup
        Timer.delay(3.0);
        mrLift.up();
    }

    public void teleopInit()
    {
        driveType = (DriveType) controlChooser.getSelected();
        turretMode = (TurretMode) cameraAlign.getSelected();

    }

    /**
     * This method is called periodically when the robot is in teleop
     */
    public void teleopPeriodic() {

        // Move the robot
        moveRobot();

        //SO WE CAN HAVE VALUES BEFORE MOVING BECAUAE LOGIC!
//        camera();

        // Set the shooter wheel speed and start the wheels
        setShooter();

        // Adjust Turret
        aimTurret();

        // Pickup or eject balls
        pickupBalls();

        // Lift/feed balls into shooter
        feedBalls();

        //add automatic code here on the side to line up the robot to shoot

        //put wedge code here.
        moveWedge();


        SmartDashboard.putData("Lift Chooser", liftChooser);
        SmartDashboard.putDouble("Gyro", roundTwoPlaces(mrGyro.getAngle()));
        updateDashboard();

//        if (counter >= 6) {
//            if (relayBool) {
//                mrRelay.set(Relay.Value.kReverse);
//            } else {
//                mrRelay.set(Relay.Value.kForward);
//            }
//            counter = 0;
//        }
//        counter++;

//        if (driveStick1.getRawButton(1)) {
//            mrRelay.set(Relay.Value.kForward);
//            dslcd.println(DriverStationLCD.Line.kMain6, 1, "Joystick Pressed.     ");
//        } else {
//            mrRelay.set(Relay.Value.kReverse);
//            dslcd.println(DriverStationLCD.Line.kMain6, 1, "Joystick NOT Pressed.");
//        }

        dslcd.updateLCD();

    }

    public void moveRobot() {
        if (driveType.getDriveType() == TANK_DRIVE) {
            left = driveStick1.getY();
            right = -driveStick2.getY();
        } else if (driveType.getDriveType() == TILT_DRIVE) {
            left = roundDirectionNum(driveStick1.getY() - driveStick1.getX()) * (driveStick1.getThrottle() + 1) / 2;
            right = roundDirectionNum(-driveStick1.getY() - driveStick1.getX());
        } else if (driveType.getDriveType() == TWIST_DRIVE) {
            left = driveStick1.getX() + driveStick1.getTwist();
            right = driveStick1.getX() - driveStick1.getTwist();
        }
        if(driveStick1.getRawButton(3) && driveStick2.getRawButton(3))
        {
            left *= 0.5;
            right *= 0.5;
        }
//        left*=((driveStick1.getThrottle()+1)/2);
//        right*= ((driveStick1.getThrottle()+1)/2);
        dslcd.println(DriverStationLCD.Line.kUser3, 1, "Left: " + left);
        dslcd.println(DriverStationLCD.Line.kUser4, 1, "Right: " + right);
        driveTrain.setSpeed(left, right);
    }

    public void setShooter() {
        // Readdress gamepad if single drive stick is being used
        if (driveType.getDriveType() != TANK_DRIVE) {
            gamepad = driveStick2;
        }
        if(isProjectAimbot)
        {
            if (gamepad.getRawButton(1)) {
                mrShooter.setWheelSpeed(PERCENT_0);
            } else if (gamepad.getRawButton(2)) {
                mrShooter.setWheelSpeed(PERCENT_33);
            } else if (gamepad.getRawButton(3)) {
                mrShooter.setWheelSpeed(PERCENT_66);
            } else if (gamepad.getRawButton(4)) {
                mrShooter.setWheelSpeed(PERCENT_100);
            }
        }
        SmartDashboard.putDouble("Actual Wheel Speed (RPM): ", mrShooter.getWheelSpeed());

        // debug code
//        mrShooter.setWheelSpeed(0.5);
//        System.out.println("Button 4 status: " + gamepad.getRawButton(4));
//        System.out.println("Percent status 66: " + PERCENT_66 + "Percent status 33: " + PERCENT_33);
        //^ debug code

//        mrShooter.wheelsOn();
    }

    /*
     *   Not used at this time...
     */
    public void setWheelMode() {
        // Readdress gamepad if single drive stick is being used
        if (driveType.getDriveType() != TANK_DRIVE) {
            gamepad = driveStick2;
        }

//        if(mrJoystick3.getAxisChannel(Joystick.AxisType.kNumAxis)) {
//
//        }
    }

    /**
     * Read digital (left) joystick on gamepad (X)
     */
    public void aimTurret() {
        // Readdress gamepad if single drive stick is being used
        if (driveType.getDriveType() != TANK_DRIVE) {
            gamepad = driveStick2;
        }

        if(turretMode.isManual())
        {
            double leftGamepadStick = gamepad.getAxis(Joystick.AxisType.kX);
            //System.out.println("LeftStrick Dir: " + leftGamepadStick);
            if (leftGamepadStick > 0.1)
            {
                mrShooter.turretClockwise();
            }
            else if(leftGamepadStick < -0.1)
            {
                mrShooter.turretCounterClockwise();
            }
            else
            {
                mrShooter.turretStop();
            }
        }
        else
        {
            double direction = spyCam.getMotorDirection(spyCam.getPreciseRecPos(0));
            if(direction > 0)
            {
                mrShooter.turretClockwise();
            }
            else if(direction < 0)
            {
                mrShooter.turretCounterClockwise();
            }
        }




    }

    public void pickupBalls() {
        // Readdress gamepad if single drive stick is being used
        if (driveType.getDriveType() != TANK_DRIVE) {
            gamepad = driveStick2;
        }

        if (gamepad.getRawButton(5))
        {
            mrPickup.setSpeed(-mrPickup.pickUpSpeed);
        } 
        else if (gamepad.getRawButton(7) && !gamepad.getRawButton(5))
        {
            mrPickup.setSpeed(mrPickup.pickUpSpeed);
        }
        else
        {
            mrPickup.setSpeed(0.0);
        }
    }

    public void feedBalls() {
//     if(LIFTMANUAL) {
            // Readdress gamepad if single drive stick is being used
            if (driveType.getDriveType() != TANK_DRIVE) {
                gamepad = driveStick2;
            }

            if (gamepad.getRawButton(6))
            {
                mrLift.up();
            } 
            else if (gamepad.getRawButton(8) && !gamepad.getRawButton(6))
            {
                mrLift.down();
            }
            else
            {
                mrLift.reset();
            }
//        } else {
//            if(driveStick1.getThrottle() > .5) {
//                mrRelay.set(Relay.Value.kForward);
//            } else {
//                mrRelay.set(Relay.Value.kReverse);
//            }

//        }
    }

    public void moveWedge() {
        // Readdress gamepad if single drive stick is being used
        if (driveType.getDriveType() != TANK_DRIVE) {
            gamepad = driveStick2;
        }

        if (gamepad.getRawButton(9) && !gamepad.getRawButton(10))
        {
            mrWedge.up();
        } 
        else if (gamepad.getRawButton(10) && !gamepad.getRawButton(9))
        {
            mrWedge.down();
        }
        else
        {
            mrWedge.reset();
        }
       // mrWedge.reader();
//        else
//        {
//            mrWedge.reader();
//        }
    }

    public void updateDashboard()
    {
        if (driveType.getDriveType() == TANK_DRIVE)
        {
            dslcd.println(DriverStationLCD.Line.kMain6, 1, "Output to left side: " + driveStick1.getY());
            dslcd.println(DriverStationLCD.Line.kUser2, 1, "Output to right side: " + driveStick2.getY());
        } 
        else if (driveType.getDriveType() == TILT_DRIVE)
        {
            dslcd.println(DriverStationLCD.Line.kMain6, 1,
                    "Output to left side: " + driveStick1.getX());
            dslcd.println(DriverStationLCD.Line.kUser2, 1,
                    "Output to right side: " + (driveStick1.getX()));
        }
        else if (driveType.getDriveType() == TWIST_DRIVE)
        {
            dslcd.println(DriverStationLCD.Line.kMain6, 1,
                    "Output to left side: " + driveStick1.getX() + driveStick1.getTwist());
            dslcd.println(DriverStationLCD.Line.kUser2, 1,
                    "Output to right side: " + (driveStick1.getX() - driveStick1.getTwist()));
        }
        SmartDashboard.putInt("dataTest", spyCam.pos);
        SmartDashboard.putInt("Distance", (int)spyCam.getDistance(spyCam.getPreciseRecPos(0)));
        SmartDashboard.putDouble("MotorDirection", spyCam.getMotorDirection(0));
        dslcd.updateLCD();
    }

    //@TODO: make dis work
    public void camera()
    {
        camera = AxisCamera.getInstance();
        camera.writeResolution(AxisCamera.ResolutionT.k320x240);
        camera.writeBrightness(50);
        if(camera.freshImage())
        {
            try
            {
                spyCam.calImage(camera.getImage(), 0);
            }
            catch (NIVisionException ex)
            {
                ex.printStackTrace();
            }
            catch(AxisCameraException  e)
            {
                e.printStackTrace();
            }
        }

    }

    /**
     * Used by SmartDashboard to record Drive Type selected by User
     */
    public class DriveType {

        public int driveType;

        public DriveType(int dType) {
            driveType = dType;
        }

        public int getDriveType() {
            return driveType;
        }
    }

    /**
     * Used by SmartDashboard to capture Autonomous Shooter Wheel Speed set by User
     */
    public class AutoSpeed
    {

        public double autoSpeed;

        public AutoSpeed(double spd)
        {
            autoSpeed = spd;
        }

        public double getSpeed()
        {
            return autoSpeed;
        }
    }
    
    /**
     * Used by SmartDashboard to set the mode of the turret
     * (Manual vs Camera Alignment)
     */
    public class TurretMode
    {
        private boolean isManual;
        public TurretMode(boolean b)
        {
            isManual = b;
        }
        public boolean isManual()
        {
            return isManual;
        }
    }

    public double roundDirectionNum(double num)
    {
        if (num > 1)
        {
            num = 1;

        }
        if (num < -1)
        {
            num = -1;
        }
        return num;
    }

    public double roundTwoPlaces(double dbl) {
        return (double) (int) ((dbl + 0.005) * 100.0) / 100.0;
    }
}
