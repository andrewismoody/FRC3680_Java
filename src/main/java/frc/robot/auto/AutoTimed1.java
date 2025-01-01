package frc.robot.auto;

import frc.robot.DriveModule;
import frc.robot.RobotModule;
import frc.robot.ModuleController;

public class AutoTimed1 implements Auto {
    long driveTime1 = 3000;
    long steerTime1 = 500;
    long shootTime = 2000;
    long driveTime2 = 3000;
    long pickupTime = 2000;
    long startTime = 0;
    long accumulatedTime = 0;

    ModuleController controller;
    DriveModule drive;
    RobotModule shooter;
    RobotModule intake;
    RobotModule feed;

    public AutoTimed1(ModuleController Controller, DriveModule Drive, RobotModule Shooter, RobotModule Intake, RobotModule Feed) {
        controller = Controller;
        drive = Drive;
        shooter = Shooter;
        intake = Intake;
        feed = Feed;
    }

    public void Initialize() {
        startTime = System.currentTimeMillis();
    }

    public void Update() {
        if (accumulatedTime < driveTime1) {
            drive.ProcessForwardSpeed(1.0);
        }

        if (accumulatedTime > driveTime1 && accumulatedTime < driveTime1 + steerTime1) {
            drive.ProcessRotationAngle(1.0);
        }

        if (accumulatedTime > driveTime1 + steerTime1 && accumulatedTime < driveTime1 + steerTime1 + shootTime) {
            shooter.ProcessState(true);
        } else {
            shooter.ProcessState(false);
        }

        if (accumulatedTime > driveTime1 + steerTime1 + shootTime && accumulatedTime < driveTime1 + steerTime1 + shootTime + driveTime2) {
            drive.ProcessForwardSpeed(1.0);
        }

        if (accumulatedTime > driveTime1 + steerTime1 + shootTime + driveTime2 && accumulatedTime < driveTime1 + steerTime1 + shootTime + driveTime2 + pickupTime) {
            intake.ProcessState(true);
            feed.ProcessState(true);
        } else {
            intake.ProcessState(false);
            feed.ProcessState(false);
        }

        if (accumulatedTime > driveTime1 + steerTime1 + shootTime + driveTime2 + pickupTime && accumulatedTime < driveTime1 + steerTime1 + shootTime + driveTime2 + pickupTime + shootTime) {
            shooter.ProcessState(true);
        } else {
            shooter.ProcessState(false);
        }

        controller.ProcessDrive();

        accumulatedTime = System.currentTimeMillis() - startTime;
    }

    public void Shutdown() {

    }
}
