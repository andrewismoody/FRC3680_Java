package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;

public class Controller {
    PS4Controller ps4Controller;
    XboxController xboxController;

    public enum ControllerType {
        Xbox,
        PS4
    }

    ControllerType Type;

    int Index;

    public Controller(int index, ControllerType type) {
        Index = index;
        Type = type;
        switch (Type) {
            case Xbox:
                xboxController = new XboxController(index);
            case PS4:
                ps4Controller = new PS4Controller(index);
        }
    }

    public boolean getSquareButton() {
        switch (Type) {
            case Xbox:
                return xboxController.getXButton();
            case PS4:
                return ps4Controller.getSquareButton();
        }

        return false;
    }

    public boolean getTriangleButton() {
        switch (Type) {
            case Xbox:
                return xboxController.getYButton();
            case PS4:
                return ps4Controller.getTriangleButton();
        }

        return false;
    }



    public boolean getCircleButton() {
        switch (Type) {
            case Xbox:
                return xboxController.getBButton();
            case PS4:
                return ps4Controller.getCircleButton();
        }

        return false;
    }
 
    public boolean getCrossButton() {
        switch (Type) {
            case Xbox:
                return xboxController.getAButton();
            case PS4:
                return ps4Controller.getCrossButton();
        }

        return false;
    }
 
    public boolean getL1Button() {
        switch (Type) {
            case Xbox:
                return xboxController.getLeftBumper();
            case PS4:
                return ps4Controller.getL1Button();
        }

        return false;
    }

    public double getL2Button() {
        switch (Type) {
            case Xbox:
                return xboxController.getLeftTriggerAxis();
            case PS4:
                return ps4Controller.getL2Button() ? 1.0 : 0.0;
        }

        return 0.0;
    }
 
    public boolean getR1Button() {
        switch (Type) {
            case Xbox:
                return xboxController.getRightBumper();
            case PS4:
                return ps4Controller.getR1Button();
        }

        return false;
    }

    public double getR2Button() {
        switch (Type) {
            case Xbox:
                return xboxController.getRightTriggerAxis();
            case PS4:
                return ps4Controller.getR2Button() ? 1.0 : 0.0;
        }

        return 0.0;
    }

    public double getRightY() {
        switch (Type) {
            case Xbox:
                return xboxController.getRightY();
            case PS4:
                return ps4Controller.getRightY();
        }

        return 0.0;
    }

    public double getRightX() {
        switch (Type) {
            case Xbox:
                return xboxController.getRightX();
            case PS4:
                return ps4Controller.getRightX();
        }

        return 0.0;
    }

    public double getLeftY() {
        switch (Type) {
            case Xbox:
                return xboxController.getLeftY();
            case PS4:
                return ps4Controller.getLeftY();
        }

        return 0.0;
    }

    public double getLeftX() {
        switch (Type) {
            case Xbox:
                return xboxController.getLeftX();
            case PS4:
                return ps4Controller.getLeftX();
        }

        return 0.0;
    }
}
