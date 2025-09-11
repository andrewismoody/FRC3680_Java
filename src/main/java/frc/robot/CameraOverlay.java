package frc.robot;

import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

public class CameraOverlay {
    public static void main(String[] args) {
        // Load OpenCV native library
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        // Open the USB camera feed
        VideoCapture camera = new VideoCapture(0); // 0 is the default camera
        if (!camera.isOpened()) {
            System.out.println("Error: Could not open camera");
            return;
        }

        // Load the overlay image
        Mat overlayImage = Imgcodecs.imread("Doom1.png", Imgcodecs.IMREAD_UNCHANGED);
        if (overlayImage.empty()) {
            System.out.println("Error: Could not load overlay image");
            return;
        }
        
        Mat frame = new Mat();

        while (true) {
            // Capture a frame from the camera
            if (!camera.read(frame)) {
                System.out.println("Error: Could not read frame");
                break;
            }

            // Resize the overlay to match the frame size (optional)
            Mat resizedOverlay = new Mat();
            Imgproc.resize(overlayImage, resizedOverlay, frame.size());

            // Exit condition to break the loop
        }
    }
}