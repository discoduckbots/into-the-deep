package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import org.firstinspires.ftc.teamcode.opmode.FancyCancelableTeleop;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;

import java.util.ArrayList;
import java.util.List;

public class OpencvBlockDetector {


    static int width = 0;
    static int height = 0;
    static double XOFFSET = 1.0;
    static double YOFFSET = 0.6;
    static int ERROR_THRESHOLD = 100;

    public static void detectBlock(Mat image, FancyCancelableTeleop.MotorOffset motorOffset) {

        width = image.cols();
        height = image.rows();


        // Show the original image and wait for click
//            showImageAndWait(image, "Original Image - " + file.getName() + " - Click to proceed");

        // 5. Convert to HSV
        Mat hsv = new Mat();
        Imgproc.cvtColor(image, hsv, Imgproc.COLOR_BGR2HSV);

        // 6. Create masks for red, blue, and yellow
        // --- Red Mask (using two ranges for red) ---
        Mat lowerRedMask = new Mat();
        Mat upperRedMask = new Mat();
        Core.inRange(hsv, new Scalar(0, 100, 100), new Scalar(10, 255, 255), lowerRedMask);
        Core.inRange(hsv, new Scalar(160, 100, 100), new Scalar(179, 255, 255), upperRedMask);
        Mat redMask = new Mat();
        Core.add(lowerRedMask, upperRedMask, redMask);

        // --- Blue Mask (broad range) ---
        Mat blueMask = new Mat();
        Core.inRange(hsv, new Scalar(80, 50, 50), new Scalar(140, 255, 255), blueMask);

        // --- Yellow Mask ---
        Mat yellowMask = new Mat();
        Core.inRange(hsv, new Scalar(20, 100, 100), new Scalar(30, 255, 255), yellowMask);

        // 7. Morphological operations to clean up noise
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));
        Imgproc.erode(redMask, redMask, kernel);
        Imgproc.dilate(redMask, redMask, kernel);

        Imgproc.erode(blueMask, blueMask, kernel);
        Imgproc.dilate(blueMask, blueMask, kernel);

        Imgproc.erode(yellowMask, yellowMask, kernel);
        Imgproc.dilate(yellowMask, yellowMask, kernel);

        // (Optional) Show the merged masks
        Mat mergedMasks = new Mat();
        List<Mat> maskChannels = new ArrayList<>();
        // Merging: (yellow->channel0, blue->channel1, red->channel2)
        maskChannels.add(yellowMask);
        maskChannels.add(blueMask);
        maskChannels.add(redMask);
        Core.merge(maskChannels, mergedMasks);

        //showImageAndWait(mergedMasks, "Merged Masks - " + file.getName() + " - Click to proceed");

        // 8. For each mask, find contours and draw bounding boxes on a clone of the original
        //Mat output = image.clone();
        Mat output = mergedMasks;

        drawColorBoundingBoxes(redMask, output, new Scalar(0, 0, 255), "Red", motorOffset);
        drawColorBoundingBoxes(blueMask, output, new Scalar(255, 0, 0), "Blue", motorOffset);
        drawColorBoundingBoxes(yellowMask, output, new Scalar(0, 255, 255), "Yellow", motorOffset);

       // drawColorBoundingLines(redMask, output, new Scalar(0, 0, 255), "Red");
        //drawColorBoundingLines(blueMask, output, new Scalar(255, 0, 0), "Blue");
        //drawColorBoundingLines(yellowMask, output, new Scalar(0, 255, 255), "Yellow");

    }

    /**
     * Draw bounding boxes for the given mask (single-channel binary image).
     */
    private static void drawColorBoundingBoxes(Mat mask, Mat output, Scalar color, String label, FancyCancelableTeleop.MotorOffset motorOffset) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        // Find contours
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        double maxArea = 0;
        Rect maxRect = null;
        Log.d("DET", "conours :" + contours.size());
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            // Example: skip small areas, or you can tweak these thresholds
            if (area < 1000) continue;
            // if (area > 50000) continue;

            // Compute bounding rectangle
            Rect rect = Imgproc.boundingRect(contour);

            Log.d("DET","Contour rect " + rect);
            Imgproc.rectangle(output, rect, color, 2);

            // Optionally label
            String text = label + " - Area: " + (int) area;
            Imgproc.putText(output, text, new Point(rect.x, rect.y - 5),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color, 2);

            if (area > maxArea) {
                maxArea = area;
                maxRect = rect;
            }
        }
        double xoffset = 0;
        double yoffset = 0;

        if (maxRect != null) {

            double topOffset = maxRect.y;
            double bottomOffset = height - (maxRect.y + maxRect.height);
            double leftOffset = maxRect.x;
            double rightOffset = width - (maxRect.x + maxRect.width);
            if (Math.abs(leftOffset - rightOffset) < ERROR_THRESHOLD) {
                xoffset = 0;
            } else if (leftOffset < rightOffset) {
                xoffset = XOFFSET;
            } else {
                xoffset = -XOFFSET;
            }
            if (Math.abs(topOffset - bottomOffset) < ERROR_THRESHOLD) {
                yoffset = 0;
            } else if (topOffset < bottomOffset) {
                yoffset = +YOFFSET;
            } else {
                yoffset = -YOFFSET;
            }
            if (xoffset == 0 && yoffset == 0) {
                motorOffset.detected = true;
            }
            motorOffset.x_offset = xoffset;
            motorOffset.y_offset = yoffset;

            Log.d("DET", "x offset: " + xoffset + " y offset: " + yoffset + "detected " + motorOffset.detected);
        }

    }

    private static void drawColorBoundingLines(Mat mask, Mat output, Scalar color, String label) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        // Find contours
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Process each detected block (contour)
        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);
            Point[] boxPoints = new Point[4];
            rotatedRect.points(boxPoints);

            double width = rotatedRect.size.width;
            double height = rotatedRect.size.height;

            // Skip boxes outside desired size
            if (width < 100 || height < 100) continue;
            // if (width > 650 || height > 650) continue;

            // Draw the bounding box on the original image
            for (int j = 0; j < 4; j++) {
                Imgproc.line(output, boxPoints[j], boxPoints[(j + 1) % 4], new Scalar(0, 255, 0), 2);
            }

            // Add text with dimensions
            String dimensionsText = "Width: " + String.format("%.2f", width) +
                    " | Height: " + String.format("%.2f", height);
            Point textPosition = new Point(boxPoints[0].x + 10, boxPoints[0].y + 10);
            Imgproc.putText(output, dimensionsText, textPosition, Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.5, new Scalar(0, 255, 0), 2);

            // Correct the angle calculation
            double angle = rotatedRect.angle;
            if (width < height) {
                angle += 90;
            }
            angle = angle < 0 ? angle + 90 : angle;

            String angleText = "Angle: " + String.format("%.2f", angle);
            Point anglePosition = new Point(boxPoints[0].x + 10, boxPoints[0].y + 30);
            Imgproc.putText(output, angleText, anglePosition, Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.5, new Scalar(0, 255, 0), 2);

            System.out.println("Block Orientation Angle: " + angle);
            System.out.println("Block Width: " + width);
            System.out.println("Block Height: " + height);

            double aspectRatio = rotatedRect.size.width / rotatedRect.size.height;
            System.out.println("Block is " + (aspectRatio > 1 ? "horizontal." : "vertical."));
        }

    }

}
