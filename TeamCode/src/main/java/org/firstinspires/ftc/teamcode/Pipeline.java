package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline {
    Mat mat = new Mat();
    // Color Evaluation
    public int lowX = 200;
    public int lowY = 120;
    public int lowZ = 120;
    public int highX = 255;
    public int highY = 255;
    public int highZ = 255;
    
    private int LeftROIStartRow = 180;
    private int LeftROIEndRow = 320;
    private int MiddleROIStartRow = 20;
    private int MiddleROIEndRow = 150;
    private int ROIStartCol = 60; // Expanded height in case camera tilts
    private int ROIEndCol = 180; // Expanded height in case camera tilts
    private int resultROI;

    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_BGR2RGBA);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGBA2BGR);

        // Creates mask to identify specific color
        Scalar lowColorValue = new Scalar(lowX, lowY, lowZ);
        Scalar highColorValue = new Scalar(highX, highY, highZ);
        // Applies mask.  Most colors become black, some become white.
        Core.inRange(mat, lowColorValue, highColorValue, mat);

        // Create the areas we are interested in looking at.
        Mat LeftROI = mat.submat(LeftROIStartRow, LeftROIEndRow, ROIStartCol, ROIEndCol);
        Mat MiddleROI = mat.submat(MiddleROIStartRow, MiddleROIEndRow, ROIStartCol, ROIEndCol);

        setResultROI(evaluateROIs(LeftROI, MiddleROI));

        LeftROI.release();      // Added by Ohm Raiders to prevent memory leak
        MiddleROI.release();    // Added by Ohm Raiders

        // Adds the rectangles so we can see where we are looking (the ROIs)
        Imgproc.rectangle(mat, new Point(ROIStartCol, LeftROIStartRow), new Point(ROIEndCol, LeftROIEndRow), new Scalar(128,128,128), 2);
        Imgproc.rectangle(mat, new Point(ROIStartCol, MiddleROIStartRow), new Point(ROIEndCol, MiddleROIEndRow), new Scalar(128,128,128), 2);
        return mat;
    }

    private int evaluateROIs(Mat LeftROI, Mat MiddleROI) {
        // Returns results:
        // 0 - Left 
        // 1 - Middle
        // 2 - Right
        int LResult = findWhiteCount(LeftROI);
        int MResult = findWhiteCount(MiddleROI);
        
        if (LResult - MResult > 50) {
            return 0;
        } else if (MResult - LResult > 50) {
            return 1;
        } else {
            return 2;
        }
    }

    private int findWhiteCount(Mat roi) {
        int count = 0;
        for (int row = 0; row < roi.height(); row++) {
            for (int col = 0; col < roi.width(); col++) {
                if (roi.get(row, col)[0] > 1) {
                    count += 1;
                }
            }
        }
        return count;
    }

    public int getResultROI()
    {
        return resultROI;
    }

    private void setResultROI(int roi)
    {
        resultROI = roi;
    }
}
