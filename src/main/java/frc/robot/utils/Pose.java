package frc.robot.utils;

// Import the needed libraries
import edu.wpi.first.math.geometry.Rotation2d;

public class Pose {
    
    // Private variables
    double x; // X value in inches
    double y; // Y value in inches
    Rotation2d angle; // angle in radians

    /**
     * Create a new pose
     * @param _x x comp
     * @param _y y comp
     * @param _angle angle of the pose
     */

     public Pose(double _x, double _y, Rotation2d _angle) {
        x = _x;
        y = _y;
        angle = _angle;
     }

    /**
     * Create a new pose at 90 deg
     */

     public Pose() {
        x = 0;
        y = 0;
        angle = Rotation2d.fromDegrees(90);
     }

    /**
     * Get the x comp in inches
     * @return x comp
     */

     public double GetXValue() {
        return x;
     }

    /**
     * Get the y comp in inches
     * @return y comp
     */

     public double GetYValue() {
        return y;
     }

    /**
     * Get the angle in degrees
     * @return angle in degrees
     */

     public double GetAngleValue() {
        return angle.getDegrees();
     }

    /**
     * Set the x componenet in inches
     * @param _x x comp
     */

     public void SetX(double _x) {
        x = _x;
     }

    /**
     * Set the y componenet in inches
     * @param _y y comp
     */

     public void SetY(double _y) {
        y = _y;
     }

    /**
     * Set the angle in degrees
     * @param _angle new angle
     */

     public void SetAngle(Rotation2d _angle) {
        //angle = new Rotation2d(_angle.getDegrees());
        angle = _angle;
     }

    /**
     * Reflect the pose along the y axis
     */

     public void ReflectY() {
        x = -x;
        angle = new Rotation2d(Math.atan2(Math.sin(angle.getDegrees()), -Math.cos(angle.getDegrees())));
     }

    /**
     * Add vector to the pose
     * @param v the vector to add
     */

     public void AddVector(Vector v) {
        x += v.GetXValue();
        y += v.GetYValue();
     }

    /**
     * Subtract vector from the pose
     * @param v the vector to subtract
     */

     public void SubtractVector(Vector v) {
        x -= v.GetXValue();
        y -= v.GetYValue();
     }

    /**
     * Subtract a pose from another pose
     * @param p The pose to subtract
     * @return the resultant vector
     */

     public Vector Subtract(Pose p) {
        return new Vector(x - p.GetXValue(), y - p.GetYValue());
     }

    /**
     * Sets the x y and angle comps
     * @param _x new x comp
     * @param _y new y comp
     * @param _angle new angle comp
     */

     public void SetPose(double _x, double _y, Rotation2d _angle) {
        x = _x;
        y = _y;
        angle = _angle;
     }

    /**
     * Sets the x y and angle componenets
     * @param _pose new pose
     */

     public void SetPose(Pose _pose) {
        x = _pose.GetXValue();
        y = _pose.GetYValue();
        angle = new Rotation2d( _pose.GetAngleValue());
     }

    /**
     * Change the frame of reference from robot to field
     * @param GyroAngle the angle of the gyro as a double, but in degrees
     */

     public void Transform(double GyroAngle) {
        angle = angle.plus(Rotation2d.fromDegrees(GyroAngle));
        angle = Calculations.NormalizeAngle(angle);
     }

         /**
     * Print the pose with a prefix string
     * @param pre String to print before the pose
     */
    public void Print(String pre) {
        System.out.printf("%s: x:%.2f y:%.2f a:%.2f\n", pre, x, y, angle.getDegrees());
    }

    /**
     * Print the pose with a prefix and a number
     * @param pre String to print before the pose
     * @param i   Number to print before the pose
     */
    public void Print(String pre, int i) {
        System.out.printf("%s %d: x:%.2f y:%.2f a:%.2f\n", pre, i, x, y, angle.getDegrees());
    }

    /**
     * Print the pose with a number only
     * @param i Number to print before the pose
     */
    public void Print(int i) {
        System.out.printf("%d: x:%.2f y:%.2f a:%.2f\n", i, x, y, angle.getDegrees());
    }

    /**
     * Print the pose without any prefix
     */
    public void Print() {
        System.out.printf("x:%.2f y:%.2f a:%.2f\n", x, y, angle.getDegrees());
    }
     
}
