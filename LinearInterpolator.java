
/**
 * The LinearInterpolator class provides linear interpolation between given data points.
 */

class LinearInterpolator {
    private final double[] x;
    private final double[] y;

    /**
     * Constructor to initialize the LinearInterpolator with known data points.
     *
     * @param x An array of known x values (independent variable, e.g., altitude )
     * @param y An array of known y values (dependent variable, e.g., vertical speed or angle)
     */
    public LinearInterpolator(double[] x, double[] y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Performs linear interpolation to estimate the y value for a given x value.
     * If the x value is within the range of known x values, it interpolates between the nearest two points.
     * If the x value is outside the range, it returns the closest y value.
     *
     * @param value The x value for which the corresponding y value is to be estimated.
     * @return The interpolated y value.
     */
    public double interpolate(double value) {

        // Edge Cases
        if (value <= x[0]) return y[0];
        if (value >= x[x.length - 1]) return y[y.length - 1];

        for (int i = 0; i < x.length - 1; i++) {
            if (value >= x[i] && value <= x[i + 1]) {
                // Calculate the interpolation factor 't' (fractional position between x[i] and x[i+1])
                double t = (value - x[i]) / (x[i + 1] - x[i]);
                // Perform linear interpolation using the formula: y = y[i] + t * (y[i+1] - y[i])
                return y[i] + t * (y[i + 1] - y[i]);
            }
        }
        // Fallback case
        return y[0];
    }
}
