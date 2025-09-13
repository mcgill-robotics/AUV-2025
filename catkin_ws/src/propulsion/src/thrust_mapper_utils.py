import numpy as np

# Thrusters like the Blue Robotics T200 do not have a perfectly linear thrust response near 0 N.
# To avoid abrupt changes and motor jitter near zero thrust, a linear deadband smoothing is applied
# in the small force region (|force| < deadband_eps). Outside this region, a 5th-order polynomial 
# fitted to experimental data is used to convert thrust (N) to PWM (μs).

# Deadband region around 0 N where smoothing is applied
deadband_eps = 0.5

# Polynomial coefficients for each thruster.
# Each tuple contains (coeffs_left, coeffs_right), where:
# - coeffs_left corresponds to negative thrust (force < -deadband_eps)
# - coeffs_right corresponds to positive thrust (force > +deadband_eps)
# The polynomials were fit using least-squares to May 2025 thruster test data.
thruster_poly_coeffs = [
    (
        [1.0230567551000761e-05, 0.00102996961393584, 0.040389999005431505, 0.7860223131995762, 16.77703700452621, 1458.4801287000814],
        [3.430757828456944e-06, -0.0004931410796589675, 0.026547762979844677, -0.6681700673886003, 15.361505309013667, 1532.2050959044316]
    ),
    (
        [1.9792619863856807e-05, 0.00430568657152564, 0.16552600527321884, 2.005302506063938, 21.44799478063138, 1469.2876938929282],
        [2.7590747726334654e-06, -0.0004267788232113865, 0.023788005495560777, -0.6053581191075251, 15.257178074416181, 1537.8143209940818]
    ),
    (
        [0.00015828221751606117, 0.01158034626673705, 0.29139761923839835, 2.9414234100262417, 23.829243182617976, 1462.4806630341138],
        [1.7707706911964165e-06, -0.00027076966193783096, 0.015275019271750026, -0.423532293350616, 13.60742808725115, 1537.3243900034613]
    ),
    (
        [-1.1552965779022153e-05, 0.00038064670238558356, 0.06404572005694033, 1.6789593308233672, 26.85675098723853, 1489.744988765967],
        [4.28729616014896e-06, -0.0005682519968556897, 0.02886925945271591, -0.7292412806746723, 17.22831131585594, 1532.5325493925116]
    ),
    (
        [-6.941580671470798e-06, -0.00058557986492076, -0.013961189254281707, -0.0002720187161500033, 12.236038684182764, 1448.4552329804822],
        [8.932847739553046e-07, -0.00019474823917039024, 0.013292476455512672, -0.39827137223089093, 13.002323887488586, 1535.342515302347]
    ),
    (
        [1.5816032862031332e-05, 0.0012422185608465996, 0.038226378488502424, 0.6168147468496009, 15.760260299367387, 1456.195377454675],
        [2.7107832175712445e-06, -0.00037330147281785505, 0.019542179140516265, -0.4943442917222498, 14.201154149884227, 1534.225644742625]
    ),
    (
        [4.7698603291927485e-06, 0.0005523684514175677, 0.025124707629480242, 0.5688673391241492, 16.13382387002397, 1464.9715537145275],
        [-9.267882571985216e-07, 4.954572425873791e-05, 0.0016201687947042655, -0.15277570672157192, 10.893237417742466, 1544.2819910395674]
    ),
    (
        [1.969990453701789e-05, 0.0018732855138689026, 0.0669614466686662, 1.1258357561487122, 18.729839818258526, 1465.5025109239948],
        [4.747498741718638e-06, -0.0005519151129485432, 0.0243819736986421, -0.5313334295643649, 13.750987401737758, 1533.5188424837772]
    ),
]


def force_to_pwm(force, coeffs_left, coeffs_right):
    """
    Converts a desired thrust force (in Newtons) into a PWM signal (in microseconds) using:
    - A two-sided 5th-order polynomial for forces |force| > deadband_eps
    - Linear interpolation for forces within the deadband region (|force| ≤ deadband_eps)

    Parameters:
        force         : float, desired thrust force in Newtons
        coeffs_left   : list of float, polynomial coefficients for negative forces
        coeffs_right  : list of float, polynomial coefficients for positive forces

    Returns:
        int: PWM signal in microseconds, clipped to [1100, 1900] µs (ESC input range)
    """
    if abs(1* force) <= deadband_eps:
        # Linear interpolation within deadband to ensure smooth transition
        pwm_pos_eps = np.polyval(coeffs_right, deadband_eps)
        pwm_neg_eps = np.polyval(coeffs_left, -deadband_eps)
        if 1 * force >= 0:
            pwm = 1500 + (1 * force / deadband_eps) * (pwm_pos_eps - 1500)
        else:
            pwm = 1500 + (1 * force / deadband_eps) * (1500 - pwm_neg_eps)
    elif 1 * force < -deadband_eps:
        pwm = np.polyval(coeffs_left, 1 * force)
    else:
        pwm = np.polyval(coeffs_right, 1 * force)

    return int(np.clip(pwm, 1100, 1900))

def force_to_pwm_thruster(thruster_index, force):
    """
    Dispatches force-to-PWM conversion for a specific thruster based on its index.

    Parameters:
        thruster_index : int, index from 1 to 8 representing the 8 thrusters
        force          : float, desired thrust force in Newtons

    Returns:
        int: PWM signal for that specific thruster
    """
    coeffs_left, coeffs_right = thruster_poly_coeffs[thruster_index - 1]
    return force_to_pwm(force, coeffs_left, coeffs_right)