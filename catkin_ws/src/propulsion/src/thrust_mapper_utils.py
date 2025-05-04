import numpy as np
#Thrusters, like the Blue Robotics T200, don’t have a perfectly linear response near zero force.
#Linear transition region around zero force (Newtons)
#Creates a smooth transition around zero force to avoid abrupt changes in PWM.

thruster_mount_dirs = [ #rep physical orientation of thrusters
    1,   # BACK_LEFT (CCW)
    1,  # HEAVE_BACK_LEFT (CW)
    1,   # HEAVE_FRONT_LEFT (CCW)
    1,  # FRONT_LEFT (CW)
    1,   # FRONT_RIGHT (CCW)
    1,  # HEAVE_FRONT_RIGHT (CW)
    1,   # HEAVE_BACK_RIGHT (CCW)
    1   # BACK_RIGHT (CW)
]
#1 is forward, -1 is backward, 0.5 is half thrust in specific direction
#If force is exactly zero, returns 1500 μs (neutral signal, no movement).

def force_to_pwm_thruster1(force):
    '''Converts a desired thrust force (in Newtons) into a PWM signal (in microseconds)
    for Thruster 1 using a two-sided 5th-order polynomial fit and linear smoothing 
    in the deadband region. Coefficients of the polynomials for all 8 thrusters were made
    to fit the the Thruster Test Data of May 2025. 

    - For forces above +0.5 N, a polynomial fit for positive forces is used.
    - For forces below -0.5 N, a polynomial fit for negative forces is used.
    - For small forces within ±0.5 N, the function linearly interpolates 
      between 1500 µs and the polynomial curves to ensure smooth transitions 
      and prevent jitter near zero thrust.
    
    The output PWM is clipped to remain within [1100, 1900] µs, which matches 
    our ESC input limits.'''
    
    deadband_eps = 0.5
    coeffs_left = [1.0230567551000761e-05, 0.00102996961393584, 0.040389999005431505, 0.7860223131995762, 16.77703700452621, 1458.4801287000814] 
    coeffs_right = [3.430757828456944e-06, -0.0004931410796589675, 0.026547762979844677, -0.6681700673886003, 15.361505309013667, 1532.2050959044316]  

    if abs(force) <= deadband_eps:
        pwm_pos_eps = np.polyval(coeffs_right, deadband_eps)
        pwm_neg_eps = np.polyval(coeffs_left, -deadband_eps)
        if force >= 0:
            pwm = 1500 + (force / deadband_eps) * (pwm_pos_eps - 1500)
        else:
            pwm = 1500 + (force / deadband_eps) * (1500 - pwm_neg_eps)
    elif force < -deadband_eps:
        pwm = np.polyval(coeffs_left, force)
    else:
        pwm = np.polyval(coeffs_right, force)

    return int(np.clip(pwm, 1100, 1900))

def force_to_pwm_thruster2(force):
    deadband_eps = 0.5
    coeffs_left = [1.9792619863856807e-05, 0.00430568657152564, 0.16552600527321884, 2.005302506063938, 21.44799478063138, 1469.2876938929282]
    coeffs_right = [2.7590747726334654e-06, -0.0004267788232113865, 0.023788005495560777, -0.6053581191075251, 15.257178074416181, 1537.8143209940818]

    if abs(force) <= deadband_eps:
        pwm_pos_eps = np.polyval(coeffs_right, deadband_eps)
        pwm_neg_eps = np.polyval(coeffs_left, -deadband_eps)
        if force >= 0:
            pwm = 1500 + (force / deadband_eps) * (pwm_pos_eps - 1500)
        else:
            pwm = 1500 + (force / deadband_eps) * (1500 - pwm_neg_eps)
    elif force < -deadband_eps:
        pwm = np.polyval(coeffs_left, force)
    else:
        pwm = np.polyval(coeffs_right, force)

    return int(np.clip(pwm, 1100, 1900))


def force_to_pwm_thruster3(force):
    deadband_eps = 0.5
    coeffs_left = [0.00015828221751606117, 0.01158034626673705, 0.29139761923839835, 2.9414234100262417, 23.829243182617976, 1462.4806630341138] 
    coeffs_right =  [1.7707706911964165e-06, -0.00027076966193783096, 0.015275019271750026, -0.423532293350616, 13.60742808725115, 1537.3243900034613] 

    if abs(force) <= deadband_eps:
        pwm_pos_eps = np.polyval(coeffs_right, deadband_eps)
        pwm_neg_eps = np.polyval(coeffs_left, -deadband_eps)
        if force >= 0:
            pwm = 1500 + (force / deadband_eps) * (pwm_pos_eps - 1500)
        else:
            pwm = 1500 + (force / deadband_eps) * (1500 - pwm_neg_eps)
    elif force < -deadband_eps:
        pwm = np.polyval(coeffs_left, force)
    else:
        pwm = np.polyval(coeffs_right, force)

    return int(np.clip(pwm, 1100, 1900))


def force_to_pwm_thruster4(force):
    deadband_eps = 0.5
    coeffs_left = [-1.1552965779022153e-05, 0.00038064670238558356, 0.06404572005694033, 1.6789593308233672, 26.85675098723853, 1489.744988765967] 
    coeffs_right = [4.28729616014896e-06, -0.0005682519968556897, 0.02886925945271591, -0.7292412806746723, 17.22831131585594, 1532.5325493925116]

    if abs(force) <= deadband_eps:
        pwm_pos_eps = np.polyval(coeffs_right, deadband_eps)
        pwm_neg_eps = np.polyval(coeffs_left, -deadband_eps)
        if force >= 0:
            pwm = 1500 + (force / deadband_eps) * (pwm_pos_eps - 1500)
        else:
            pwm = 1500 + (force / deadband_eps) * (1500 - pwm_neg_eps)
    elif force < -deadband_eps:
        pwm = np.polyval(coeffs_left, force)
    else:
        pwm = np.polyval(coeffs_right, force)

    return int(np.clip(pwm, 1100, 1900))


def force_to_pwm_thruster5(force):
    deadband_eps = 0.5
    coeffs_left = [-6.941580671470798e-06, -0.00058557986492076, -0.013961189254281707, -0.0002720187161500033, 12.236038684182764, 1448.4552329804822]
    coeffs_right = [8.932847739553046e-07, -0.00019474823917039024, 0.013292476455512672, -0.39827137223089093, 13.002323887488586, 1535.342515302347]

    if abs(force) <= deadband_eps:
        pwm_pos_eps = np.polyval(coeffs_right, deadband_eps)
        pwm_neg_eps = np.polyval(coeffs_left, -deadband_eps)
        if force >= 0:
            pwm = 1500 + (force / deadband_eps) * (pwm_pos_eps - 1500)
        else:
            pwm = 1500 + (force / deadband_eps) * (1500 - pwm_neg_eps)
    elif force < -deadband_eps:
        pwm = np.polyval(coeffs_left, force)
    else:
        pwm = np.polyval(coeffs_right, force)

    return int(np.clip(pwm, 1100, 1900))



def force_to_pwm_thruster6(force):
    deadband_eps = 0.5
    coeffs_left = [1.5816032862031332e-05, 0.0012422185608465996, 0.038226378488502424, 0.6168147468496009, 15.760260299367387, 1456.195377454675] 
    coeffs_right = [2.7107832175712445e-06, -0.00037330147281785505, 0.019542179140516265, -0.4943442917222498, 14.201154149884227, 1534.225644742625]

    if abs(force) <= deadband_eps:
        pwm_pos_eps = np.polyval(coeffs_right, deadband_eps)
        pwm_neg_eps = np.polyval(coeffs_left, -deadband_eps)
        if force >= 0:
            pwm = 1500 + (force / deadband_eps) * (pwm_pos_eps - 1500)
        else:
            pwm = 1500 + (force / deadband_eps) * (1500 - pwm_neg_eps)
    elif force < -deadband_eps:
        pwm = np.polyval(coeffs_left, force)
    else:
        pwm = np.polyval(coeffs_right, force)

    return int(np.clip(pwm, 1100, 1900))


def force_to_pwm_thruster7(force):
    deadband_eps = 0.5
    coeffs_left = [4.7698603291927485e-06, 0.0005523684514175677, 0.025124707629480242, 0.5688673391241492, 16.13382387002397, 1464.9715537145275]
    coeffs_right = [-9.267882571985216e-07, 4.954572425873791e-05, 0.0016201687947042655, -0.15277570672157192, 10.893237417742466, 1544.2819910395674]

    if abs(force) <= deadband_eps:
        pwm_pos_eps = np.polyval(coeffs_right, deadband_eps)
        pwm_neg_eps = np.polyval(coeffs_left, -deadband_eps)
        if force >= 0:
            pwm = 1500 + (force / deadband_eps) * (pwm_pos_eps - 1500)
        else:
            pwm = 1500 + (force / deadband_eps) * (1500 - pwm_neg_eps)
    elif force < -deadband_eps:
        pwm = np.polyval(coeffs_left, force)
    else:
        pwm = np.polyval(coeffs_right, force)

    return int(np.clip(pwm, 1100, 1900))

def force_to_pwm_thruster8(force):
    deadband_eps = 0.5
    coeffs_left = [1.969990453701789e-05, 0.0018732855138689026, 0.0669614466686662, 1.1258357561487122, 18.729839818258526, 1465.5025109239948]
    coeffs_right = [4.747498741718638e-06, -0.0005519151129485432, 0.0243819736986421, -0.5313334295643649, 13.750987401737758, 1533.5188424837772]

    if abs(force) <= deadband_eps:
        pwm_pos_eps = np.polyval(coeffs_right, deadband_eps)
        pwm_neg_eps = np.polyval(coeffs_left, -deadband_eps)
        if force >= 0:
            pwm = 1500 + (force / deadband_eps) * (pwm_pos_eps - 1500)
        else:
            pwm = 1500 + (force / deadband_eps) * (1500 - pwm_neg_eps)
    elif force < -deadband_eps:
        pwm = np.polyval(coeffs_left, force)
    else:
        pwm = np.polyval(coeffs_right, force)

    return int(np.clip(pwm, 1100, 1900))