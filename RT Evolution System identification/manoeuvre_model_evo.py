import numpy as np
from ship_class import ship

def thrust_cone(x_eng, y_eng, az_eng, cone_deg, flow_distance, x_down, y_down):
    #check axis!!!!!!! coherent to choosen axis system
    top_corner = [y_eng, x_eng]
    right_corner = [y_eng + np.sin(np.deg2rad(az_eng - cone_deg/2.))*flow_distance, x_eng + np.cos(np.deg2rad(az_eng - cone_deg/2.))*flow_distance]
    left_corner = [y_eng + np.sin(np.deg2rad(az_eng + cone_deg/2.))*flow_distance, x_eng + np.cos(np.deg2rad(az_eng + cone_deg/2.))*flow_distance]
    triangle_list = [top_corner, right_corner, left_corner]
    x1, y1, x2, y2, x3, y3, xp, yp = top_corner[0], top_corner[1], right_corner[0], right_corner[1], left_corner[0], left_corner[1], y_down, x_down
    c1 = (x2 - x1) * (yp - y1) - (y2 - y1) * (xp - x1)
    c2 = (x3 - x2) * (yp - y2) - (y3 - y2) * (xp - x2)
    c3 = (x1 - x3) * (yp - y3) - (y1 - y3) * (xp - x3)
    if (c1 < 0 and c2 < 0 and c3 < 0) or (c1 > 0 and c2 > 0 and c3 > 0):
    # if max([pos[0] for pos in triangle_list])>=y_down>=min([pos[0] for pos in triangle_list]) and max([pos[1] for pos in triangle_list])>=x_down>=min([pos[1] for pos in triangle_list]):
        return (1 - np.deg2rad(abs((azimuth([y_eng, x_eng],[y_down,x_down])-az_eng))))
    else:
        return 0

def azimuth(point1, point2):
    angle = np.arctan2(point2[0] - point1[0], point2[1] - point1[1])
    return np.degrees(angle) if angle >= 0 else np.degrees(angle) + 360
    #add thrust fraction here

def thruster_interaction_coefficient(x_eng, y_eng, az_eng, cone_deg, flow_distance, x_down, y_down, az_down):  # give engine number: shipclass engine number
    thrust_cone_boolean = thrust_cone(x_eng, y_eng, az_eng, cone_deg, flow_distance, x_down, y_down)
    x_d_ratio = np.sqrt((x_down-x_eng)**2+abs(y_down-y_eng)) / ship.D_p
    t_engine = (1 - 0.75**(x_d_ratio**(2/3)))/(thrust_cone_boolean)
    # print(abs(az_eng-az_down), 1)
    t = t_engine+(1-t_engine)*(((abs(az_eng-az_down))**3)/(((130.)/(t_engine**3)) + ((abs(az_eng-az_down))**3)))
    if math.isnan(t):
        return 0
    else:
        return 1-t
def manoeuvre_model_rt_evolution(u, v, r, heading, rpm_0, rpm_1, rpm_2, rsa_0, rsa_1, rsa_2, dt):  #rpm in per second!
    
    u_a_2 = (1 - ship.w) * ((-u - r * abs(ship.y_2)) * np.cos(np.deg2rad(rsa_2)) + ( -v - r * abs(ship.x_2)) * np.sin(np.deg2rad(rsa_2)))
    u_a_1 = (1 - ship.w) * ((-u + r * abs(ship.y_1)) * np.cos(np.deg2rad(rsa_1)) + ( -v - r * abs(ship.x_1)) * np.sin(np.deg2rad(rsa_1))) 
    u_a_0 = (1 - ship.w) * (u * -1 * np.cos(np.deg2rad(rsa_0)) + ((-v + r * abs(ship.x_0)) * np.sin(np.deg2rad(rsa_0))))

    beta_2 = np.rad2deg(np.arctan((u_a_2) / (0.7 * np.pi * rpm_2 * ship.D_p)))
    if beta_2<0:
        beta_2 = beta_2 + 360.

    beta_1 = np.rad2deg(np.arctan((u_a_1) / (0.7 * np.pi * rpm_1 * ship.D_p)))
    if beta_1 < 0:
        beta_1 = beta_1 + 360.

    beta_0 = np.rad2deg(np.arctan((u_a_0) / (0.7 * np.pi * rpm_0 * ship.D_p)))
    if beta_0 < 0:
        beta_0 = beta_0 + 360.

    # first engine listed experiences thrust decrease, t_21 means thrust reduction ratio due to downstream flow caused by engine 1
    t_21_phi = thruster_interaction_coefficient(ship.x_1, ship.y_1, rsa_1, 25.0, 100.0, ship.x_2, ship.y_2, rsa_2)
    t_20_phi = thruster_interaction_coefficient(ship.x_0, ship.y_0, rsa_0, 25.0, 100.0, ship.x_2, ship.y_2, rsa_2)
    f_p_4Q_2 = (1 - t_21_phi) * (1 - t_20_phi) * ((1 - ship.t) * ship.beta_coef(beta_2) * 0.5 * ship.rho * (((((1 - ship.w) * u_a_2) ** 2) + (0.7 * np.pi * rpm_2 * ship.D_p) ** 2)) * np.pi / 4 * ship.D_p ** 2)

    t_12_phi = thruster_interaction_coefficient(ship.x_2, ship.y_2, rsa_2, 25.0, 100.0, ship.x_1, ship.y_1, rsa_1)
    t_10_phi = thruster_interaction_coefficient(ship.x_0, ship.y_0, rsa_0, 25.0, 100.0, ship.x_1, ship.y_1, rsa_1)
    f_p_4Q_1 = (1 - t_12_phi) * (1 - t_10_phi) * ((1 - ship.t) * ship.beta_coef(beta_1) * 0.5 * ship.rho * (
    ((((1 - ship.w) * u_a_1) ** 2) + (0.7 * np.pi * rpm_1 * ship.D_p) ** 2)) * np.pi / 4 * ship.D_p ** 2)

    t_02_phi = thruster_interaction_coefficient(ship.x_2, ship.y_2, rsa_2, 25.0, 100.0, ship.x_0, ship.y_0, rsa_0)
    t_01_phi = thruster_interaction_coefficient(ship.x_1, ship.y_1, rsa_1, 25.0, 100.0, ship.x_0, ship.y_0, rsa_0)
    f_p_4Q_0 = (1 - t_02_phi) * (1 - t_01_phi) * ((1 - ship.t) * ship.beta_coef(beta_0) * 0.5 * ship.rho * (
        ((((1 - ship.w) * u_a_0) ** 2) + (0.7 * np.pi * rpm_0 * ship.D_p) ** 2)) * np.pi / 4 * ship.D_p ** 2)


    # precalculate all values raised to powers or those with less than 5 chained multiplications (don't use numpy for this part)
    # https://stackoverflow.com/questions/18453771/why-is-x3-slower-than-xxx/18453999#18453999

    i_t = 1

    u2 = u ** 2
    u3 = u * u * u
    v2 = v ** 2
    v3 = v * v * v
    r2 = r ** 2
    r3 = r * r * r
    rvu = r * v * u
    abs_v = abs(v)
    abs_r = abs(r)
    # surge
    surge_partial_force = (surge_coeff[1] * u2 +
                           surge_coeff[2] * u3 +
                           surge_coeff[3] * v2 +
                           surge_coeff[4] * r2 +
                           surge_coeff[5] * v * r +
                           surge_coeff[6] * u * v2 +
                           surge_coeff[7] * rvu +
                           surge_coeff[8] * u * r2
                           )
    u_accel = (surge_partial_force + vessel_mass_kg_m * r * v + (2 * F_p_4Q * i_t + 2 * F_p * (1 - i_t))) / (
            vessel_mass_kg_m - surge_coeff[0])

    # print(f'u {u}, v {v}, r {r} , surge_1_5 {surge_partial_force} )

    sway_partial_force = (sway_coeff[1] * u * v +
                          sway_coeff[2] * u * r +
                          sway_coeff[3] * (u2) * r +
                          sway_coeff[4] * (u2) * v +
                          sway_coeff[5] * (v3) +
                          sway_coeff[6] * r3 +
                          sway_coeff[7] * (r2) * v +
                          sway_coeff[8] * (v2) * r +
                          sway_coeff[9] * abs_v * v +
                          sway_coeff[10] * abs_r * v +
                          sway_coeff[11] * abs_v * r +
                          sway_coeff[12] * abs_r * r
                          )
    v_accel = (sway_partial_force + F_r - vessel_mass_kg_m * r * u) / (vessel_mass_kg_m - sway_coeff[0])

    force_partial = (rot_coeff[1] * u * v +
                     rot_coeff[2] * u * r +
                     rot_coeff[3] * (u2) * r +
                     rot_coeff[4] * (u2) * v +
                     rot_coeff[5] * (v3) +
                     rot_coeff[6] * r3 +
                     rot_coeff[7] * (r2) * v +
                     rot_coeff[8] * (v2) * r +
                     rot_coeff[9] * abs_v * v +
                     rot_coeff[10] * abs_r * v +
                     rot_coeff[11] * abs_v * r +
                     rot_coeff[12] * abs_r * r -
                     F_r * rudder_dist_cog_m_x_r)

    r_accel = force_partial / (moi_kgm3_I_zz - rot_coeff[0])

    # Calculating distance covered during this time iterval (using velocity from last interval)
    delta_x_0 = (u * np.cos(np.deg2rad(heading)) - v * np.sin(np.deg2rad(heading))) * dt
    delta_y_0 = (v * np.cos(np.deg2rad(heading)) + u * np.sin(np.deg2rad(heading))) * dt
    delta_r_0 = r * dt  # radians turned in the existing time step

    # Calculating new velocities
    next_u = u + u_accel * dt
    next_v = v + v_accel * dt
    next_r = r + r_accel * dt

    return next_u, next_v, next_r, delta_x_0, delta_y_0, delta_r_0, u_accel, v_accel, r_accel