import opt_mintime_traj
import numpy as np
import time
import json
import os
import trajectory_planning_helpers as tph
import copy
import matplotlib.pyplot as plt
import configparser
import pkg_resources
import helper_funcs_glob
import yaml

"""
Created by:
Alexander Heilmeier

Documentation:
This script has to be executed to generate an optimal trajectory based on a given reference track.
"""

# Read map name from config file
module = os.path.dirname(os.path.abspath(__file__))
config_file = module + "/config/params.yaml"
with open(config_file, 'r') as stream:
    parsed_yaml = yaml.safe_load(stream)
input_map = parsed_yaml["map_name"]
num_lanes = parsed_yaml["num_lanes"]


# ----------------------------------------------------------------------------------------------------------------------
# USER INPUT -----------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

# choose vehicle parameter file ----------------------------------------------------------------------------------------
file_paths = {"veh_params_file": "racecar.ini"}

# debug and plot options -----------------------------------------------------------------------------------------------
debug = True                                    # print console messages
plot_opts = {"mincurv_curv_lin": False,         # plot curv. linearization (original and solution based) (mincurv only)
             "raceline": True,                  # plot optimized path
             "imported_bounds": False,          # plot imported bounds (analyze difference to interpolated bounds)
             "raceline_curv": True,             # plot curvature profile of optimized path
             "racetraj_vel": True,              # plot velocity profile
             "racetraj_vel_3d": False,          # plot 3D velocity profile above raceline
             "racetraj_vel_3d_stepsize": 1.0,   # [m] vertical lines stepsize in 3D velocity profile plot
             "spline_normals": False,           # plot spline normals to check for crossings
             "mintime_plots": False}            # plot states, controls, friction coeffs etc. (mintime only)

# select track file (including centerline coordinates + track widths) --------------------------------------------------
# file_paths["track_name"] = "rounded_rectangle"                              # artificial track
# file_paths["track_name"] = "handling_track"                                 # artificial track
# file_paths["track_name"] = "berlin_2018"                                    # Berlin Formula E 2018
# file_paths["track_name"] = "modena_2019"                                    # Modena 2019

file_paths["track_name"] = input_map + "/centerline"
print("track filename: ", file_paths["track_name"])


# set import options ---------------------------------------------------------------------------------------------------
imp_opts = {"flip_imp_track": False,                # flip imported track to reverse direction
            "set_new_start": True,                 # set new starting point (changes order, not coordinates)
            "new_start": np.array([0.0, -47.0]),    # [x_m, y_m]
            "min_track_width": None,                # [m] minimum enforced track width (set None to deactivate)
            "num_laps": 1}                          # number of laps to be driven (significant with powertrain-option),
                                                    # only relevant in mintime-optimization

# set optimization type ------------------------------------------------------------------------------------------------
# 'shortest_path'       shortest path optimization
# 'mincurv'             minimum curvature optimization without iterative call
# 'mincurv_iqp'         minimum curvature optimization with iterative call
# 'mintime'             time-optimal trajectory optimization
opt_type = 'mincurv_iqp'

# set mintime specific options (mintime only) --------------------------------------------------------------------------
# tpadata:                      set individual friction map data file if desired (e.g. for varmue maps), else set None,
#                               e.g. "berlin_2018_varmue08-12_tpadata.json"
# warm_start:                   [True/False] warm start IPOPT if previous result is available for current track
# var_friction:                 [-] None, "linear", "gauss" -> set if variable friction coefficients should be used
#                               either with linear regression or with gaussian basis functions (requires friction map)
# reopt_mintime_solution:       reoptimization of the mintime solution by min. curv. opt. for improved curv. smoothness
# recalc_vel_profile_by_tph:    override mintime velocity profile by ggv based calculation (see TPH package)

mintime_opts = {"tpadata": None,
                "warm_start": False,
                "var_friction": None,
                "reopt_mintime_solution": False,
                "recalc_vel_profile_by_tph": False}

# lap time calculation table -------------------------------------------------------------------------------------------
lap_time_mat_opts = {"use_lap_time_mat": False,             # calculate a lap time matrix (diff. top speeds and scales)
                     "gg_scale_range": [0.3, 1.0],          # range of gg scales to be covered
                     "gg_scale_stepsize": 0.05,             # step size to be applied
                     "top_speed_range": [100.0, 150.0],     # range of top speeds to be simulated [in km/h]
                     "top_speed_stepsize": 5.0,             # step size to be applied
                     "file": "lap_time_matrix.csv"}         # file name of the lap time matrix (stored in "outputs")

# ----------------------------------------------------------------------------------------------------------------------
# CHECK USER INPUT -----------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

if opt_type not in ["shortest_path", "mincurv", "mincurv_iqp", "mintime"]:
    raise IOError("Unknown optimization type!")

if opt_type == "mintime" and not mintime_opts["recalc_vel_profile_by_tph"] and lap_time_mat_opts["use_lap_time_mat"]:
    raise IOError("Lap time calculation table should be created but velocity profile recalculation with TPH solver is"
                  " not allowed!")

# ----------------------------------------------------------------------------------------------------------------------
# CHECK PYTHON DEPENDENCIES --------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

# get current path
file_paths["module"] = os.path.dirname(os.path.abspath(__file__))

# read dependencies from requirements.txt
requirements_path = os.path.join(file_paths["module"], 'requirements.txt')
dependencies = []

with open(requirements_path, 'r') as fh:
    line = fh.readline()

    while line:
        dependencies.append(line.rstrip())
        line = fh.readline()

# check dependencies
pkg_resources.require(dependencies)

# ----------------------------------------------------------------------------------------------------------------------
# INITIALIZATION OF PATHS ----------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

# assemble track import path (from lane_generator outputs)
file_paths["track_file"] = os.path.join(file_paths["module"], "outputs", file_paths["track_name"] + ".csv")

# assemble friction map import paths
file_paths["tpamap"] = os.path.join(file_paths["module"], "inputs", "frictionmaps",
                                    file_paths["track_name"] + "_tpamap.csv")

if mintime_opts["tpadata"] is None:
    file_paths["tpadata"] = os.path.join(file_paths["module"], "inputs", "frictionmaps",
                                         file_paths["track_name"] + "_tpadata.json")
else:
    file_paths["tpadata"] = os.path.join(file_paths["module"], "inputs", "frictionmaps", mintime_opts["tpadata"])

# check if friction map files are existing if the var_friction option was set
if opt_type == 'mintime' \
        and mintime_opts["var_friction"] is not None \
        and not (os.path.exists(file_paths["tpadata"]) and os.path.exists(file_paths["tpamap"])):

    mintime_opts["var_friction"] = None
    print("WARNING: var_friction option is not None but friction map data is missing for current track -> Setting"
          " var_friction to None!")

# create outputs folder(s) for lane_generator
os.makedirs(file_paths["module"] + "/outputs", exist_ok=True)

if opt_type == 'mintime':
    os.makedirs(file_paths["module"] + "/outputs/mintime", exist_ok=True)

# ----------------------------------------------------------------------------------------------------------------------
# IMPORT VEHICLE DEPENDENT PARAMETERS ----------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

# load vehicle parameter file into a "pars" dict
parser = configparser.ConfigParser()
pars = {}

if not parser.read(os.path.join(file_paths["module"], "params", file_paths["veh_params_file"])):
    raise ValueError('Specified config file does not exist or is empty!')

pars["ggv_file"] = json.loads(parser.get('GENERAL_OPTIONS', 'ggv_file'))
pars["ax_max_machines_file"] = json.loads(parser.get('GENERAL_OPTIONS', 'ax_max_machines_file'))
pars["stepsize_opts"] = json.loads(parser.get('GENERAL_OPTIONS', 'stepsize_opts'))
pars["reg_smooth_opts"] = json.loads(parser.get('GENERAL_OPTIONS', 'reg_smooth_opts'))
pars["veh_params"] = json.loads(parser.get('GENERAL_OPTIONS', 'veh_params'))
pars["vel_calc_opts"] = json.loads(parser.get('GENERAL_OPTIONS', 'vel_calc_opts'))

if opt_type == 'shortest_path':
    pars["optim_opts"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'optim_opts_shortest_path'))

elif opt_type in ['mincurv', 'mincurv_iqp']:
    pars["optim_opts"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'optim_opts_mincurv'))

elif opt_type == 'mintime':
    pars["curv_calc_opts"] = json.loads(parser.get('GENERAL_OPTIONS', 'curv_calc_opts'))
    pars["optim_opts"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'optim_opts_mintime'))
    pars["vehicle_params_mintime"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'vehicle_params_mintime'))
    pars["tire_params_mintime"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'tire_params_mintime'))
    pars["pwr_params_mintime"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'pwr_params_mintime'))

    # modification of mintime options/parameters
    pars["optim_opts"]["var_friction"] = mintime_opts["var_friction"]
    pars["optim_opts"]["warm_start"] = mintime_opts["warm_start"]
    pars["vehicle_params_mintime"]["wheelbase"] = (pars["vehicle_params_mintime"]["wheelbase_front"]
                                                   + pars["vehicle_params_mintime"]["wheelbase_rear"])

# ----------------------------------------------------------------------------------------------------------------------
# CREATE OUTPUT DIRECTORY STRUCTURE BASED ON GGV AND V_MAX -----------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

# Extract GGV profile name from filename (e.g., "ggv_conservative.csv" -> "conservative")
ggv_filename = pars["ggv_file"].replace(".csv", "")
if ggv_filename.startswith("ggv_"):
    ggv_profile = ggv_filename.replace("ggv_", "")
elif ggv_filename == "ggv":
    ggv_profile = "default"
else:
    ggv_profile = ggv_filename

# Create output subdirectory name: {ggv_profile}_{v_max}
v_max = pars["veh_params"]["v_max"]
output_subdir = f"{ggv_profile}_{v_max}"

print(f"Output subdirectory: {input_map}/{output_subdir}/")

# Create path folder for final trajectory outputs
os.makedirs(os.path.join(file_paths["module"], "..", "path", input_map, output_subdir), exist_ok=True)
if opt_type == 'mintime':
    os.makedirs(os.path.join(file_paths["module"], "..", "path", input_map, output_subdir, "mintime"), exist_ok=True)

# Assemble export paths (using relative path to path directory with GGV/v_max subdirectory)
file_paths["mintime_export"] = os.path.join(file_paths["module"], "..", "path", input_map, output_subdir, "mintime")
file_paths["traj_race_export"] = os.path.join(file_paths["module"], "..", "path", input_map, output_subdir, "traj_race_cl.csv")
file_paths["lane_optimal_export"] = os.path.join(file_paths["module"], "..", "path", input_map, output_subdir, "lane_optimal.csv")

# file_paths["traj_ltpl_export"] = os.path.join(file_paths["module"], "..", "path", input_map, output_subdir, "traj_ltpl_cl.csv")
file_paths["lap_time_mat_export"] = os.path.join(file_paths["module"], "..", "path", input_map, output_subdir, lap_time_mat_opts["file"])

# ----------------------------------------------------------------------------------------------------------------------

# set import path for ggv diagram and ax_max_machines (if required)
if not (opt_type == 'mintime' and not mintime_opts["recalc_vel_profile_by_tph"]):
    file_paths["ggv_file"] = os.path.join(file_paths["module"], "inputs", "veh_dyn_info", pars["ggv_file"])
    file_paths["ax_max_machines_file"] = os.path.join(file_paths["module"], "inputs", "veh_dyn_info",
                                                      pars["ax_max_machines_file"])

# ----------------------------------------------------------------------------------------------------------------------
# IMPORT TRACK AND VEHICLE DYNAMICS INFORMATION ------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

# save start time
t_start = time.perf_counter()

# import track
reftrack_imp = helper_funcs_glob.src.import_track.import_track(imp_opts=imp_opts,
                                                               file_path=file_paths["track_file"],
                                                               width_veh=pars["veh_params"]["width"])

# import ggv and ax_max_machines (if required)
if not (opt_type == 'mintime' and not mintime_opts["recalc_vel_profile_by_tph"]):
    ggv, ax_max_machines = tph.import_veh_dyn_info.\
        import_veh_dyn_info(ggv_import_path=file_paths["ggv_file"],
                            ax_max_machines_import_path=file_paths["ax_max_machines_file"])
else:
    ggv = None
    ax_max_machines = None

# set ax_pos_safe / ax_neg_safe / ay_safe if required and not set in parameters file
if opt_type == 'mintime' and pars["optim_opts"]["safe_traj"] \
        and (pars["optim_opts"]["ax_pos_safe"] is None
             or pars["optim_opts"]["ax_neg_safe"] is None
             or pars["optim_opts"]["ay_safe"] is None):

    # get ggv if not available
    if ggv is None:
        ggv = tph.import_veh_dyn_info. \
            import_veh_dyn_info(ggv_import_path=file_paths["ggv_file"],
                                ax_max_machines_import_path=file_paths["ax_max_machines_file"])[0]

    # limit accelerations
    if pars["optim_opts"]["ax_pos_safe"] is None:
        pars["optim_opts"]["ax_pos_safe"] = np.amin(ggv[:, 1])
    if pars["optim_opts"]["ax_neg_safe"] is None:
        pars["optim_opts"]["ax_neg_safe"] = -np.amin(ggv[:, 1])
    if pars["optim_opts"]["ay_safe"] is None:
        pars["optim_opts"]["ay_safe"] = np.amin(ggv[:, 2])

# ----------------------------------------------------------------------------------------------------------------------
# PREPARE REFTRACK -----------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

reftrack_interp, normvec_normalized_interp, a_interp, coeffs_x_interp, coeffs_y_interp = \
    helper_funcs_glob.src.prep_track.prep_track(reftrack_imp=reftrack_imp,
                                                reg_smooth_opts=pars["reg_smooth_opts"],
                                                stepsize_opts=pars["stepsize_opts"],
                                                debug=debug,
                                                min_width=imp_opts["min_track_width"])

# ----------------------------------------------------------------------------------------------------------------------
# CALL OPTIMIZATION (FIRST: TRUE OPTIMAL RACELINE) ---------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------
# First optimization: Use tight width_opt to get the TRUE globally optimal raceline

print("=" * 80)
print("FIRST OPTIMIZATION: TRUE OPTIMAL RACELINE (width_opt = %.3fm)" % pars["optim_opts"]["width_opt"])
print("=" * 80)

# if reoptimization of mintime solution is used afterwards we have to consider some additional deviation in the first
# optimization
if opt_type == 'mintime' and mintime_opts["reopt_mintime_solution"]:
    w_veh_tmp = pars["optim_opts"]["width_opt"] + (pars["optim_opts"]["w_tr_reopt"] - pars["optim_opts"]["w_veh_reopt"])
    w_veh_tmp += pars["optim_opts"]["w_add_spl_regr"]
    pars_tmp = copy.deepcopy(pars)
    pars_tmp["optim_opts"]["width_opt"] = w_veh_tmp
else:
    pars_tmp = pars

# call optimization
if opt_type == 'mincurv':
    alpha_opt = tph.opt_min_curv.opt_min_curv(reftrack=reftrack_interp,
                                              normvectors=normvec_normalized_interp,
                                              A=a_interp,
                                              kappa_bound=pars["veh_params"]["curvlim"],
                                              w_veh=pars["optim_opts"]["width_opt"],
                                              print_debug=debug,
                                              plot_debug=plot_opts["mincurv_curv_lin"])[0]

elif opt_type == 'mincurv_iqp':
    alpha_opt, reftrack_interp, normvec_normalized_interp = tph.iqp_handler.\
        iqp_handler(reftrack=reftrack_interp,
                    normvectors=normvec_normalized_interp,
                    A=a_interp,
                    kappa_bound=pars["veh_params"]["curvlim"],
                    w_veh=pars["optim_opts"]["width_opt"],
                    print_debug=debug,
                    plot_debug=plot_opts["mincurv_curv_lin"],
                    stepsize_interp=pars["stepsize_opts"]["stepsize_reg"],
                    iters_min=pars["optim_opts"]["iqp_iters_min"],
                    curv_error_allowed=pars["optim_opts"]["iqp_curverror_allowed"])

elif opt_type == 'shortest_path':
    alpha_opt = tph.opt_shortest_path.opt_shortest_path(reftrack=reftrack_interp,
                                                        normvectors=normvec_normalized_interp,
                                                        w_veh=pars["optim_opts"]["width_opt"],
                                                        print_debug=debug)

elif opt_type == 'mintime':
    # reftrack_interp, a_interp and normvec_normalized_interp are returned for the case that non-regular sampling was
    # applied
    alpha_opt, v_opt, reftrack_interp, a_interp_tmp, normvec_normalized_interp = opt_mintime_traj.src.opt_mintime.\
        opt_mintime(reftrack=reftrack_interp,
                    coeffs_x=coeffs_x_interp,
                    coeffs_y=coeffs_y_interp,
                    normvectors=normvec_normalized_interp,
                    pars=pars_tmp,
                    tpamap_path=file_paths["tpamap"],
                    tpadata_path=file_paths["tpadata"],
                    export_path=file_paths["mintime_export"],
                    print_debug=debug,
                    plot_debug=plot_opts["mintime_plots"])

    # replace a_interp if necessary
    if a_interp_tmp is not None:
        a_interp = a_interp_tmp

else:
    raise ValueError('Unknown optimization type!')
    # alpha_opt = np.zeros(reftrack_interp.shape[0])

# ----------------------------------------------------------------------------------------------------------------------
# REOPTIMIZATION OF THE MINTIME SOLUTION -------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

if opt_type == 'mintime' and mintime_opts["reopt_mintime_solution"]:

    # get raceline solution of the time-optimal trajectory
    raceline_mintime = reftrack_interp[:, :2] + np.expand_dims(alpha_opt, 1) * normvec_normalized_interp

    # calculate new track boundaries around raceline solution depending on alpha_opt values
    w_tr_right_mintime = reftrack_interp[:, 2] - alpha_opt
    w_tr_left_mintime = reftrack_interp[:, 3] + alpha_opt

    # create new reference track around the raceline
    racetrack_mintime = np.column_stack((raceline_mintime, w_tr_right_mintime, w_tr_left_mintime))

    # use spline approximation a second time
    reftrack_interp, normvec_normalized_interp, a_interp = \
        helper_funcs_glob.src.prep_track.prep_track(reftrack_imp=racetrack_mintime,
                                                    reg_smooth_opts=pars["reg_smooth_opts"],
                                                    stepsize_opts=pars["stepsize_opts"],
                                                    debug=False,
                                                    min_width=imp_opts["min_track_width"])[:3]

    # set artificial track widths for reoptimization
    w_tr_tmp = 0.5 * pars["optim_opts"]["w_tr_reopt"] * np.ones(reftrack_interp.shape[0])
    racetrack_mintime_reopt = np.column_stack((reftrack_interp[:, :2], w_tr_tmp, w_tr_tmp))

    # call mincurv reoptimization
    alpha_opt = tph.opt_min_curv.opt_min_curv(reftrack=racetrack_mintime_reopt,
                                              normvectors=normvec_normalized_interp,
                                              A=a_interp,
                                              kappa_bound=pars["veh_params"]["curvlim"],
                                              w_veh=pars["optim_opts"]["w_veh_reopt"],
                                              print_debug=debug,
                                              plot_debug=plot_opts["mincurv_curv_lin"])[0]

    # calculate minimum distance from raceline to bounds and print it
    if debug:
        raceline_reopt = reftrack_interp[:, :2] + np.expand_dims(alpha_opt, 1) * normvec_normalized_interp
        bound_r_reopt = (reftrack_interp[:, :2]
                         + np.expand_dims(reftrack_interp[:, 2], axis=1) * normvec_normalized_interp)
        bound_l_reopt = (reftrack_interp[:, :2]
                         - np.expand_dims(reftrack_interp[:, 3], axis=1) * normvec_normalized_interp)

        d_r_reopt = np.hypot(raceline_reopt[:, 0] - bound_r_reopt[:, 0], raceline_reopt[:, 1] - bound_r_reopt[:, 1])
        d_l_reopt = np.hypot(raceline_reopt[:, 0] - bound_l_reopt[:, 0], raceline_reopt[:, 1] - bound_l_reopt[:, 1])

        print("INFO: Mintime reoptimization: minimum distance to right/left bound: %.2fm / %.2fm"
              % (np.amin(d_r_reopt) - pars["veh_params"]["width"] / 2,
                 np.amin(d_l_reopt) - pars["veh_params"]["width"] / 2))

# ----------------------------------------------------------------------------------------------------------------------
# SECOND OPTIMIZATION (FOR AVOIDANCE LANES) ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------
# Second optimization: Use wider avoidance_opt to get boundaries for avoidance lanes

print("\n" + "=" * 80)
print("SECOND OPTIMIZATION: AVOIDANCE BOUNDARIES (avoidance_opt = %.3fm)" % pars["optim_opts"]["avoidance_opt"])
print("=" * 80)

# Store the first optimization results
alpha_opt_true = copy.deepcopy(alpha_opt)
reftrack_interp_true = copy.deepcopy(reftrack_interp)
normvec_normalized_interp_true = copy.deepcopy(normvec_normalized_interp)
a_interp_true = copy.deepcopy(a_interp)

# Reload original reftrack for second optimization (start from centerline again)
reftrack_interp_avoidance = helper_funcs_glob.src.prep_track.prep_track(reftrack_imp=reftrack_imp,
                                                                         reg_smooth_opts=pars["reg_smooth_opts"],
                                                                         stepsize_opts=pars["stepsize_opts"],
                                                                         debug=False,
                                                                         min_width=imp_opts["min_track_width"])[0]

# Get normvec and a_interp for second optimization
refpath_interp_cl = np.vstack((reftrack_interp_avoidance[:, :2], reftrack_interp_avoidance[0, :2]))
coeffs_x_interp_av, coeffs_y_interp_av, a_interp_avoidance, normvec_normalized_interp_avoidance = tph.calc_splines.\
    calc_splines(path=refpath_interp_cl)

# Run second optimization with avoidance_opt
if opt_type == 'mincurv':
    alpha_opt_avoidance = tph.opt_min_curv.opt_min_curv(reftrack=reftrack_interp_avoidance,
                                                         normvectors=normvec_normalized_interp_avoidance,
                                                         A=a_interp_avoidance,
                                                         kappa_bound=pars["veh_params"]["curvlim"],
                                                         w_veh=pars["optim_opts"]["avoidance_opt"],
                                                         print_debug=debug,
                                                         plot_debug=False)[0]

elif opt_type == 'mincurv_iqp':
    alpha_opt_avoidance, reftrack_interp_avoidance, normvec_normalized_interp_avoidance = tph.iqp_handler.\
        iqp_handler(reftrack=reftrack_interp_avoidance,
                    normvectors=normvec_normalized_interp_avoidance,
                    A=a_interp_avoidance,
                    kappa_bound=pars["veh_params"]["curvlim"],
                    w_veh=pars["optim_opts"]["avoidance_opt"],
                    print_debug=debug,
                    plot_debug=False,
                    stepsize_interp=pars["stepsize_opts"]["stepsize_reg"],
                    iters_min=pars["optim_opts"]["iqp_iters_min"],
                    curv_error_allowed=pars["optim_opts"]["iqp_curverror_allowed"])

elif opt_type == 'shortest_path':
    alpha_opt_avoidance = tph.opt_shortest_path.opt_shortest_path(reftrack=reftrack_interp_avoidance,
                                                                   normvectors=normvec_normalized_interp_avoidance,
                                                                   w_veh=pars["optim_opts"]["avoidance_opt"],
                                                                   print_debug=debug)
else:
    # For mintime, use same result (avoidance lanes not applicable for mintime optimization)
    alpha_opt_avoidance = alpha_opt
    reftrack_interp_avoidance = reftrack_interp
    normvec_normalized_interp_avoidance = normvec_normalized_interp

# Restore first optimization results for main trajectory
alpha_opt = alpha_opt_true
reftrack_interp = reftrack_interp_true
normvec_normalized_interp = normvec_normalized_interp_true
a_interp = a_interp_true

print("INFO: Second optimization completed")
print("      First optimization used width_opt = %.3fm (true optimal)" % pars["optim_opts"]["width_opt"])
print("      Second optimization used avoidance_opt = %.3fm (for boundaries)" % pars["optim_opts"]["avoidance_opt"])

# ----------------------------------------------------------------------------------------------------------------------
# INTERPOLATE SPLINES TO SMALL DISTANCES BETWEEN RACELINE POINTS -------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

raceline_interp, a_opt, coeffs_x_opt, coeffs_y_opt, spline_inds_opt_interp, t_vals_opt_interp, s_points_opt_interp,\
    spline_lengths_opt, el_lengths_opt_interp = tph.create_raceline.\
    create_raceline(refline=reftrack_interp[:, :2],
                    normvectors=normvec_normalized_interp,
                    alpha=alpha_opt,
                    stepsize_interp=pars["stepsize_opts"]["stepsize_interp_after_opt"])

# ----------------------------------------------------------------------------------------------------------------------
# CALCULATE HEADING AND CURVATURE --------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

# calculate heading and curvature (analytically)
psi_vel_opt, kappa_opt = tph.calc_head_curv_an.\
    calc_head_curv_an(coeffs_x=coeffs_x_opt,
                      coeffs_y=coeffs_y_opt,
                      ind_spls=spline_inds_opt_interp,
                      t_spls=t_vals_opt_interp)

# ----------------------------------------------------------------------------------------------------------------------
# CALCULATE VELOCITY AND ACCELERATION PROFILE --------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

if opt_type == 'mintime' and not mintime_opts["recalc_vel_profile_by_tph"]:
    # interpolation
    s_splines = np.cumsum(spline_lengths_opt)
    s_splines = np.insert(s_splines, 0, 0.0)
    vx_profile_opt = np.interp(s_points_opt_interp, s_splines[:-1], v_opt)

else:
    vx_profile_opt = tph.calc_vel_profile.\
        calc_vel_profile(ggv=ggv,
                         ax_max_machines=ax_max_machines,
                         v_max=pars["veh_params"]["v_max"],
                         kappa=kappa_opt,
                         el_lengths=el_lengths_opt_interp,
                         closed=True,
                         filt_window=pars["vel_calc_opts"]["vel_profile_conv_filt_window"],
                         dyn_model_exp=pars["vel_calc_opts"]["dyn_model_exp"],
                         drag_coeff=pars["veh_params"]["dragcoeff"],
                         m_veh=pars["veh_params"]["mass"])

# calculate longitudinal acceleration profile
vx_profile_opt_cl = np.append(vx_profile_opt, vx_profile_opt[0])
ax_profile_opt = tph.calc_ax_profile.calc_ax_profile(vx_profile=vx_profile_opt_cl,
                                                     el_lengths=el_lengths_opt_interp,
                                                     eq_length_output=False)

# calculate laptime
t_profile_cl = tph.calc_t_profile.calc_t_profile(vx_profile=vx_profile_opt,
                                                 ax_profile=ax_profile_opt,
                                                 el_lengths=el_lengths_opt_interp)

if plot_opts["racetraj_vel"]:
    s_points = np.cumsum(el_lengths_opt_interp[:-1])
    s_points = np.insert(s_points, 0, 0.0)

    plt.plot(s_points, vx_profile_opt)
    plt.plot(s_points, ax_profile_opt)
    plt.plot(s_points, t_profile_cl[:-1])

    plt.grid()
    plt.xlabel("distance in m")
    plt.legend(["vx in m/s", "ax in m/s2", "t in s"])

    plt.show()

# ----------------------------------------------------------------------------------------------------------------------
# CALCULATE LAP TIMES (AT DIFFERENT SCALES AND TOP SPEEDS) -------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

if lap_time_mat_opts["use_lap_time_mat"]:
    # simulate lap times
    ggv_scales = np.linspace(lap_time_mat_opts['gg_scale_range'][0],
                             lap_time_mat_opts['gg_scale_range'][1],
                             int((lap_time_mat_opts['gg_scale_range'][1] - lap_time_mat_opts['gg_scale_range'][0])
                                 / lap_time_mat_opts['gg_scale_stepsize']) + 1)
    top_speeds = np.linspace(lap_time_mat_opts['top_speed_range'][0] / 3.6,
                             lap_time_mat_opts['top_speed_range'][1] / 3.6,
                             int((lap_time_mat_opts['top_speed_range'][1] - lap_time_mat_opts['top_speed_range'][0])
                                 / lap_time_mat_opts['top_speed_stepsize']) + 1)

    # setup results matrix
    lap_time_matrix = np.zeros((top_speeds.shape[0] + 1, ggv_scales.shape[0] + 1))

    # write parameters in first column and row
    lap_time_matrix[1:, 0] = top_speeds * 3.6
    lap_time_matrix[0, 1:] = ggv_scales

    for i, top_speed in enumerate(top_speeds):
        for j, ggv_scale in enumerate(ggv_scales):
            tph.progressbar.progressbar(i*ggv_scales.shape[0] + j,
                                        top_speeds.shape[0] * ggv_scales.shape[0],
                                        prefix="Simulating laptimes ")

            ggv_mod = np.copy(ggv)
            ggv_mod[:, 1:] *= ggv_scale

            vx_profile_opt = tph.calc_vel_profile.\
                calc_vel_profile(ggv=ggv_mod,
                                 ax_max_machines=ax_max_machines,
                                 v_max=top_speed,
                                 kappa=kappa_opt,
                                 el_lengths=el_lengths_opt_interp,
                                 dyn_model_exp=pars["vel_calc_opts"]["dyn_model_exp"],
                                 filt_window=pars["vel_calc_opts"]["vel_profile_conv_filt_window"],
                                 closed=True,
                                 drag_coeff=pars["veh_params"]["dragcoeff"],
                                 m_veh=pars["veh_params"]["mass"])

            # calculate longitudinal acceleration profile
            vx_profile_opt_cl = np.append(vx_profile_opt, vx_profile_opt[0])
            ax_profile_opt = tph.calc_ax_profile.calc_ax_profile(vx_profile=vx_profile_opt_cl,
                                                                 el_lengths=el_lengths_opt_interp,
                                                                 eq_length_output=False)

            # calculate lap time
            t_profile_cl = tph.calc_t_profile.calc_t_profile(vx_profile=vx_profile_opt,
                                                             ax_profile=ax_profile_opt,
                                                             el_lengths=el_lengths_opt_interp)

            # store entry in lap time matrix
            lap_time_matrix[i + 1, j + 1] = t_profile_cl[-1]

    # store lap time matrix to file
    np.savetxt(file_paths["lap_time_mat_export"], lap_time_matrix, delimiter=",", fmt="%.3f")

# ----------------------------------------------------------------------------------------------------------------------
# DATA POSTPROCESSING --------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

# arrange data into one trajectory
trajectory_opt = np.column_stack((s_points_opt_interp,
                                  raceline_interp,
                                  psi_vel_opt,
                                  kappa_opt,
                                  vx_profile_opt,
                                  ax_profile_opt))
spline_data_opt = np.column_stack((spline_lengths_opt, coeffs_x_opt, coeffs_y_opt))

# create a closed race trajectory array
traj_race_cl = np.vstack((trajectory_opt, trajectory_opt[0, :]))
traj_race_cl[-1, 0] = np.sum(spline_data_opt[:, 0])  # set correct length

# print end time and lap time for FIRST optimization (true optimal)
print("\n" + "=" * 80)
print("FIRST OPTIMIZATION RESULTS (TRUE OPTIMAL RACELINE)")
print("=" * 80)
print("INFO: Estimated laptime: %.2fs" % t_profile_cl[-1])
print("INFO: Runtime from import to final trajectory was %.2fs" % (time.perf_counter() - t_start))

# ----------------------------------------------------------------------------------------------------------------------
# CHECK TRAJECTORY -----------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

bound1, bound2 = helper_funcs_glob.src.check_traj.\
    check_traj(reftrack=reftrack_interp,
               reftrack_normvec_normalized=normvec_normalized_interp,
               length_veh=pars["veh_params"]["length"],
               width_veh=pars["veh_params"]["width"],
               debug=debug,
               trajectory=trajectory_opt,
               ggv=ggv,
               ax_max_machines=ax_max_machines,
               v_max=pars["veh_params"]["v_max"],
               curvlim=pars["veh_params"]["curvlim"],
               mass_veh=pars["veh_params"]["mass"],
               dragcoeff=pars["veh_params"]["dragcoeff"])

# ----------------------------------------------------------------------------------------------------------------------
# EXPORT ---------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

# export race trajectory  to CSV
if "traj_race_export" in file_paths.keys():
    helper_funcs_glob.src.export_traj_race.export_traj_race(file_paths=file_paths,
                                                            traj_race=traj_race_cl)

# if requested, export trajectory including map information (via normal vectors) to CSV
if "traj_ltpl_export" in file_paths.keys():
    helper_funcs_glob.src.export_traj_ltpl.export_traj_ltpl(file_paths=file_paths,
                                                            spline_lengths_opt=spline_lengths_opt,
                                                            trajectory_opt=trajectory_opt,
                                                            reftrack=reftrack_interp,
                                                            normvec_normalized=normvec_normalized_interp,
                                                            alpha_opt=alpha_opt)

print("INFO: Finished export of trajectory:", time.strftime("%H:%M:%S"))

# save another copy of lane_optimal.csv
with open(file_paths["lane_optimal_export"], 'w') as f:
    for x, y, v in zip(trajectory_opt[:, 1], trajectory_opt[:, 2], trajectory_opt[:, 5]):
        f.write("%.3f,%.3f,%.3f\n" % (x, y, v))

# ----------------------------------------------------------------------------------------------------------------------
# EXPORT AVOIDANCE LANES (FROM SECOND OPTIMIZATION) -------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------
# Export avoidance lanes from the second optimization with avoidance_opt
# These boundaries are wider and allow for overtaking maneuvers

print("\n" + "=" * 80)
print("EXPORTING AVOIDANCE LANES")
print("=" * 80)

# Create raceline and trajectory from second optimization (avoidance)
raceline_interp_avoidance, a_opt_av, coeffs_x_opt_av, coeffs_y_opt_av, spline_inds_opt_interp_av, \
t_vals_opt_interp_av, s_points_opt_interp_av, spline_lengths_opt_av, el_lengths_opt_interp_av = \
tph.create_raceline.create_raceline(refline=reftrack_interp_avoidance[:, :2],
                                       normvectors=normvec_normalized_interp_avoidance,
                                       alpha=alpha_opt_avoidance,
                                       stepsize_interp=pars["stepsize_opts"]["stepsize_interp_after_opt"])

# Calculate heading and curvature for avoidance raceline
psi_vel_opt_av, kappa_opt_av = tph.calc_head_curv_an.\
calc_head_curv_an(coeffs_x=coeffs_x_opt_av,
                     coeffs_y=coeffs_y_opt_av,
                     ind_spls=spline_inds_opt_interp_av,
                     t_spls=t_vals_opt_interp_av)

# Calculate velocity profile for avoidance raceline
vx_profile_opt_av = tph.calc_vel_profile.\
calc_vel_profile(ggv=ggv,
                    ax_max_machines=ax_max_machines,
                    v_max=pars["veh_params"]["v_max"],
                    kappa=kappa_opt_av,
                    el_lengths=el_lengths_opt_interp_av,
                    closed=True,
                    filt_window=pars["vel_calc_opts"]["vel_profile_conv_filt_window"],
                    dyn_model_exp=pars["vel_calc_opts"]["dyn_model_exp"],
                    drag_coeff=pars["veh_params"]["dragcoeff"],
                    m_veh=pars["veh_params"]["mass"])

# Calculate normal vectors for the avoidance trajectory
trajectory_opt_av = np.column_stack((s_points_opt_interp_av,
                                    raceline_interp_avoidance,
                                    psi_vel_opt_av,
                                    kappa_opt_av,
                                    vx_profile_opt_av))

normvec_normalized_opt_av = tph.calc_normal_vectors.calc_normal_vectors(trajectory_opt_av[:, 3])

# Calculate avoidance boundaries from the second optimization raceline
# Normal vector points to the LEFT of travel direction (psi - pi/2)
# + normvec: move to the left
# - normvec: move to the right
avoidance_opt = pars["optim_opts"]["avoidance_opt"]

veh_bound_left = trajectory_opt_av[:, 1:3] + normvec_normalized_opt_av * avoidance_opt / 2
veh_bound_right = trajectory_opt_av[:, 1:3] - normvec_normalized_opt_av * avoidance_opt / 2

print("INFO: Calculating individual velocity profiles for left and right avoidance lanes...")

# Calculate velocity profile for LEFT avoidance lane
# Create splines for left boundary
veh_bound_left_cl = np.vstack((veh_bound_left, veh_bound_left[0]))
coeffs_x_left, coeffs_y_left, a_left, normvec_left = tph.calc_splines.calc_splines(path=veh_bound_left_cl)

# Calculate curvature for left boundary
el_lengths_left = np.sqrt(np.sum(np.power(np.diff(veh_bound_left_cl, axis=0), 2), axis=1))
psi_left, kappa_left = tph.calc_head_curv_num.calc_head_curv_num(path=veh_bound_left,
                                                                   el_lengths=el_lengths_left,
                                                                   is_closed=True)

# Calculate velocity profile for left boundary
vx_profile_left = tph.calc_vel_profile.calc_vel_profile(
    ggv=ggv,
    ax_max_machines=ax_max_machines,
    v_max=pars["veh_params"]["v_max"],
    kappa=kappa_left,
    el_lengths=el_lengths_left,
    closed=True,
    filt_window=pars["vel_calc_opts"]["vel_profile_conv_filt_window"],
    dyn_model_exp=pars["vel_calc_opts"]["dyn_model_exp"],
    drag_coeff=pars["veh_params"]["dragcoeff"],
    m_veh=pars["veh_params"]["mass"])

# Calculate velocity profile for RIGHT avoidance lane
# Create splines for right boundary
veh_bound_right_cl = np.vstack((veh_bound_right, veh_bound_right[0]))
coeffs_x_right, coeffs_y_right, a_right, normvec_right = tph.calc_splines.calc_splines(path=veh_bound_right_cl)

# Calculate curvature for right boundary
el_lengths_right = np.sqrt(np.sum(np.power(np.diff(veh_bound_right_cl, axis=0), 2), axis=1))
psi_right, kappa_right = tph.calc_head_curv_num.calc_head_curv_num(path=veh_bound_right,
                                                                     el_lengths=el_lengths_right,
                                                                     is_closed=True)

# Calculate velocity profile for right boundary
vx_profile_right = tph.calc_vel_profile.calc_vel_profile(
    ggv=ggv,
    ax_max_machines=ax_max_machines,
    v_max=pars["veh_params"]["v_max"],
    kappa=kappa_right,
    el_lengths=el_lengths_right,
    closed=True,
    filt_window=pars["vel_calc_opts"]["vel_profile_conv_filt_window"],
    dyn_model_exp=pars["vel_calc_opts"]["dyn_model_exp"],
    drag_coeff=pars["veh_params"]["dragcoeff"],
    m_veh=pars["veh_params"]["mass"])

# Export left avoidance lane with its own velocity profile (using relative path with GGV/v_max subdirectory)
file_paths["lane_left_export"] = os.path.join(file_paths["module"], "..", "path", input_map, output_subdir,
                                                    "lane_left.csv")
with open(file_paths["lane_left_export"], 'w') as f:
    f.write("# x_m, y_m, v_mps\n")
    for i in range(veh_bound_left.shape[0]):
        f.write("%.3f,%.3f,%.3f\n" % (veh_bound_left[i, 0], veh_bound_left[i, 1], vx_profile_left[i]))

# Export right avoidance lane with its own velocity profile (using relative path with GGV/v_max subdirectory)
file_paths["lane_right_export"] = os.path.join(file_paths["module"], "..", "path", input_map, output_subdir,
                                                     "lane_right.csv")
with open(file_paths["lane_right_export"], 'w') as f:
    f.write("# x_m, y_m, v_mps\n")
    for i in range(veh_bound_right.shape[0]):
        f.write("%.3f,%.3f,%.3f\n" % (veh_bound_right[i, 0], veh_bound_right[i, 1], vx_profile_right[i]))

print("INFO: Avoidance lanes exported successfully")
print("      Based on SECOND optimization with avoidance_opt = %.3fm" % avoidance_opt)
print("      Left lane:  %s" % file_paths["lane_left_export"])
print("      Right lane: %s" % file_paths["lane_right_export"])
print("      Offset from avoidance raceline: %.3fm (avoidance_opt/2)" % (avoidance_opt/2))
print("\nNOTE: lane_optimal.csv contains the TRUE optimal raceline from FIRST optimization (width_opt = %.3fm)" % pars["optim_opts"]["width_opt"])

# ----------------------------------------------------------------------------------------------------------------------
# PLOT RESULTS ---------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

# get bound of imported map (for reference in final plot)
bound1_imp = None
bound2_imp = None

if plot_opts["imported_bounds"]:
    # try to extract four times as many points as in the interpolated version (in order to hold more details)
    n_skip = max(int(reftrack_imp.shape[0] / (bound1.shape[0] * 4)), 1)

    _, _, _, normvec_imp = tph.calc_splines.calc_splines(path=np.vstack((reftrack_imp[::n_skip, 0:2],
                                                                         reftrack_imp[0, 0:2])))

    bound1_imp = reftrack_imp[::n_skip, :2] + normvec_imp * np.expand_dims(reftrack_imp[::n_skip, 2], 1)
    bound2_imp = reftrack_imp[::n_skip, :2] - normvec_imp * np.expand_dims(reftrack_imp[::n_skip, 3], 1)

# Plot with custom visualization including avoidance lanes
if plot_opts["raceline"]:
    # Calculate vehicle boundary points for TRUE optimal raceline (first optimization)
    normvec_normalized_opt = tph.calc_normal_vectors.calc_normal_vectors(trajectory_opt[:, 3])

    # These are just for visualization reference (not the actual avoidance lanes)
    veh_bound1_virt = trajectory_opt[:, 1:3] + normvec_normalized_opt * pars["optim_opts"]["width_opt"] / 2
    veh_bound2_virt = trajectory_opt[:, 1:3] - normvec_normalized_opt * pars["optim_opts"]["width_opt"] / 2

    veh_bound1_real = trajectory_opt[:, 1:3] + normvec_normalized_opt * pars["veh_params"]["width"] / 2
    veh_bound2_real = trajectory_opt[:, 1:3] - normvec_normalized_opt * pars["veh_params"]["width"] / 2

    point1_arrow = reftrack_interp[0, :2]
    point2_arrow = reftrack_interp[3, :2]
    vec_arrow = point2_arrow - point1_arrow

    # Plot track including ALL trajectories
    plt.figure(figsize=(14, 10))

    # Plot centerline
    plt.plot(reftrack_interp[:, 0], reftrack_interp[:, 1], "k--", linewidth=0.7, label='Centerline')

    # Plot TRUE optimal raceline (FIRST optimization) - RED
    plt.plot(trajectory_opt[:, 1], trajectory_opt[:, 2], "r-", linewidth=1.5, label='Optimal raceline (1st opt)')

    # Plot vehicle bounds for optimal raceline (cyan - for reference)
    plt.plot(veh_bound1_virt[:, 0], veh_bound1_virt[:, 1], "c:", linewidth=0.5, alpha=0.5, label='Optimal bounds (width_opt)')
    plt.plot(veh_bound2_virt[:, 0], veh_bound2_virt[:, 1], "c:", linewidth=0.5, alpha=0.5)

    # Plot AVOIDANCE LANES (SECOND optimization) - BLUE (thick)
    plt.plot(veh_bound_left[:, 0], veh_bound_left[:, 1], "b-", linewidth=2.0, label='Left avoidance (2nd opt)')
    plt.plot(veh_bound_right[:, 0], veh_bound_right[:, 1], "b-", linewidth=2.0, label='Right avoidance (2nd opt)')

    # Plot track boundaries
    plt.plot(bound1[:, 0], bound1[:, 1], "k-", linewidth=0.7, alpha=0.7, label='Track boundaries')
    plt.plot(bound2[:, 0], bound2[:, 1], "k-", linewidth=0.7, alpha=0.7)

    if plot_opts["imported_bounds"] and bound1_imp is not None and bound2_imp is not None:
        plt.plot(bound1_imp[:, 0], bound1_imp[:, 1], "y-", linewidth=0.7, alpha=0.5)
        plt.plot(bound2_imp[:, 0], bound2_imp[:, 1], "y-", linewidth=0.7, alpha=0.5)

    plt.grid()
    ax = plt.gca()
    ax.arrow(point1_arrow[0], point1_arrow[1], vec_arrow[0], vec_arrow[1],
             head_width=0.5, head_length=0.5, fc='g', ec='g')
    ax.set_aspect("equal", "datalim")
    plt.xlabel("east in m")
    plt.ylabel("north in m")
    plt.title("Trajectory Optimization Results\n" +
             "Red: True Optimal (width_opt=%.2fm) | Blue: Avoidance Lanes (avoidance_opt=%.2fm)" %
             (pars["optim_opts"]["width_opt"], pars["optim_opts"]["avoidance_opt"]))
    plt.legend(loc='best')
    plt.show()

# Plot 3D visualization with velocity as z-axis
if plot_opts["raceline"]:
    from mpl_toolkits.mplot3d import Axes3D

    fig = plt.figure(figsize=(16, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Scale factors for better visualization
    scale_x = 1.0
    scale_y = 1.0
    scale_z = 0.3  # scale z axis (velocity) so it doesn't appear too stretched

    # Recast get_proj function to use scaling factors
    ax.get_proj = lambda: np.dot(Axes3D.get_proj(ax), np.diag([scale_x, scale_y, scale_z, 1.0]))

    # Plot track boundaries at z=0
    ax.plot(bound1[:, 0], bound1[:, 1], 0.0, "k-", linewidth=0.7, alpha=0.5, label='Track boundaries')
    ax.plot(bound2[:, 0], bound2[:, 1], 0.0, "k-", linewidth=0.7, alpha=0.5)

    # Plot centerline at z=0
    ax.plot(reftrack_interp[:, 0], reftrack_interp[:, 1], 0.0, "k--", linewidth=0.7, alpha=0.5, label='Centerline')

    # Plot TRUE OPTIMAL raceline with velocity (FIRST optimization) - RED
    ax.plot(trajectory_opt[:, 1], trajectory_opt[:, 2], vx_profile_opt,
            "r-", linewidth=2.5, label='Optimal raceline (1st opt)', zorder=10)

    # Plot LEFT avoidance lane with velocity (SECOND optimization) - BLUE
    ax.plot(veh_bound_left[:, 0], veh_bound_left[:, 1], vx_profile_left,
            "b-", linewidth=2.0, label='Left avoidance (2nd opt)', zorder=9)

    # Plot RIGHT avoidance lane with velocity (SECOND optimization) - GREEN
    ax.plot(veh_bound_right[:, 0], veh_bound_right[:, 1], vx_profile_right,
            "g-", linewidth=2.0, label='Right avoidance (2nd opt)', zorder=9)

    # Add vertical lines to show velocity at key points (every 2 meters)
    stepsize = 2.0  # meters
    s_cumsum = np.cumsum(el_lengths_opt_interp)
    s_cumsum = np.insert(s_cumsum, 0, 0.0)

    for s_val in np.arange(0, s_cumsum[-1], stepsize):
        idx = np.argmin(np.abs(s_cumsum - s_val))
        if idx < len(trajectory_opt):
            x_val = trajectory_opt[idx, 1]
            y_val = trajectory_opt[idx, 2]
            v_val = vx_profile_opt[idx]

            # Color based on acceleration
            if idx < len(trajectory_opt) - 1:
                if vx_profile_opt[idx+1] > vx_profile_opt[idx]:
                    col = "g"  # Accelerating
                elif vx_profile_opt[idx+1] < vx_profile_opt[idx]:
                    col = "r"  # Braking
                else:
                    col = "gray"
            else:
                col = "gray"

            ax.plot([x_val, x_val], [y_val, y_val], [0.0, v_val],
                   color=col, linewidth=0.8, alpha=0.6)

    ax.grid(True, alpha=0.3)

    # Set equal aspect ratio for x and y axes
    max_range = np.array([trajectory_opt[:, 1].max()-trajectory_opt[:, 1].min(),
                          trajectory_opt[:, 2].max()-trajectory_opt[:, 2].min()]).max() / 2.0
    mid_x = (trajectory_opt[:, 1].max()+trajectory_opt[:, 1].min()) * 0.5
    mid_y = (trajectory_opt[:, 2].max()+trajectory_opt[:, 2].min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)

    ax.set_xlabel("east in m", fontsize=10)
    ax.set_ylabel("north in m", fontsize=10)
    ax.set_zlabel("velocity in m/s", fontsize=10)
    ax.set_title("3D Trajectory Optimization Results\n" +
                "Red: True Optimal (width_opt=%.2fm) | Blue: Left Avoidance | Green: Right Avoidance (avoidance_opt=%.2fm)\n" %
                (pars["optim_opts"]["width_opt"], pars["optim_opts"]["avoidance_opt"]) +
                "Vertical lines: Green=Acceleration, Red=Braking",
                fontsize=11)
    ax.legend(loc='upper left', fontsize=9)

    # Set better viewing angle
    ax.view_init(elev=25, azim=45)

    plt.show()

# Plot other visualization options using helper function (for curvature, velocity, etc.)
if plot_opts["raceline_curv"] or plot_opts["racetraj_vel_3d"] or plot_opts["spline_normals"]:
    helper_funcs_glob.src.result_plots.result_plots(plot_opts=plot_opts,
                                                    width_veh_opt=pars["optim_opts"]["width_opt"],
                                                    width_veh_real=pars["veh_params"]["width"],
                                                    refline=reftrack_interp[:, :2],
                                                    bound1_imp=bound1_imp,
                                                    bound2_imp=bound2_imp,
                                                    bound1_interp=bound1,
                                                    bound2_interp=bound2,
                                                    trajectory=trajectory_opt)
