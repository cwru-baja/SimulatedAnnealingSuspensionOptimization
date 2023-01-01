# Simulated Annealing Suspension Optimization

This program optimizes a Double A Arm Front Suspension against suspension characteristics using Simulated Annealing with a weighted square error cost function. This program was written to optimize a BAJA SAE car; however, with the correct optimization goals and constraints, this program could be used for a variety of vehicle types.

Provide the program some basics of the car geometry in a json e.g:

```json
{
    "tire_radius": 12.0,
    "kingpin_to_hub": 6.0,
    "kingpin_to_ground": 5.0,
    "front_to_back": 60.0,
    "pedalbox_width": 8.0,
    "preload_ratio": 0.5
}

```

the program is unit agnostic, but users must consistently use one unit of length throughout.

Additionally, specify a center position and radius of the search space

```json
{
    "theta":1.0,
    "beta":2.0,
    "x_one":3.0,
    "x_two":4.0,
    "x_three":5.0,
    "y_three":6.0,
    "z_three":7.0,
    "upper_y":8.0,
    "x_four":9.0,
    "x_five":10.0,
    "x_six":11.0,
    "y_six":12.0,
    "z_six":13.0,
    "travel":10
}
```

Theta and Beta are in degrees and define the angle in the xy plane from Point 1 to 2 and 4 to 5 respectively.

Lastly specify target suspension characteritic values and weights for each value

```json
{
    "ideal_wheelbase": 32,
    "ideal_track_width": 33.0,
    "ideal_scrubs": [0.123, 5.678, 7.89],
    "ideal_caster_trails": [2.3, 6.1, 1.3],
    "ideal_ground_clearance": 0.94,
    "ideal_motion_ratio": 1.0,
    "ideal_recession_travel": 7.0,
    "ideal_camber_gain": 4.0,
    "ideal_kp_size": 8.0,
    "wheelbase_weight": 0.1,
    "track_width_weight": 0.1,
    "scrub_weights": [0.1, 0.1, 0.1],
    "caster_trail_weights": [0.1, 0.1, 0.1],
    "ground_clearance_weight": 0.1,
    "motion_ratio_weight": 0.1,
    "recessional_travel_weight": 0.1,
    "camber_gain_weight": 0.1,
    "kp_size_weight": 0.1
}

```

Optionally, you can pass in an existing front geometry to compare against the optimization generated geometry.

All JSON paths are specified with command line arguments.

```bash
Usage: SuspensionOptimizationSA [-g geometry_file] [-c center_file] [-r radius_file] [-t target_characteristics_file] [-b benchmark_file]
```
