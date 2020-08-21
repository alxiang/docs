# Running Trials

Once you've [started the simulator](running), you're ready to run a trial. Use the `trial_runner.py` script.

Here is an example:

    rosrun social_sim_ros trial_runner.py _num_trials:=10 _num_peds:=10 _time_limit:=60 _trial_name:=nt_outdoor_1

Parameters:

 - `_num_trials`: total number of individual a-to-b navigation routes to attempt

 - `_num_peds`: number of pedestrians to spawn in the environment during each run

 - `_time_limit`: number of seconds to wait before moving onto the next run, if the goal position is not reached by the robot

 - `_trial_name`: the name of the current trial run

    The trial runner allows testing the same initial conditions (e.g. initial people and initial robot positions). This is done by randomly initializing the initial conditions when a trial is first run. Trials are identified by their `_trial_name`. Subsequent runs of the same `_trial_name` will re-use the initial conditions from the first run. Note that if the first run has less `_num_trials` than subsequent runs, only the initial `_num_trials` will be run.

    This means that every run with unique initial conditions should have a unique `_trial_name`.
