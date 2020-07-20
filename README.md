Derived from [mdp-lib]{https://github.com/luisenp/mdp-lib}. Updated to support SSPs parsed from RDDL files.

RDDL support is provided by a modified version of the PROST planner (wrapped up as a library).

Packaged and structured to be used via Docker.

Building Docker Image & Running with Docker Compose
---------------------

This configuration expects all data files needed to live in the `files/` directory (which is bound to the container at runtime).

In the top level directory

    docker-compose build pinedalib
    PROBLEM=<problem_name> SOLVER=<name_of_solver> HEURISTIC=<name_of_heuristic> VERBOSITY=<verbosity_level> docker-compose run --abort-on-container-exit libmdprddl

Example
-------

For example, given an instance file `someinstance_mdp__1.rddl` (in a single file in files/, this impelementation is expecting a single rddl file, not separate domain and instance files), we can invoke LAO* with the HMIN heurisitic and verbose output with

    PROBLEM=someinstance_mdp__1 SOLVER=laostar HEURISTIC=hmin VERBOSITY=1000 NSIMS=10 docker-compose run libmdprddl

Additional Options
-------------------

- Values of verbosity >=1000 will also list the valid actions at each
time step and the corresponding q-values for those actions in the
state which follows that list.

- Adding the environment variable INTERACTIVE=1 (or anything non-empty)
  will cause simulations to be executed interactively at runtime (with
  the permitted actions and corresponding q-values listed).