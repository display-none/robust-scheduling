#include <ilsched/iloscheduler.h>

ILOSTLBEGIN


struct Problem {

    Problem(IloInt noOfMachines,
            IloInt noOfActivities,
            IloInt* processingTimes) : noOfMachines(noOfMachines),
                                       noOfActivities(noOfActivities),
                                       processingTimes(processingTimes) { }

    IloInt noOfMachines;
    IloInt noOfActivities;
    IloInt* processingTimes;
};

//IloInt saharaDurations[] = {5, 9, 2, 12, 8, 5, 9, 11, 5, 6, 5, 4, 2, 12, 8, 5, 9, 11, 5, 10};
//Problem sahara(5, 20, saharaDurations);
IloInt saharaDurations[] = {5, 9, 2, 12, 8, 5, 9, 11, 5, 6};
Problem sahara(5, 10, saharaDurations);

IloEnforcementLevel getEnforcementLevel(int argc, char** argv) {
    IloEnforcementLevel level = IloBasic;
    if (argc > 2) {
        if (!strcmp(argv[2], "low"))
            level = IloLow;
        else if (!strcmp(argv[2], "mediumLow"))
            level = IloMediumLow;
        else if (!strcmp(argv[2], "basic"))
            level = IloBasic;
        else if (!strcmp(argv[2], "mediumHigh"))
            level = IloMediumHigh;
        else if (!strcmp(argv[2], "high"))
            level = IloHigh;
        else if (!strcmp(argv[2], "extended"))
            level = IloExtended;
    }
    return level;
}

Problem getProblem(int argc, char** argv) {
    Problem problem = sahara;
    if (argc > 1) {
        if (!strcmp(argv[2], "sahara"))
            problem = sahara;
        else if (!strcmp(argv[2], "mediumLow"))
            problem = sahara;
    }
    return problem;
}

IloInt computeHorizon(IloInt noOfActivities, IloInt* processingTimes) {
    IloInt horizon = 0;
    for (IloInt i = 0; i < noOfActivities; ++i) {
        horizon += processingTimes[i];
    }
    return horizon;
}

IloModel generateModel(IloEnv env, Problem problem, IloNumVar &makespan, IloEnforcementLevel enforcementLevel) {
    IloModel model = IloModel(env);

    IloInt horizon = computeHorizon(problem.noOfActivities, problem.processingTimes);
    IloSchedulerEnv schedulerEnv(env);
    schedulerEnv.setHorizon(horizon);

    makespan = IloIntVar(env, 0, horizon);

    IloDiscreteResource machines(env, problem.noOfMachines);
    machines.setCapacityEnforcement(enforcementLevel);

    char name[128];
    for (IloInt i = 0; i < problem.noOfActivities; ++i) {
        sprintf(name, "Activity %ld ", i + 1);
        IloActivity activity(env, problem.processingTimes[i], name);
        model.add(activity.endsBefore(makespan));
        model.add(activity.requires(machines));
    }

    return model;
}

int main(int argc, char** argv) {
    try {
        IloEnv env;
        IloEnforcementLevel level = getEnforcementLevel(argc, argv);
        Problem problem = getProblem(argc, argv);
        IloNumVar makespan;
        IloModel model = generateModel(env, problem, makespan, level);
        model.add(IloMinimize(env, makespan));

        IloSolver solver(model);
//        IloGoal goal = IloRankForward(env, makespan);
        IloGoal goal = IloSetTimesForward(env, makespan, IloSelFirstActMinEndMin);

        if (solver.solve(goal)) {
            IlcScheduler scheduler(solver);
            for (IlcActivityIterator iterator(scheduler); iterator.ok(); ++iterator)
                scheduler.getSolver().out() << *iterator << endl;
            solver.printInformation();
        } else {
            solver.out() << "No solution!" << endl;
        }

        env.end();

    } catch (IloException &exc) {
        cout << exc << endl;
    }

    return 0;
}
