- 2019-07-18: Periodicity of the states and/or controls can be enforced with
              MocoPeriodicityGoals that impose equality of the initial and final
              state/control values.
- 2019-07-15: MocoGoals can be enforced either as objective terms or as endpoint
              constraints. Applying a MocoGoal as an endpoint constraint is
              supported by MocoCasADiSolver (not MocoTropterSolver).
- 2019-07-09: Cost terms can now depend on initial states/controls, not just
              final states/controls. Endpoint and integral costs are combined: a
              single cost can depend on both an integral and initial/final
              states/controls. This change is necessary to support costs like
              metabolic cost of transport, which depends on both the integral of
              metabolic rate and the difference between final and initial states
              (for distance traveled).

