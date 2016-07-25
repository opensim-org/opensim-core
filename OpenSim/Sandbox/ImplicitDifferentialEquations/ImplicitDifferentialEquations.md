Interface for Implicit Differential Equations
=============================================


Overview
--------
OpenSim allows components to provide their dynamics as explicit differential
equations (e.g., `ydot = f(y)`), yet implicit differential equations
(e.g., `f(y, ydot) = 0`) are often more natural and have nice numerical
features (e.g., avoids divide-by-zero).


Requirements
------------

### Component users
1. Users of Components with implicit differential equations can obtain the
   residual of the implicit differential equation directly from the relevant
   Component.
2. Users of any Components can obtain the dynamics of the entire Model as
   the residual of implicit differential equations.
3. Acccessing the residual of the implicit differential equations across the
   whole model is as efficient as possible.
4. Multibody constraints are handled as additional "residuals."
5. Easy to determine which state variable each residual is associated with
   (i.e., "What's the order of the residual errors?").

### Component developers
1. Developers of Components with dynamics can optionally specify their dynamics
   using implicit differential equations.

Background
----------

### How does OpenSim currently handle state variables?

First off, there are two types of state variables:
1. *built-in*: This consists solely of a Coordinate's value `q` and speed `u`.
2. *added*: These are custom state variables like activation and fiber length.

These two state variables have the same interface, but are handled
differently internally.

#### Component user

To the user, there is no difference between built-in and added state variables.
Each Component provides the following methods for accessing state variables
values and their derivatives.

```cpp
int getNumStateVariablesAddedByComponent() const;
Array<std::string> getStateVariablesNamesAddedByComponent() const;
double getStateVariableValue(const SimTK::State&, const std::string&) const;
void setStateVariableValue(const SimTK::State&, const std::string&, double) const;
double getStateVariableDerivativeValue(const SimTK::State&, const std::string&) const;
```

The following additional member functions are recursive through subcomponents:
```cpp
int getNumStateVariables() const;
Array<std::string> getStateVariableNames() const;
SimTK::Vector getStateVariableValues(cont SimTK::State&);
void setStateVariableValues(SimTK::State&, const SimTK::Vector&);
```

#### Component developer

A Component developer implements the following methods:

```cpp
void extendAddToSystem(SimTK::MultibodySystem& sys) const {
    addStateVariable("activation");
    // Note: must still call addStateVariable() for built-in state variables.
}
void computeStateVariableDerivatives(const SimTK::State& s) const {
    // Only for added state variables.
    double e = getExcitation(s); double a = getActivation(s);
    double tau = getTimeConstant();
    setStateVariableDerivativeValue(s, "activation", (e - a) / tau);
}
// Next two are not as important for this proposal.
void extendInitStateFromProperties(SimTK::State& s) const {
    setStateVariableValue(s, "activation", get_default_activation());
}
void extendSetPropertiesFromState(const SimTK::State& s) {
    set_default_activation(getActivation(s));
}
```

(Note: the implementations above are for illustration and do not fully
conform to the requirements of the API.)

#### Simbody and OpenSim internals

I'll explain the internals from the top down.

* Simbody's integrators (for explicit differential equations) require
  `ydot = f(y)`. When one invokes
  `SimTK::System::realize(state, SimTK::Stage::Acceleration)`, `ydot` is cached
  in the `SimTK::State` object and is available via `SimTK::State::getYDot()`.
* Realizing the Simbody System to the Acceleration stage invokes
  `Component::extendRealizeAcceleration(const SimTK::State&)` on each Component
  in the Model.
* This, in turn, invokes
  `Component::computeStateVariableDerivatives(const SimTK::State&)`
  on the Component, which is implemented by the specific Component, as shown
  above.
* Aside: Each state variable is managed by OpenSim through two classes: `StateVariable`
  and `StateVariableInfo`. The `StateVariable` class is abstract; there are separate
  subclasses for each built-in state variable (`CoordinateStateVariable` and `SpeedStateVariable`), and all added state variables use `AddedStateVariable`.
  The `StateVariable` class has the following interface:

  ```cpp
  double getValue(const SimTK::State&) const;
  void setValue(const SimTK::State&, double value) const;
  double getDerivative(const SimTK::State&) const;
  void setDerivative(const SimTK::State&, double deriv) const;
  ```

  * The `getValue()` and `setValue()` methods ultimately tunnel to the appropriate
    entry in the passed-in state.
  * For built-in state variables, `setDerivative()` throws an exception; Simbody
    decides the derivative for these state variables! For added state variables,
    the derivative value is stored in a cache variable
    (named, e.g., `activation_deriv`).
  * For built-in state variables, `getDerivative()` tunnels into the
    passed-in state. For added state variables, it access the value from the
    cache variable.
* Model's `computeStateVariableDerivatives()` is intended to handle the
  calculation of `udot`. It calls `realizeAcceleration()`. Though I am not sure
  this is necessary. Anyway, we could do something similar for implicit
  residuals.
* The method `setStateVariableDerivativeValue()`--invoked only for added state
  variables--internally calls `setDerivative()` on the `StateVariable`.
* Now, back in `Component::extendRealizeAcceleration()`, we have computed and
  cached the state variable derivative for all added state variables,
  and we fetch out the cached value and update the value in the passed-in
  state object (`SimTK::State.updZ(s)[zIndex] = stateVar.getDerivative(s)`).


Considerations
--------------

#### Efficiency of grabbing built-in residuals.

Simbody computes the residuals for the entire multibody system in one method
and returns it in a single vector. So it would not make sense to use a scheme
in which we have to ask each Coordinate individually for elements of that
residual vector.

#### Any single residual can depend on ydot of any state variable.

Just how any state variable derivative can depend on all the other state
variables (coupled ODEs), each residual can depend on all the other state
variable derivatives. Therefore, it is not sufficient to pass in only `adot`
when wanting the residual for activation dynamics.

#### What Stage should be required to obtain the residual?

Simbody's `calcResidualForce()` says it requires `Velocity`, but OpenSim's
`InverseDynamicsSolver` requires `Dynamics` in order to compute forces.

Proposal
--------

All states must have an explicit form, and can optionally also have an implicit
form.

Computing the residuals requires having `yDotGuess` and `lambdaGuess`. We
present two schemes for passing this information around. These two schemes
arose from a discussion with Sherm on 6 July 2016.

### Scheme A: Discrete state variables

Our goal here is to create a cache variable for the residual. Cache variables
must be recomputable from the state variables, and since the residual depends
on `yDotGuess` and `lambdaGuess`, we would need to create (discrete) state
variables to hold these quantities.
We create discrete state variables in the `State` object to hold onto
`yDotGuess` and `lambdaGuess`.
This is not my preferred scheme, as it abuses discrete state variables,
but it's easier for me to think through, since it parallels the way we handle
the explicit form, and requires fewer arguments to pass around.
A large downside is that discrete state variables will
be serialized as part of a states trajectory, but there is no need to store
an optimizer's guesses for `ydot` and `lambda`--it's a detail of the solver,
not part of the model or its state.

#### Interface for Component users

1. The most important method is the one that actually gives all the residuals
   to the user:

   ```cpp
   SimTK::Vector Model::calcImplicitResidual(const SimTK::State& s) const;
   ```

   The returned vector would contain the error in both the differential equations
   *and* in the algebraic constraint equations (e.g., `State::getUDotErr()`).
   Alternatively, this method could return two separate Vectors: one for the
   differential equations and one for the (algebraic) constraint equations.
   The guesses are expected to have been set already, perhaps by something like this:
   ```cpp
   Model::setGuesssForImplicitResidual(const SimTK::State& s,
                                       const SimTK::Vector& yDotGuess,
                                       const SimTK::Vector& lambdaGuess) const;
   ```
2. Each Component has a few methods related to its own state variables:

  ```cpp
  bool hasImplicitResidual(const std::string& stateVarName) const;
  // Do *all* state variables have an implicit form? (TODO better name)
  bool hasFullImplicitResidual() const;
  // YDot is stored in the State (need YDot for *all* state variables).
  double getStateVariableImplicitResidual(const SimTK::State&, const std::string&);
  ```

  I'm not sure this last method is well-defined. Can you say that each
  state variable has a corresponding residual?

3. The Model has a method to determine if the Model can be used in implicit
   form. At first, models with multibody constraints or prescribed motion would
   return `false`.

  ```cpp
  bool Model::isImplicitResidualAvailable() const;
  ```

  This method could take a second argument `bool userProvided` to check if
  the implicit form is a "real" implicit form or is just using the explicit form
  for a subset of the state variables.


Alternate terms to use for these methods:

1. `ImplicitDifferentialEquation`,
2. `ImplicitForm`.


#### Interface for Component developers

1. When adding the StateVariable, the developer should notify the Component
   internals that it intends to provide an implicit residual:

    ```cpp
    void extendAddToSystem(SimTK::MultibodySystem& sys) const {
        bool activationHasImplicitForm = true;
        this->_aDotGuessIndex = addStateVariable("activation", activationHasImplictForm);
        addStateVariable("foo"); // not all state vars need implicit form.
    }
    ```

   This can be used later for error checking (to ensure the developer actually
   provides the implict form) and so that users can query if an implicit form has been provided.
   A Component can provide an implicit form for only a subset of its state
   variables.

2. If the additional flag was provided above, Component developers must implement
   the following method:

    ```cpp
    void computeStateVariableImplicitResidual(const SimTK::State& s) const {
        double e = getExcitation(s); double a = getActivation(s);
        double tau = getTimeConstant();
        double adot = model.getYDotGuess(s)[this->_aDotGuessIndex];
        setStateVariableImplicitResidual(s, "activation", e - a - adot*tau);
    }
    ```
   Notice how the guess for `adot` was obtained from the model, which provides access to
   the `yDotGuess` discrete state variable. We know the index of our specific `adot` guess
   because we got an index when we added the state variable. We could alternatively
   do this look-up using string names:
   ```cpp
   double adot = this->getDerivativeGuess(s, "activation");
   ```

3. Developers still need to implement the other state-variable related methods; nothing
   different here:
    * `computeStateVariableDerivatives(const SimTK::State& s)`
    * `extendInitStateFromProperties(SimTK::State& s)`
    * `extendSetPropertiesFromState(const SimTK::State& s)`

A big question: What if a residual depends on the `yDotGuess` for a completely different 
state variable, from somewhere else in the model? How would it get the correct entry in
the `yDot` discrete state variable? Should we even allow this kind of coupling? If there
is a method like `Component::getDerivativeGuess(const SimTK::State& s, std::string name)`,
that could be used to obtain the guess from some other component. But this lookup by name
could be inefficient. To avoid the string lookup, the component could first grab the 
appropriate index with something like `int Component::getDerivativeGuessIndex(...)`.

#### OpenSim internals

As in the Background section, we start from the top.

1. What's inside of `Model::calcImplicitResidual(const SimTK::State& s)`? Well, in this
   scheme, the residual is a cache variable. So we must cache it if it is not valid. 
   This is modeled somewhat after the Model's `controlsCache`.

   TODO 
   ```cpp
   class Model : .... {
   public:
       void extendRealizeTopology(SimTK::State& s) {
           // OpenSim only supports scalar discrete state variables.
           const auto& subsys = getSystem().getDefaultSubsystem();
           idxYdotGuess = subsys.allocateDiscreteVariable(s, SimTK::Stage::TODO,
                new SimTK::Value<SimTK::Vector>(...));
           idxLambdaGuess = subsys.allocateDiscreteVariable(s, SimTK::Stage::TODO,
                new SimTK::Value<SimTK::Vector>(...));
       }
       SimTK::Vector calcImplicitResidual(const SimTK::State& s) {
           if (!this->_residualCache.isValid(s)) {
               for (const auto& comp : getComponentList()) {
                   comp.computeStateVariableImplicitResidual(s);
               }
           }
           return this->_residualCache.getValue(s);
       }
       void computeStateVariableImplicitResidual(const SimTK::State& s) {
           // Handle multibody residuals.
           qDotGuess = s.getYDotGuess()(0, s.getNQ()); // kinematics
           qResidual = qDotGuess - s.getQDot(); 
           getMatterSubsystem().calcResidualForce(s, ...); // dynamics
       }
   private:
       void extendAddToSystem(SimTK::MultibodySystem& sys) const {
           _residualsCache = Measure_<Vector>::Result(_system->updDefaultSubsystem(),
                                                      Stage::Dynamics, Stage::Acceleration);
       }
       Measure_<Vector>::Result _residualCache;
   };
   ```

2. For Components that do not provide an implicit form, we can provide a trivial
   implicit form using the explicit form: `ydot - f(y) = 0`. This can be done in
   the base Component class, similar to how
   `Component::computeStateVariableDerivatives()` throws an exception if the
   derivative for a state variable has not been set:

   ```cpp
   void computeStateVariableImplicitResidual(const SimTK::State& s) const {
       for (const StateVariable& stateVar : addedStateVariables) {
           if (!stateVar.hasImplicitForm()) {
               // Use trivial implicit form.
               double ydot = stateVar.getDerivative(s);
               double yDotGuess = stateVar.getDerivativeGuess(s);
               setStateVariableImplicitResidual(s, "activation", ydot - yDotGuess);
           } else {
               if (implictResidualNotProvided) {
                   throw Exception("Component developer forgot to provide "
                       "implicit residual.");
               }
           }
       }
   }
   ```
   Here, we see that the `StateVariable` class could know the index of its own guess
   in the `yDotGuess` discrete state variable, allowing a call like
   `stateVar.getDerivativeGuess(s)`.
3. The StateVariable class would gain:
     1. boolean for whether or not it has an implicit residual
     2. member variable for the index of its yDotGuess in the
        discrete state variable, and
     3. getter for the value of the residual.

An alternative to centralized cache variable and discrete state variable vectors,
each AddedStateVariable can create its own scalar cache for the residual and yDotGuess.

---

### Scheme B: Pass around yDotGuess and lambdaGuess as arguments everywhere

In this scheme, we follow the syntax of `SimbodyMatterSubsystem::calcResidualForce()`, which
simply requires `knownUdot` and `knownLambda` as inputs, and returns the residual (without
 caching it). This is also similar to "solver"-like methods on `SimTK::System`, like `projectQ()`.

 The biggest problem with this scheme is that acceleration- and lambda-dependent quantities
 will likely use the `udot` and `lambda` from forward dynamics rather than from `yDotGuess`
 and `lambdaGuess`. For example, it wouldn't work to compute constraint forces (to use
 in an objective function).

#### Interface for Component users

1. Rather than calling two methods to obtain the residual (first setting the guesses, then updating
   and accessing the cache variable), there's just one call:

   ```cpp
   SimTK::Vector Model::calcImplicitResidual(const SimTK::State& s,
                                             const SimTK::Vector& yDotGuess,
                                             const SimTK::Vector& lambdaGuess) const;
   ```
2. Just as with Scheme A, each Component has a few methods related to its own state variables:

   ```cpp
   bool hasImplicitResidual(const std::string& stateVarName) const;
   bool hasFullImplicitResidual() const;
   double getStateVariableImplicitResidual(const SimTK::State&, const std::string&,
                                           const SimTK::Vector& yDotGuess,
                                           const SimTK::Vector& lambdaGuess);
   ```

  The difference with Scheme A is that this last function now takes the guesses as arguments.
  TODO this last function may not be possible if there is no underlying cache variable.

3. Just as with Scheme A:

   ```cpp
   bool Model::isImplicitResidualAvailable() const;
   ```

#### Interface for Component developers

1. Same as with Scheme A:

    ```cpp
    void extendAddToSystem(SimTK::MultibodySystem& sys) const {
        bool activationHasImplicitForm = true;
        this->_aDotGuessIndex = addStateVariable("activation", activationHasImplictForm);
        addStateVariable("foo"); // not all state vars need implicit form.
    }
    ```

2. The method for providing the implicit residual would be different:

   ```cpp
   void computeStateVariableImplicitResidual(const SimTK::State& s,
                                             const SimTK::Vector& yDotGuess,
                                             const SimTK::Vector& lambdaGuess,
                                             SimTK::VectorView& residual) {
      double e = getExcitation(e); double a = getActivation(s);
      double tau = getTimeConstant();
      double adot = yDotGuess[this->_aDotGuessIndex]; 
      // or this->getDerivativeGuess(yDotGuess, "activation");
      residual[0] = e - a - adot*tau;
   }
   ```

   Two main things to discuss:
     1. How to get the desired entry out of yDotGuess? `addStateVariable` can give back an index
        into this vector, or one could call a method that uses a string lookup and accesses
        the correct element internally.
     2. How do we collect all the residuals? The current proposal is that this method is provided
        with a view into the vector of all the residuals, and must know the proper local indices
        to use for storing the residuals. The size of `residual` is chosen based on how many calls
        to `addStateVariable()` have a `true` value for `hasImplicitForm`.

#### OpenSim internals

1. What's inside of `Model::calcImplicitResidual(const SimTK::State& s)`? No cache or discrete
   state variables this time.

   ```cpp
   class Model : .... {
   public:
       SimTK::Vector calcImplicitResidual(const SimTK::State& s,
                                          const SimTK::Vector& yDotGuess,
                                          const SimTK::Vector& lambdaGuess) {
           SimTK::Vector residual(s.getNY()); int count = 0;
           for (const auto& comp : getComponentList()) {
               int numImplicit = comp.getNumImplicitResidual();
               comp.computeStateVariableImplicitResidual(s, yDotGuess, lambdaGuess,
                    residual.updElt(count, numImplicit));
               count += numImplicit;
           }
       }
       void computeStateVariableImplicitResidual(const SimTK::State& s,
                                          const SimTK::Vector& yDotGuess,
                                          const SimTK::Vector& lambdaGuess,
                                          SimTK::VectorView& residual) {
           // Handle multibody residuals.
           qDotGuess = s.getYDotGuess()(0, s.getNQ()); // kinematics
           residual.updElt(0, s.getNQ()) = qDotGuess - s.getQDot(); 
           getMatterSubsystem().calcResidualForce(s, ...); // dynamics
       }
   };
   ```
2. For Components that do not provide an implicit form:
    
   ```cpp
   void computeStateVariableImplicitResidual(const SimTK::State& s,
                                          const SimTK::Vector& yDotGuess,
                                          const SimTK::Vector& lambdaGuess,
                                          SimTK::VectorView& residual) const {
   for (const StateVariable& stateVar : addedStateVariables) {
       if (!stateVar.hasImplicitForm()) {
           // Use trivial implicit form.
           double ydot = stateVar.getDerivative(s);
           residual[TODO] = ydot - ydotGuess[TODO];
       } else {
           if (implictResidualNotProvided) {
               throw Exception("Component developer forgot to provide "
                   "implicit residual.");
           }
        }
    }
   ```
   The main challenge here is that the idea is that `residual` would be of the correct size,
   for each implicit residual the component can provide, but this method is supposed to 
   handle all the remaining state variables. Perhaps the explicit-only state variables
   should be handled elsewhere.


TODO note: do not need `deriv` if multibody, since can also set YDot?
TODO cannot simply set `deriv`; need ydot for all states to compute a single
residual. May need to simply register additional equations

TODO SimbodyMatterSubsystem::calcResidualForce(): need to manually set
"inconsistent" Q and U.



Again, there is a question of where the `yDotGuess` should come from. This is
especially problematic here because `ydot` is also coming from the State
object (in the case of built-in; for added ones, there is no problem
because the value comes from a cache variable); the residual would always be 0!
We could probably get away with this if we store `yDotGuess` in the State
(because only added state variables would need this special handling), but it
would be ugly.








TODO UDotErr as well as acceleration and lambda-dependent quantities (angular acceleration, 
reaction forces) currently use UDot, but they would need to use UDotGuess instead.






TODO constraints.

#### Proposal A: Model method

#### Proposal B: Separate "solver-like" class


TODO have a single cache variable for the entire residual vector?

TODO
```cpp
SimTK::Vector calcImplicitResidual(const SimTK::State& s,
                                   const SimTK::Vector& yDotGuess,
                                   const SimTK::Vector& lambaGuess) const;
```

TODO talk with Sherm 20160706:

To the integrators, they really just call a getDerivative() method, not getYDot(),
so the interface for an implicit solver would use a getResidual() method. (no need for State::getResidual()).
If Residual is a cache variable, then the yDotGuess and lambaGuess *must* be
discrete state variables (since they are independent and are needed for computing the cache).
However, logically, this doesn't sound great. The guess is really just related
to the solver.
Want a method on the model that just computes the residual, then the
solver holds onto it.
TODO the issue here is that
TODO can the differential equations depend on lambdaGuess?
TODO make a proposal that uses discrete state variables and cache variables,
and one that doesn't.




### Constraints (ramblings)

For forward-integration, constraints are satisfied at the acceleration level,
and this is sufficient because the initial values of `q` and `u` must satisfy
the position level and velocity level constraints. To account for numerical
error throughout subsequent forward integration, `q` and `u` are projected
so that the constraints remain satisfied.

Would a solver that uses implicit differential equations want to satisfy
the constraints at the acceleration level as well? It would not be sufficient
to use only position-level constraints, since
this would not handle non-holonomic constraints.

It seems that with a direct collocation method, you could specify the
constraints on the lowest level at which they are available (holonomic
constraints at the position level, non-holonomic at the velocity level).
Alternatively, one could enforce the lowest level at the first mesh point, then
enforce only the acceleration level constraints for the remaining time steps.
I think this would still cause issues of drift in later time steps (since error
correction methods like projecting would not be used with direct collocation,
 I think). Another option is to apply the constraints at *all* levels (position,
 velocity, *and* acceleration), though this may introduce too many constraints
 (the problem may not have any degrees of freedom, and the constraints may
 be so similar that they are not independent, causing ill-conditioning?).
 Nonetheless, it is probably favorable to use the acceleration level constraints
 since they are more linear (actually...nevermind; not true for direct
 collocation, since `q` is also an unknown and still appears nonlinearly
 in the velocity and acceleration level constraints).

How would a Rosenbrock method (or other forward integrators that use implicit
differential equations) handle algebraic constraints? van den Bogert 2011
presents the Rosenbrock method for ODEs but not for DAEs.

It may be that the proper way to handle constraints depends on the solver,
and thus it is not clear how `Model::calcResidual()` could handle this. Users
may need to call `calcResidual()` (with an appropriate guess for lambda),
then get the constraint violations on their own in a consistent way (
`State::getQErr()`, `State::getUErr()`, `State::getUDotErr()`).

I think, in Simbody, the `lambda` vector has `np+nv+na` constraints
(that is, position, velocity, and acceleration level constraints), so this
may make the decision for us! TODO I think lambda is still all acceleration-level constraints,
just including the ones originating from the position and velocity levels.

Obtaining `QErr` and `UErr` from a State should be straightforward, but
the `UDotErr` constraint errors depend on `UDot`--the `UDot` here should be from
`yDotGuess`, not from `State::getUDot()` (obtained by realizing to Acceleration).
It may be necessary to read the Simbody theory guide to understand this.

### Prescribed motion

The doumentation for `SimbodyMatterSubsystem::calcResidualForce()`
(with constraints) ignores prescribed motion (constraints). What do we do??


// TODO is there one residual equation for each state variable?


// TODO component-by-component; e.g., StateVariable class has another field
// for calcImpilcitResidual.
// TODO or some aggregate fashion, so that we can ignore multibody states.

TODO problem with looping through each StateVariable is inefficiency.
TODO
