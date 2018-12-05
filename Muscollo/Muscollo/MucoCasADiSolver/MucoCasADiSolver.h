#ifndef MUSCOLLO_MUCOCASADISOLVER_H
#define MUSCOLLO_MUCOCASADISOLVER_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoCasADiSolver.h                                       *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "../osimMuscolloDLL.h"

#include "../MucoSolver.h"

class CasADiTranscription;

namespace OpenSim {

// TODO have a different class for each transcription scheme.
// TODO need functions to convert between data types
class OSIMMUSCOLLO_API MucoCasADiSolver : public MucoSolver {
    OpenSim_DECLARE_CONCRETE_OBJECT(MucoCasADiSolver, MucoSolver);
public:
    OpenSim_DECLARE_PROPERTY(num_mesh_points, int,
    "The number of mesh points for discretizing the problem (default: 100).");
    MucoCasADiSolver();

    /// @name Specifying an initial guess
    /// @{

    /// Create a guess that you can edit and then set using setGuess().
    /// The types of guesses available are:
    /// - **bounds**: variable values are the midpoint between the variables'
    ///   bounds (the value for variables with ony one bound is the specified
    ///   bound). This is the default type.
    /// - **random**: values are randomly generated within the bounds.
    /// - **time-stepping**: see MucoSolver::createGuessTimeStepping().
    /// @note Calling this method does *not* set an initial guess to be used
    /// in the solver; you must call setGuess() or setGuessFile() for that.
    /// @precondition You must have called resetProblem().
    MucoIterate createGuess(const std::string& type = "bounds") const;

    /// The number of time points in the iterate does *not* need to match
    /// `num_mesh_points`; the iterate will be interpolated to the correct size.
    /// This clears the `guess_file`, if any.
    void setGuess(MucoIterate guess);
    /// Use this convenience function if you want to choose the type of guess
    /// used, but do not want to modify it first.
    void setGuess(const std::string& type) { setGuess(createGuess(type)); }
    /// This clears any previously-set guess, if any. The file is not loaded
    /// until solving or if you call getGuess().
    /// Set to an empty string to clear the guess file.
    void setGuessFile(const std::string& file);

    /// Clear the stored guess and the `guess_file` if any.
    void clearGuess();

    /// Access the guess, loading it from the guess_file if necessary.
    /// This throws an exception if you have not set a guess (or guess file).
    /// If you have not set a guess (or guess file), this returns an empty
    /// guess, and when solving, we will generate a guess using bounds.
    const MucoIterate& getGuess() const;

    /// @}

protected:
    void resetProblemImpl(const MucoProblemRep&) const override {}
    MucoSolution solveImpl() const override;

    std::unique_ptr<CasADiTranscription> createTranscription() const;

private:

    OpenSim_DECLARE_PROPERTY(guess_file, std::string,
            "A MucoIterate file storing an initial guess.");

    void constructProperties();

    // When a copy of the solver is made, we want to keep any guess specified
    // by the API, but want to discard anything we've cached by loading a file.
    MucoIterate m_guessFromAPI;
    mutable SimTK::ResetOnCopy<MucoIterate> m_guessFromFile;
    mutable SimTK::ReferencePtr<const MucoIterate> m_guessToUse;
};

} // namespace OpenSim


#endif // MUSCOLLO_MUCOCASADISOLVER_H
