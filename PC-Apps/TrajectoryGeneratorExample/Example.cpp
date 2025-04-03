/*
 * Rapid trajectory generation for quadrocopters
 *
 * Copyright 2014 by Mark W. Mueller <mwm@mwm.im>
 *
 * This code is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This code is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the code.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Modified the original files from https://github.com/markwmuller/RapidQuadrocopterTrajectories
 Xiangyu Wu (wuxiangyu@berkeley.edu)

Changes:
  - used template to make the codes work on MCU
*/

#include <iostream>
#include "Components/TrajectoryGenerator/TrajectoryGenerator.hpp"

using namespace std;
using namespace RapidQuadrocopterTrajectoryGenerator;

//Two simple helper function to make testing easier

template<typename Real>
const char* GetInputFeasibilityResultName(typename RapidTrajectoryGenerator<Real>::InputFeasibilityResult fr)
{
  switch(fr)
  {
  case RapidTrajectoryGenerator<Real>::InputFeasible:             return "Feasible";
  case RapidTrajectoryGenerator<Real>::InputIndeterminable:       return "Indeterminable";
  case RapidTrajectoryGenerator<Real>::InputInfeasibleThrustHigh: return "InfeasibleThrustHigh";
  case RapidTrajectoryGenerator<Real>::InputInfeasibleThrustLow:  return "InfeasibleThrustLow";
  case RapidTrajectoryGenerator<Real>::InputInfeasibleRates:      return "InfeasibleRates";
  }
  return "Unknown!";
}


template<typename Real>
const char* GetStateFeasibilityResultName(typename RapidTrajectoryGenerator<Real>::StateFeasibilityResult fr)
{
  switch(fr)
  {
  case RapidTrajectoryGenerator<Real>::StateFeasible:   return "Feasible";
  case RapidTrajectoryGenerator<Real>::StateInfeasible: return "Infeasible";
  }
  return "Unknown!";
}

int main(void)
{
  //Define the trajectory starting state:
  Vec3d pos0 = Vec3d(0, 0, 2); //position
  Vec3d vel0 = Vec3d(0, 0, 0); //velocity
  Vec3d acc0 = Vec3d(0, 0, 0); //acceleration

  //define the goal state:
  Vec3d posf = Vec3d(1, 0, 1); //position
  Vec3d velf = Vec3d(0, 0, 1); //velocity
  Vec3d accf = Vec3d(0, 9.81, 0); //acceleration

  //define the duration:
  double Tf = 3;

  float fmin = 5;//[m/s**2]
  float fmax = 25;//[m/s**2]
  float wmax = 20;//[rad/s]
  float minTimeSec = 0.02;//[s]

  //Define how gravity lies in our coordinate system
  Vec3d gravity = Vec3d(0,0,-9.81);//[m/s**2]

  //Define the state constraints. We'll only check that we don't fly into the floor:
  Vec3d floorPos = Vec3d(0,0,0);//any point on the boundary
  Vec3d floorNormal = Vec3d(0,0,1);//we want to be in this direction of the boundary

  RapidTrajectoryGenerator<double> traj(pos0, vel0, acc0, gravity);
  traj.SetGoalPosition(posf);
  traj.SetGoalVelocity(velf);
  traj.SetGoalAcceleration(accf);

  // Note: if you'd like to leave some states free, you can encode it like below.
  // Here we would be leaving the velocity in `x` (axis 0) free:
  //
  // traj.SetGoalVelocityInAxis(1,velf[1]);
  // traj.SetGoalVelocityInAxis(2,velf[2]);

  traj.Generate(Tf);

  for(int i = 0; i < 3; i++)
  {
    cout << "Axis #" << i << "\n";
    cout << "\talpha = " << traj.GetAxisParamAlpha(i);
    cout << "\tbeta = "  << traj.GetAxisParamBeta(i);
    cout << "\tgamma = " << traj.GetAxisParamGamma(i);
    cout << "\n";
  }
  cout << "Total cost = " << traj.GetCost() << "\n";
  cout << "Input feasible? " << GetInputFeasibilityResultName<double>(traj.CheckInputFeasibility(fmin,fmax,wmax,minTimeSec)) << "\n";
  cout << "Position feasible? " << GetStateFeasibilityResultName<double>(traj.CheckPositionFeasibility(floorPos, floorNormal)) << "\n";
  return 0;
}





