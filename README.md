Simulation files for experiments at http://iitg.vlab.co.in/?sub=62&brch=271

The robotic arm model is based on Dagu arm (http://www.dagurobot.com/goods.php?id=68). 

It is designed by following the instructions given at http://www.coppeliarobotics.com/helpFiles/en/tutorials.htm , please go through 'Building a clean model' tutorial to get an idea of how the robotic arm is built. Also refer http://www.coppeliarobotics.com/helpFiles/en/objects.htm to know about different objects and reasons to use them. 

For example, instead of using random shapes in dynamic configuration, we have used similar pure shapes in dynamic configuration and made the corresponding random non-dynamic shape as its child. The reason for using such a configuration is, random shapes will have extremely large number of triangles in meshes and will take long time for simulation. Please refer the links given above for more detailed instructions.

Simulations can be done using child scripts in v-rep as well as using remoteAPI clients, instructions to enable clients written using different platforms are available at http://www.coppeliarobotics.com/helpFiles/en/remoteApiClientSide.htm .

We have used remoteAPI clients written in python and octave. Since the inverse kinematics problem involves solving non-linear constrained expressions with optimizations, we have used octave solver to solve the expressions.

Octave is an open source software similar to Matlab, it can be downloaded at https://www.gnu.org/software/octave/ .

Use build files buildLin.m in linux, buildMac.m in mac and buildWin.m in windows to build remoteApi.oct for the first time.
Octave code can be used in matlab with some syntax changes at appropriate places.
