

**MEL7520**

**Dr. B. Ravindra**

Railway Active Vehicle Suspension

**Anshul Kulhari**

**B17ME016**

Method

In comparison to passive suspensions, the LQR controller designed will allow the active

suspension system to dynamically adjust in different operating conditions without

compromising rail handling capacity or ride comfort. The active suspension system is operated

by an electrohydraulic actuator with a LQR control strategy. Simulink will be used to construct

block diagrams and simulate the systems, while MATLAB will be used to build and analyze the

state-space model for both passive and active systems.

MATLAB/Simulink

The state-space model of the railway suspension system and the LQR controller for the active

suspension system will be developed using MATLAB software, which will also house the model

parameters that will be used in conjunction with Simulink. When it comes to the analysis of

experimental results, MATLAB is a common program for theoretical calculations.When it comes

to analyzing the suspension parameters within the simulation model, the MATLAB environment

can come in handy. For comparison and discussion, the results obtained with MATLAB will be

used. Simulink is a program that allows you to simulate, analyze, and model complex structures.

The program can be used to study the behavior of real-world dynamic systems, and it will be

used in this dissertation to study the railway vehicle's quarter-car model suspension system.

MATHEMATICAL MODEL

When designing a control system, the establishment of the mathematical model of the system

is needed. Like most engineering control systems, a mathematical model will typically rely on

known laws which will then be derived, Newton second law of motion will be the law focus for

the quarter car suspension model. The mathematical model's primary purpose is to provide an

equation or equations describing how the system behaves.





2

QUARTER CAR MODEL

A simpliﬁed quarter car model, as shown in Figure 1 (a), was used to analyze the parameters

related to the suspension system. For the study of the vertical vibration induced by the railway

disruption, the quarter car model was chosen because it is the most general and straightforward

of the vehicle dynamic vibration models.The railway vehicle shell, also known as the sprung

mass, and the railway vehicle bogie, also known as the unsprung mass, make up the mass of

the railway vehicle. Suspension springs and dampers attach the sprung and unsprung axles to

the track.In contrast to the suspension system's vertical deﬂections, both transversal and

longitudinal deﬂections are considered negligible. Due to the lack of a control factor in the

passive suspension system (see Figure 1), the actuator force will not be considered (b).

**Figure 1:** A quarter model for the active suspension system (a) and the passive suspension system (b)





3

Where:-

QUARTER RAIL VEHICLE MODEL DESIGN PARAMETERS

Table 1 lists the input parameter values for the railcar model's passive and active suspension

systems, which will be used to evaluate and generate performance.

**Table 1**: Quarter rail model design parameters





4

EQUATION OF MOTION

The equations of motion considered for both the passive and the active suspension systems are

derived using Newton's second law of motion.

**Equation 1**

**Equation 2**

Transposed for the acceleration, the equation now becomes:

**Equation 3**

So, from the ﬁgure above, Newton's law of motion, the dynamic equation of the active

suspension system and with the forces acting on the sprung mass, the following equation

is given:

**Equation 4**

Transposed for the sprung mass acceleration, the equation now becomes:

**Equation 5**

The forces acting on the unsprung mass are the following:

**Equation 6**

Transposed for the unsprung mass acceleration, the equation now becomes:





5

STATE SPACE REPRESENTATION

The general state-space representation is given by the following:

**Equation 7**

A state-space model describing the active suspension system will be created using the

two equations of motion derived earlier. Let the state variables representing the system

be:





6

The state-space model of the active suspension system can be easily obtained and written

in the form given in equation 7, which will then be written in matrix form as shown below using

the equations of motion contained in equations 3, 4, 5, and 6.

**Equation 8**

STATE-SPACE CONTROLLABILITY

When it comes to control systems, there are many issues to consider, such as stabilizing chaotic

systems with feedback control, and controllability is a crucial aspect to consider when

attempting to solve these issues. The system can be regulated if it has a single control input, u,

that can shift a system state around in its entire conﬁguration to another state.The

controllability matrix P must be maximum rank and equal to the n number of states in a linear

time-invariant state system to be controllable.

**Equation 9**





7

FULL STATE VARIABLE FEEDBACK CONTROL

Figure 2 shows a full state feedback controller, also known as a pole placement controller, which

is the best method for achieving desired pole positions in a closed-loop system because it

allows the controller to know all state variables at all times and provide feedback.

The state space matrix is the plant with each state variable being fed back to the control input ,u,

through the gain K represented by feedback vector, which can be adjusted to reach the desired

closed-loop pole values. So, the system input is given by:

**Equation 10**

Substituting this into equation 7 yields the closed-loop system's state space equation:

**Equation 11**

**Figure 2:** A full-state feedback block diagram





8

LINEAR QUADRATIC REGULATOR (LQR)

The LQR controller is a common form of state feedback control that provides a

systematic way of determining the control gain, K. Since it is one of the more classic

control options for linear MIMO time-invariant systems and is easy to design, the LQR

approach will be used for the controller design for the active suspension system. One of

the benefits of using a LQR controller is that the variables influencing the output index

can be weighted based on the individual's desired outcome. The LQR approach will

concentrate on improving the rail handling capacity and ride comfort of the quarter rail

model.

The primary goal of a LQR controller is to minimize the cost function, J, which is

represented by the performance index in equation 12, and to measure the optimal gain,

K:

**Equation 12**

The performance index or quadratic cost function J must be minimised by adjusting both the

weighting Q and R matrices, where Q is a diagonal positive deﬁnite and R is a positive constant.

The desired closed-loop performance is then obtained by tuning the weighting matrices, by

penalising bad performance by adjusting the Q matrix or penalising actuator effort by adjusting

the R matrix until suitable results regarding the cost function are reached for the plant.

Based on equation 10, the feedback regulator and solution and to the performance index in is

given by:

**Equation 13**





9

Meaning that both the A and B matrices must correspond to the actuator control force in the

feedback regulator, giving the matrices shown in equation 14

**Equation 14**:

LQR CONTROLLER MATLAB IMPLEMENTATION

After determining that both the A and B matrices are controllable, a LQR controller can be built

and implemented in MATLAB. The following MATLAB code must be used to represent the

system matrix A, the input matrix B, the output matrix C, and the feedforward matrix D:

**A = [ 0 1 0 -1 ;**

**-ks/ms -bs/ms 0 bs/ms;**

**0 0 0 1;**

**ks/mus bs/mus -kus/mus -(bs+bus)/mus];**

**B = [0 0 ;**

**0 1/ms ;**

**-1 0 ;**

**bus/mus -1/mus ];**

**C = [ 1 0 0 0 ;**

**-ks/ms -bs/ms 0 bs/ms ];**

**D = [0 0; 0 0; 0 0; 0 0; 0 0;**

**0 1/ms];**





10

Since the open-loop system's controllability has already been established, the quadratic

performance index's weighting matrices Q and R can now be tuned for and obtained in MATLAB.

The states related to suspension travel and railway vehicle body acceleration were the

performance metrics that were of relative signiﬁcance and needed further attention.The ﬁnal

weighting matrices Q and R are shown below, after trial and error of modifying the nonzero

elements in the Q matrix and the input weighting of the R matrix:

**Equation 15**

Using the following MATLAB command, the feedback gain vector K was obtained by entering the

Q and R weighting matrices:

**lqr( A, B(:,2), Q, R )**

**Q = diag([1760\*10^6, 11.6\*10^6, 1, 1]);**

**R = 0.01;**

**K = lqr( A, B(:,2), Q, R )**

**K =1.0e+05 \***

**1.7075**

**0.3637**

**0.7759**

**0.0052**

So, the feedback gain vector obtained is:

**Equation 16**





11

SIMULINK MODEL

The state-space model and the LQR controller can now be used to model both active and

passive suspension systems in Simulink. The active and passive suspension models' modeling

processes, as well as the functions of the various subsystems, are depicted in the block

diagrams below.

When designing the device, the ﬁrst step was to create a rail disruption that would excite both

the passive and active suspensions shown in Figure 3. The ﬁrst rail disturbance (RD1) is a

simple phase input with a 0.06-meter step disturbance height. The step block provides a step

input between two deﬁnable values at a speciﬁc time; for the simulated model, a step time of 0

seconds was selected, along with an initial value of 0.

Figure 3 also shows the second rail disturbance (RD2), which is viewed as a pulse width

modulation with square waved impulses at six-second intervals. The chosen amplitude

disturbance height was 0.1m, with a pulse width of 50% of the time (three seconds) and a phase

delay of 0.1 seconds.The second disruption was selected as the worst-case scenario for the

active suspension method, and it is used in the robustness test, which will be discussed in the

results and discussion section later in this dissertation.

The next move was to build a ﬁrst-order ﬁlter for the rail disturbances, as well as a derivative

block to determine their speeds. The railway disturbance will not actually be a sharp phase

disturbance, but rather a smoother edge disturbance with some angle, which is why a ﬁlter will

be used to smooth out the disturbance for railway vehicles.Since their derivatives will be taken

later and the simulation is generating accurate results, a ﬁrst-order ﬁlter in the form of a transfer

function was used to smooth out the results generated from both the phase input in RD1 and the

pulse width generator in RD2.





12

**Figure 3:** Rail disturbance Simulink model

A state-space block will be used to model the open-loop plant, as shown in Figure 4. The

suspension system behavior, as described in equation 7, is implemented using the state-space

block.

**Figure 4:** Simulink model of the passive suspension system

Both the active and passive suspension systems' state-space models are divided into two

subsystems, as shown in Figure 5.





13

**Figure 5:** Simulink model of both the active suspension system and the passive suspension system

The active suspension system, on the other hand, has a control function, and all state variables

output from the suspension system are fed back to the control input through the gain K,

resulting in the actuator force needed to stabilize the system shown in Figure 6.

**Figure 6:**A Simulink model inputting the actuator control force into the active suspension subsystem





14

Workspace blocks that write signal data into the workspace module in MATLAB are used to

input the suspension travel in the sprung mass acceleration results into a workspace.

Calculations were carried out in order to obtain the displacements for the sprung and unsprung

masses of the railway vehicle for the active suspension system.The suspension travel, wheel

deﬂection, and rail disturbance displacement were added together and measured.

RESULTS AND ANALYSIS

Analyze the effects of the quarter car model's passive suspension system, then examine the

reaction of the active suspension system using the state space controller. The LQR controller's

effect on the device will also be evaluated to see how important it is. The ride quality and railcar

handling will be controlled when discussing the performance of the two suspension systems,

with the railcar body acceleration, suspension travel, and wheel deﬂection, as well as the quarter

car model's body displacements, being the parameters of emphasis.

The following criteria will be compared when the two systems are compared:

• Settling Time: This is the time it takes for the machine to oscillate before it reaches a point

where it starts to ﬁt the desired value.

• The Rise Time: the time it takes for the device to reach a given percentage within the deﬁned

value and how quickly it can react.

• Overshoot: a measure of how much the device initially deviates from the desired answer.

• Steady-State Error: the distance between the desired answer and the ﬁnal error.

NOTE: The results will be produced using the mathematical quarter railcar model for the

suspension system derived in the methodology, as well as the parameters described in the

methodology. Within 10 seconds, the quarter railcar model is believed to be traveling at a

constant speed along the railway with a rail disturbance height of 0.06m.

When the phase input rail disturbance is met, Figure 7 shows the response of the force

produced by the actuator in the active suspension controller. What is noticeable is that the force

produced by the actuator is applied in the opposite direction of the sprung mass or railcar body

and overshoots; this is due to the sprung mass's tendency to travel upward.The ﬁgure shows

that the actuator's response is stable and provides an adequate response to the rail disturbance

in the form of a 0.06m phase input.





15

**Figure 7:** Time response of the force generated from the actuator

Figure 8 shows the impact of inserting a feedback LQR controller on the device in terms of

suspension movement as compared to the passive suspension in Figure 35. In comparison to

the passive suspensions, a similar overshoot of 0.05 m is present.However, as compared to the

passive system, the active suspensions have a signiﬁcantly shorter settling time of 1.86

seconds, which is a 50% reduction in the response rate for the passive suspensions, resulting in

a quicker response, improved ride comfort, and reduced vibrations for the passengers.

**Figure 8:** Time response of the suspension travel for the active suspension system





16

Figure 9 depicts the impact of acceleration on the railcar body after using a LQR controller.

As compared to the response of the passive suspension in Figure 7, a similar overshoot of 7.5

m/s2 is present, as shown in Figure 6 for the suspension travel of the active suspension

system.The machine, on the other hand, shows a 47 percent increase in ride comfort and road

handling with a settling time of 1.75 seconds.

**Figure 9:** Time response of the sprung mass acceleration for the active suspension system

Figure 10 shows the effects of using the LQR controller to control the active suspension

mechanism. The active suspension has a similar overshoot in wheel deﬂection of 0.0255m as

the passive suspension. The settling time, like the previous results for the active suspension

system, has improved signiﬁcantly to 1.74 seconds, a 42 percent increase in the response

rate.Figure 10 depicts the consequences of controlling the active suspension mechanism with

the LQR controller. The active suspension has a wheel deﬂection overshoot of 0.0255m, which is

similar to the passive suspension. The settling time has increased dramatically to 1.74 seconds,

a 42 percent improvement in the response rate, similar to the previous results for the active

suspension system.In comparison to the passive suspension, which had a longer settling period,

resulting in more signiﬁcant deﬂections and poor rail handling, the LQR controller optimized both

the amplitude and the settling time, reducing vibrations and avoiding erratic movements of the

railcar on the railway.





17

**Figure 10:** Time response of the wheel deflection for the active suspension system

Figure 11 shows the active suspensions and a contrast of the sprung and unsprung mass body

displacements, as well as the displacement of the rail disturbance. The performance response

of both the railcar body and the bogie were able to achieve a steady-state close to the same

timeframe when the LQR control method was used.As compared to the passive suspension

(Figure 12), the overshoot for the sprung mass is marginally lower at 0.0143, and the unsprung

mass does not change signiﬁcantly, but the amplitude and settling period for the unsprung mass

have been decreased signiﬁcantly to 1.85 seconds and 1.42 seconds, respectively, which is a 51

percent and 43 percent increase.In contrast to passive suspension, the active suspension

system with LQR control demonstrated a major improvement in the system's ability to minimize

the body displacement of railway vehicles, providing reasonable ride comfort for passengers.





18

**Figure 11:** Time response for the body displacements for the active suspension system

**Figure 12:** Time response of the body displacements for the passive suspension system





19

ROBUSTNESS ASSESSMENT

A robustness test is performed after the LQR controller for the active suspension system has

been designed and tested. The primary explanation for this test is that when using a LQR control

for a system, the stability margins are not guaranteed to remain in place if the system's

parameters are changed.A robustness evaluation is required to ensure that a change in the

parameters does not result in a signiﬁcant reduction in the performance of the active

suspension system.

The test also revealed that, depending on the modiﬁed parameters, the device could have some

ﬂexibility.

Table 2 below lists the parameters of the changed values. The parameter variations are likely to

be on the more extreme side, and might not be conceivable in reality, but this test is intended to

test the system's robustness and for the worst-case scenario.As a worst-case scenario, for this

robustness measure, a square signal amplitude reﬂecting a railway disturbance height of 0.1m

with a time of six seconds was used to model the rail disturbance.

**Table 2:** Modified Quarter rail vehicle design parameters





20

The feedback gain vector, K, obtained for this robustness assessment is:

The results of the robustness test are shown below:





21

CONCLUSION

The test revealed little improvement in ride comfort, with the only notable difference being in the

force results for the actuator, which revealed that 70 percent increases in force are needed to

operate the device and achieve similar results as the LQR controller with the initial

parameters.Furthermore, it demonstrates that, with the changed parameters selected, the active

suspension system, with the LQR control strategy in place, demonstrates its robustness and can

adjust relatively well to variations of parameters chosen for robustness testing.

NOTE:-Please install all the required Tool Boxes.The most important Tool Box I had to install

was **Control System Toolbox**

